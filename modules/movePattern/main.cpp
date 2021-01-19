/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cstdlib>
#include <string>
#include <cmath>
#include <random>
#include <fstream>
#include <mutex>
#include <algorithm>
#include <sys/stat.h>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

#include <yarp/cv/Cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::cv;

/****************************************************************/
class PatternMover : public RFModule
{
    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;
    BufferedPort<ImageOf<PixelRgb> > gtInPort;
    BufferedPort<ImageOf<PixelRgb> > distortedInPort;
    BufferedPort<ImageOf<PixelRgb> > gtOutPort;
    BufferedPort<ImageOf<PixelRgb> > distortedOutPort;
    BufferedPort<ImageOf<PixelRgb> > leftImageInPort;
    BufferedPort<ImageOf<PixelRgb> > rightImageInPort;
    BufferedPort<ImageOf<PixelRgb> > leftImageOutPort;
    BufferedPort<ImageOf<PixelRgb> > rightImageOutPort;
    RpcClient calibPort;
    RpcServer rpcPort;

    ResourceFinder rf;
    string moduleName;
    double period;
    int nimages;
    bool random,autostart;
    double maxx,maxy,maxz,maxangle;

    Matrix pose;
    int count;
    const double square_size=0.030;
    const int nhor=8;
    const double nver=6;
    const double board_width=nhor*square_size;
    const double board_height=nver*square_size;
    bool starting,save_result,create_test;
    int dirindex;
    PolyDriver clientGazeCtrl;
    IGazeControl *igaze;
    string outdir;
    ofstream outfile;
    bool start_calibrating;

    mutex mtx;

public:

    /****************************************************************/
    bool configure(ResourceFinder& rf) override
    {
        this->rf=rf;
        moduleName=rf.check("name", Value("movePattern"), "module name (string)").asString();
        setName(moduleName.c_str());
        period=rf.check("period",Value(2.0)).asDouble();
        nimages=rf.check("nimages",Value(30)).asInt();
        random=rf.check("random",Value(true)).asBool();
        autostart=rf.check("autostart",Value(false)).asBool();
        save_result=rf.check("save",Value(true)).asBool();
        create_test=rf.check("create_test",Value(false)).asBool();
        maxx=rf.check("maxx",Value(0.06)).asDouble();
        maxy=rf.check("maxy",Value(0.05)).asDouble();
        maxz=rf.check("maxz",Value(0.0)).asDouble();
        maxangle=(M_PI/180)*rf.check("maxangle",Value(20.0)).asDouble();

        leftImageInPort.open("/"+moduleName+"/leftImage:i");
        rightImageInPort.open("/"+ moduleName + "/rightImage:i");
        gtInPort.open("/"+moduleName+"/gt:i");
        distortedInPort.open("/"+moduleName+"/distorted:i");
        gtOutPort.open("/"+moduleName+"/gt:o");
        distortedOutPort.open("/"+moduleName+"/distorted:o");
        inPort.open("/"+moduleName+"/mover:i");
        outPort.open("/"+moduleName+"/mover:o");
        leftImageOutPort.open("/"+ moduleName + "/leftImage:o");
        rightImageOutPort.open("/"+ moduleName + "/rightImage:o");
        calibPort.open("/"+moduleName+"/calib:rpc");

        rpcPort.open(("/"+getName("/rpc")).c_str());
        attach(rpcPort);

        Property options;
        options.put("device","gazecontrollerclient");
        options.put("remote","/iKinGazeCtrl");
        options.put("local","/"+moduleName+"/gaze");
        if (clientGazeCtrl.open(options))
        {
            clientGazeCtrl.view(igaze);
            yInfo()<<"Successfully connected to iKinGazeCtrl";
        }
        else
        {
            yError()<<"Unable to connect to iKinGazeCtrl";
            return false;
        }

        if (random)
        {
            pose=generateRandomPoses(nimages,maxx,maxy,maxz,maxangle);
        }
        else
        {
            pose=generateGridPoses(nimages,maxx,maxy,maxz,maxangle);
        }

        if (autostart)
        {
            count=0;
            starting=true;
            save_result=false;
        }
        else
        {
            count=1;
            starting=false;
        }
        dirindex=1;
        start_calibrating=false;

        return true;
    }

    /********************************************************/
    bool startMoving()
    {
        lock_guard<mutex> lg(mtx);

        if (save_result)
        {
            //create output folder
            char dirName[255];
            string dir=rf.getHomeContextPath().c_str();
            sprintf(dirName,"%s/%s_%i",dir.c_str(),"candidatePos",dirindex);
            outdir=dirName;
            int check=mkdir(dirName);
            if(check!=0)
            {
                yError()<<"Could not create"<<dirName;
                return false;
            }

            string fileName=outdir+"/pos.txt";
            outfile.open(fileName, ios_base::app);
            if (!outfile.is_open())
            {
                yError()<<"Could not open"<<fileName;
                return false;
            }
        }

        Vector p=pose.getRow(0);
        yInfo()<<"Moving to first position"<<p.toString()<<count;
        Bottle &poseB=outPort.prepare();
        poseB.clear();
        poseB.addDouble(p[0]);
        poseB.addDouble(p[1]);
        poseB.addDouble(p[2]);
        poseB.addDouble(p[3]);
        poseB.addDouble(p[4]);
        poseB.addDouble(p[5]);
        outPort.write();

        //start stereoCalib
        if (!start_calibrating && save_result)
        {
            Bottle cmd,rep;
            cmd.addString("start");
            if(!calibPort.write(cmd,rep))
            {
                yError()<<"Could not start stereoCalib";
                return false;
            }
            start_calibrating=true;
            string calibFile=rf.getHomeContextPath()+"/outputCalib.ini";
            yInfo()<<"We remove leftovers from previous calibration";
            if(remove(calibFile.c_str()))
            {
                yInfo()<<"Successfully removed!";
            }
        }

        starting=true;
        return true;
    }

    /********************************************************/
    bool isRunning()
    {
        lock_guard<mutex> lg(mtx);
        return starting;
    }

    /********************************************************/
    void stopMoving()
    {
        lock_guard<mutex> lg(mtx);
        starting=false;
        start_calibrating=false;
        home();
    }

    /********************************************************/
    void home()
    {
        Bottle &poseB=outPort.prepare();
        poseB.clear();
        poseB.addDouble(0.0);
        poseB.addDouble(0.0);
        poseB.addDouble(0.0);
        poseB.addDouble(0.0);
        poseB.addDouble(0.0);
        poseB.addDouble(0.0);
        outPort.write();
    }

    /****************************************************************/
    bool save(const int idx, const Vector &p, const Mat &img)
    {
        yInfo()<<"Writing"<<p.toString()<<"to file";
        Matrix chessboard_frame=getChessboardFrame();
        Matrix T0=zeros(4,4);
        Vector t0{p[0],p[1],p[2],1.0};
        Vector rpy0{p[3],p[4],p[5]};
        T0.setSubmatrix(rpy2dcm(rpy0),0,0);
        T0.setCol(3,t0);
        Vector c0=T0.getCol(3);

        Matrix T1=chessboard_frame*T0;
        T1(2,3)-=0.63;

        yarp::sig::Vector axx=T0.subcol(0,0,3);
        yarp::sig::Vector axy=T0.subcol(0,1,3);

        c0[0]=-1.0*c0[0];
        c0[1]=-1.0*c0[1];
        yarp::sig::Vector p1=-(board_width/2.0)*axx-(square_size/2.0);
        yarp::sig::Vector p2=+(board_height/2.0)*axy+(square_size/2.0);
        yarp::sig::Vector p3=+(board_width/2.0)*axx+(square_size/2.0);
        yarp::sig::Vector p4=-(board_height/2.0)*axy-(square_size/2.0);
        yarp::sig::Vector depth=c0;

        Vector tl(4,1.0);
        Vector tl_px(2);
        tl[0]=p1[0];
        tl[1]=p2[1];
        tl[2]=depth[2];
        tl=T1*tl;
        igaze->get2DPixel(0,tl,tl_px);

        Vector tr(4,1.0);
        Vector tr_px(2);
        tr[0]=p3[0];
        tr[1]=p2[1];
        tr[2]=depth[2];
        tr=T1*tr;
        igaze->get2DPixel(0,tr,tr_px);

        Vector bl(4,1.0);
        Vector bl_px(2);
        bl[0]=p1[0];
        bl[1]=p4[1];
        bl[2]=depth[2];
        bl=T1*bl;
        igaze->get2DPixel(0,bl,bl_px);

        Vector br(4,1.0);
        Vector br_px(2);
        br[0]=p3[0];
        br[1]=p4[1];
        br[2]=depth[2];
        br=T1*br;
        igaze->get2DPixel(0,br,br_px);

        //the image is reversed on x,y:
        //BR => TL
        //BL => TR
        //TR => BL
        //TL => BR
        string imgname="img"+to_string(idx)+".png";
        outfile<<"("<<br_px[0]<<" "<<br_px[1]<<" "<<bl_px[0]<<" "<<bl_px[1]<<") "
              <<"("<<tr_px[0]<<" "<<tr_px[1]<<" "<<tl_px[0]<<" "<<tl_px[1]<<") "
             <<"("<<imgname<<")"<<endl;

        //save image
        yInfo()<<"Saving image"<<idx;
        imwrite(outdir+"/"+imgname, img);

    }

    /****************************************************************/
    Matrix generateRandomPoses(const int nposes, const double &maxx, const double &maxy,
                               const double &maxz, const double &maxangle)
    {
        random_device rnd_device;
        mt19937 mersenne_engine(rnd_device());
        uniform_real_distribution<double> dx(-maxx, maxx);
        uniform_real_distribution<double> dy(-maxy, maxy);
        uniform_real_distribution<double> dz(-maxz, maxz);
        uniform_real_distribution<double> droll(-maxangle, maxangle);
        uniform_real_distribution<double> dpitch(-maxangle, maxangle);
        uniform_real_distribution<double> dyaw(-maxangle, maxangle);
        pose.resize(nposes, 6);
        pose.zero();
        for (size_t i=0; i<nposes; i++)
        {
            double x = dx(mersenne_engine);
            double y = dy(mersenne_engine);
            double z = dz(mersenne_engine);
            double roll = droll(mersenne_engine);
            double pitch = dpitch(mersenne_engine);
            double yaw = dyaw(mersenne_engine);
            pose[i][0]=x;
            pose[i][1]=y;
            pose[i][2]=z;
            pose[i][3]=roll;
            pose[i][4]=pitch;
            pose[i][5]=yaw;
        }
        return pose;
    }

    /****************************************************************/
    Matrix generateGridPoses(const int nposes, const double &maxx, const double &maxy,
                             const double &maxz, const double &maxangle)
    {
        random_device rnd_device;
        mt19937 mersenne_engine(rnd_device());
        uniform_real_distribution<double> dz(-maxz, maxz);
        uniform_real_distribution<double> droll(-maxangle, maxangle);
        uniform_real_distribution<double> dpitch(-maxangle, maxangle);
        uniform_real_distribution<double> dyaw(-maxangle, maxangle);
        pose.resize(nposes, 6);
        pose.zero();
        double maxheigth=maxy+square_size/2.0;
        vector<double> m2{-maxy,-maxy/2.0,0.0,maxheigth/3.0,(2.0/3.0)*maxheigth,maxheigth};
        vector<double> m1{-maxx,-maxx/2.0,0.0,maxx/2.0,maxx};
        int k=0;
        for (size_t i=0; i<nposes/5; i++)
        {
            for (size_t j=0; j<nposes/6; j++)
            {
                double x = m1[i];
                double y = m2[j];
                double z = dz(mersenne_engine);
                double roll = droll(mersenne_engine);
                double pitch = dpitch(mersenne_engine);
                double yaw = dyaw(mersenne_engine);
                pose[k][0]=x;
                pose[k][1]=y;
                pose[k][2]=z;
                pose[k][3]=roll;
                pose[k][4]=pitch;
                pose[k][5]=yaw;
                k++;
            }
        }
        return pose;
    }

    /****************************************************************/
    Matrix getChessboardFrame()
    {
        Matrix T;
        if (auto* b = inPort.read())
        {
            if (b->size() > 0)
            {
                Bottle *m=b->get(0).asList();
                int nrows=m->get(0).asInt();
                int ncols=m->get(1).asInt();
                T.resize(nrows,ncols);
                Bottle *t=m->get(2).asList();
                int k=0;
                for (size_t i=0; i<nrows; i++)
                {
                    for (size_t j=0; j<ncols; j++)
                    {
                        T[i][j]=t->get(k).asDouble();
                        k++;
                    }
                }
            }
        }
        return T;
    }


    /********************************************************/
    double getPeriod() override
    {
        return period;
    }

    /********************************************************/
    bool updateModule() override
    {
        lock_guard<mutex> lg(mtx);

        if (starting)
        {
            if (create_test)
            {
                //send images to dumpers
                ImageOf<PixelRgb> *gtImage=gtInPort.read();
                ImageOf<PixelRgb> &gtImgOut=gtOutPort.prepare();
                gtImgOut=*gtImage;
                gtOutPort.writeStrict();

                ImageOf<PixelRgb> *distortedImage=distortedInPort.read();
                ImageOf<PixelRgb> &distortedImgOut=distortedOutPort.prepare();
                distortedImgOut=*distortedImage;
                distortedOutPort.writeStrict();
            }

            if (save_result)
            {
                //send images to stereoCalib
                ImageOf<PixelRgb> *leftImage=leftImageInPort.read();
                ImageOf<PixelRgb> &leftImgOut=leftImageOutPort.prepare();
                leftImgOut=*leftImage;
                leftImageOutPort.writeStrict();

                ImageOf<PixelRgb> *rightImage=rightImageInPort.read();
                ImageOf<PixelRgb> &rightImgOut=rightImageOutPort.prepare();
                rightImgOut=*rightImage;
                rightImageOutPort.writeStrict();

                //save to output file
                save(count,pose.getRow(count-1),toCvMat(*leftImage));
            }

            //move to next position
            if (count<pose.rows())
            {
                Vector p=pose.getRow(count);
                yInfo()<<"Moving to"<<p.toString()<<count;
                Bottle &poseB=outPort.prepare();
                poseB.clear();
                poseB.addDouble(p[0]);
                poseB.addDouble(p[1]);
                poseB.addDouble(p[2]);
                poseB.addDouble(p[3]);
                poseB.addDouble(p[4]);
                poseB.addDouble(p[5]);
                outPort.write();

                count++;
            }
            else
            {
                if(save_result)
                {
                    //wait outputCalib to be created
                    //and copy it into the folder
                    string calibFile=rf.getHomeContextPath()+"/outputCalib.ini";
                    struct stat buffer;
                    double timeout=20.0;
                    double t0=Time::now();
                    while (stat(calibFile.c_str(), &buffer) != 0)
                    {
                        yWarning()<<"outputCalib is not there yet...";
                        yWarning()<<"Waiting...";
                        if ((Time::now()-t0)>timeout)
                        {
                            yWarning()<<"Timeout gone";
                            break;
                        }
                    }
                    Time::delay(5.0);
                    yInfo()<<"Now we can copy it";
                    ifstream src(calibFile, ios::binary);
                    ofstream dest(outdir+"/outputCalib.ini", ios::binary);
                    dest << src.rdbuf();

                    outfile.close();
                    start_calibrating=false;
                    dirindex++;
                }

                if (autostart)
                {
                    count=0;
                }
                else
                {
                    count=1;
                }
                starting=false;
                home();
            }
        }
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle& command, Bottle& reply) override
    {
        if (command.get(0).asString()=="start")
        {
            yInfo()<<"Starting";
            reply.addInt(startMoving());
        }
        else if (command.get(0).asString()=="isRunning")
        {
            reply.addInt(isRunning());
        }
        else if (command.get(0).asString()=="stop")
        {
            reply.addString("Stop moving pattern...");
            stopMoving();
        }
        else
        {
            return false;
        }

        return true;
    }

    /**********************************************************/
    bool close() override
    {
        inPort.close();
        leftImageInPort.close();
        rightImageInPort.close();
        gtInPort.close();
        distortedInPort.close();
        gtOutPort.close();
        distortedOutPort.close();
        outPort.close();
        leftImageOutPort.close();
        rightImageOutPort.close();
        calibPort.close();
        rpcPort.close();
        return true;
    }

    /********************************************************/
    void interrupt()
    {
        inPort.interrupt();
        leftImageInPort.interrupt();
        rightImageInPort.interrupt();
        gtInPort.interrupt();
        distortedInPort.interrupt();
        gtOutPort.interrupt();
        distortedOutPort.interrupt();
        outPort.interrupt();
        leftImageOutPort.interrupt();
        rightImageOutPort.interrupt();
        calibPort.interrupt();
    }


};

/****************************************************************/
int main(int argc, char* argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "Unable to connect to YARP server";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setDefaultContext("camera-calibration-best-pos");

    PatternMover mover;
    return mover.runModule(rf);
}


/*
 * Copyright (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>
#include <yarp/cv/Cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <sstream>
#include <string>
#include <fstream>
#include <mutex>

#include "calibEvaluator_IDL.h"

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    std::string moduleName;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   undistortedInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPort;
    yarp::os::RpcClient rpcClient;

    cv::Mat imgGtMat;
    cv::Mat imgUndistortedMat;
    cv::Mat finalImage;
    int ngoodmatch;
    double percentage;
    int ntot;
    int count_skipped;
    double match_thresh;

    std::mutex mtx;

public:
    /********************************************************/

    Processing(const std::string &moduleName, const int &ntot)
    {
        this->moduleName=moduleName;
        this->ntot=ntot;
    }

    /********************************************************/
    ~Processing()
    {
    }

    /********************************************************/
    bool open(){
        
        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open("/"+moduleName+"/gtImage:i");
        undistortedInPort.open("/"+ moduleName + "/undistortedImage:i");
        outPort.open("/"+ moduleName + "/image:o");
        rpcClient.open("/"+moduleName+"/rpcClient");

        ngoodmatch=0;
        count_skipped=0;
        match_thresh=4.0;

        return true;
    }

    /********************************************************/
    void close()
    {
        undistortedInPort.close();
        outPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        undistortedInPort.interrupt();
        outPort.interrupt();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &gtImage )
    {
        std::lock_guard<std::mutex> lg(mtx);

        yarp::sig::ImageOf<yarp::sig::PixelRgb> *unDistortedImage=undistortedInPort.read();
        imgGtMat=yarp::cv::toCvMat(gtImage);
        imgUndistortedMat=yarp::cv::toCvMat(*unDistortedImage);

        //to gray as needed by compare
        cv::cvtColor(imgGtMat,imgGtMat,CV_BGR2GRAY);
        cv::cvtColor(imgUndistortedMat,imgUndistortedMat,CV_BGR2GRAY);

        //to black and white
        cv::threshold(imgGtMat, imgGtMat, 128, 255, CV_THRESH_BINARY);
        cv::threshold(imgUndistortedMat, imgUndistortedMat, 128, 255, CV_THRESH_BINARY);

        //compare images
        //first method
//        cv::compare(imgGtMat,imgUndistortedMat,finalImage,cv::CMP_EQ);

//        imwrite("gt.png", imgGtMat);
//        imwrite("undistorted.png", imgUndistortedMat);

//        //second method
//        cv::bitwise_and(imgGtMat, imgUndistortedMat, finalImage);
//        cv::bitwise_not(finalImage,finalImage);

//        imwrite("final.png", finalImage);
//        int nonZero=cv::countNonZero(finalImage);

//        cv::bitwise_not(imgGtMat,imgGtMat);
//        int nonZeroGt=cv::countNonZero(imgGtMat);
//        int totSize=imgGtMat.rows*imgGtMat.cols;
//        yDebug()<<__LINE__<<nonZero<<nonZeroGt;
////        percentage=((double)nonZeroGt/nonZero)*100.0;
//        percentage=(1.0-((double)nonZero/nonZeroGt-1.0))*100.0;

//        yInfo()<< "Got" << percentage << "% match";
//        if (percentage > 75.0)
//        {
//            ngoodmatch++;
//        }

        //third method
        cv::Size patternsize(8,6); //interior number of corners
        std::vector<cv::Point2f> gtcorners;
        std::vector<cv::Point2f> undistorted_corners;
        bool gtcorners_found = findChessboardCorners(imgGtMat, patternsize, gtcorners,
                                                     cv::CALIB_CB_ADAPTIVE_THRESH
                                                     | cv::CALIB_CB_NORMALIZE_IMAGE
                                                     | cv::CALIB_CB_FAST_CHECK);
        bool undistorted_found = findChessboardCorners(imgUndistortedMat, patternsize,
                                                       undistorted_corners,
                                                       cv::CALIB_CB_ADAPTIVE_THRESH
                                                       | cv::CALIB_CB_NORMALIZE_IMAGE
                                                       | cv::CALIB_CB_FAST_CHECK);

        if (gtcorners_found)
        {
            double stddev=std::numeric_limits<double>::infinity();
            yInfo()<<"Gt corners found";
            if(undistorted_found)
            {
                std::vector<double> d(gtcorners.size(),0.0);
                double meandist=0.0;
                yInfo()<<"Undistorted corners found";
                yInfo()<<"Let's compare!";
                for (int i=0; i<gtcorners.size(); i++)
                {
                    d[i]=cv::norm(gtcorners[i]-undistorted_corners[i]);
                    meandist+=cv::norm(gtcorners[i]-undistorted_corners[i]);
                }
                meandist/=gtcorners.size();
                stddev=0.0;
                for (int i=0; i<gtcorners.size(); i++)
                {
                    stddev+=(d[i]-meandist)*(d[i]-meandist);
                }
                stddev/=gtcorners.size();
                stddev=sqrt(stddev);
            }
            yInfo()<< "Got" << stddev << "px match";
            if (stddev <= match_thresh) //2.3
            {
                ngoodmatch++;
            }
        }
        else
        {
            yWarning()<<"Gt corners not found! Skipping";
            count_skipped++;
        }

    }

    /********************************************************/
    bool reset()
    {
        std::lock_guard<std::mutex> lg(mtx);
        ngoodmatch=0;
        count_skipped=0;
        return true;
    }

    /********************************************************/
    double getScore()
    {
        std::lock_guard<std::mutex> lg(mtx);
        yInfo()<< "Final matches" << ngoodmatch << "over" << ntot;
        return (double)ngoodmatch/(ntot-count_skipped);
    }

    /********************************************************/
    bool setThreshold(const double threshold)
    {
        std::lock_guard<std::mutex> lg(mtx);
        yInfo()<< "Setting threshold to" << threshold;
        match_thresh=threshold;
        return true;
    }


};

/********************************************************/
class Module : public yarp::os::RFModule, public calibEvaluator_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("calibEvaluator"), "module name (string)").asString();
        setName(moduleName.c_str());
        int nimages=rf.check("nimages",yarp::os::Value(100),"number of images to be tested (int)").asInt();

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;       

        processing = new Processing(moduleName,nimages);
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        rpcPort.close();
        return true;
    }

    /**********************************************************/
    bool quit() override
    {
        closing = true;
        return true;
    }

    /********************************************************/
    double getScore() override
    {
        return processing->getScore();
    }

    /********************************************************/
    bool reset() override
    {
        return processing->reset();
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }

};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.setDefaultContext("calibEvaluator");
    rf.setVerbose();
    rf.setDefaultContext(rf.getContext());
    rf.configure(argc,argv);

    return module.runModule(rf);
}
//empty line to make gcc happy

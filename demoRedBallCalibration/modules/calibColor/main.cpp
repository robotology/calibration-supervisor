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
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>
#include <yarp/cv/Cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <sstream>
#include <string>
#include <fstream>

#include "calibColor_IDL.h"

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPort;

    yarp::os::RpcClient rpcClient;

    cv::Mat imgMat;
    cv::Mat imgMatOut;

    double percentageThresh;

public:
    /********************************************************/

    Processing( const std::string &moduleName )
    {
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing()
    {
    };

    /********************************************************/
    bool open(){

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open( "/" + moduleName + "/image:i" );
        outPort.open("/" + moduleName + "/image:o");
        rpcClient.open("/"+moduleName+"/rpcClient");

        percentageThresh = 1.0;

        return true;
    }

    /********************************************************/
    void close()
    {
        outPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
    }

    /********************************************************/
    bool setPercentage(const double value)
    {
        percentageThresh = value;
        return true;
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &inImage )
    {
            
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = outPort.prepare();
        
        imgMat = yarp::cv::toCvMat(inImage);
        
        assert(imgMat.channels() == 3);
        ;
        
        float half_percent = percentageThresh / 200.0f;

        std::vector<cv::Mat> tmpsplit; split(imgMat,tmpsplit);
        for(int i=0;i<3;i++) {
            //find the low and high precentile values (based on the input percentile)
            cv::Mat flat; tmpsplit[i].reshape(1,1).copyTo(flat);
            cv::sort(flat,flat,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
            int lowval = flat.at<uchar>(cvFloor(((float)flat.cols) * half_percent));
            int highval = flat.at<uchar>(cvCeil(((float)flat.cols) * (1.0 - half_percent)));
            yDebug() << lowval << " " << highval;
            
            //saturate below the low percentile and above the high percentile
            tmpsplit[i].setTo(lowval,tmpsplit[i] < lowval);
            tmpsplit[i].setTo(highval,tmpsplit[i] > highval);
            
            //scale the channel
            normalize(tmpsplit[i],tmpsplit[i],0,255,cv::NORM_MINMAX);
        }
        merge(tmpsplit,imgMatOut);
        
        outImage.resize(imgMatOut.size().width, imgMatOut.size().height);
        outImage = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(imgMatOut);
        outPort.write();

    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public calibColor_IDL
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
        std::string moduleName = rf.check("name", yarp::os::Value("calibColor"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        processing = new Processing( moduleName);

        /* now start the thread to do the work */
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
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /**********************************************************/
    bool setPercentage(const double value)
    {
        bool returnVal = processing->setPercentage(value);
        return returnVal;
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
    rf.setDefaultContext("calibColor");
    rf.setVerbose();
    rf.setDefaultContext(rf.getContext());
    rf.configure(argc,argv);

    return module.runModule(rf);
}
//empty line to make gcc happy

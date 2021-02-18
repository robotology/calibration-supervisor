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

#include "calibSupervisor_IDL.h"

struct calibrationData
{
    cv::Point topLeft;
    cv::Point topRight;
    cv::Point bottomLeft;
    cv::Point bottomRight;
    std::string imageName;
    cv::Mat image;
    cv::Mat imageMask;
    cv::Mat resultImage;
    cv::Mat hsv;
    cv::MatND hist;
};

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   inPortRight;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPortLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPortRight;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >   dispOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >   templOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  targetPort;

    yarp::os::RpcClient rpcClient;

    cv::Mat imgMatLeft;
    cv::Mat imgMatRight;
    cv::Mat imgMat_flipped;
    cv::Mat result;
    cv::Mat finalImage;
    cv::Mat proc;
    cv::Mat testImg;

    std::vector<cv::Point2f> corners;

    std::string fileNamePath;
    std::string filePath;

    yarp::os::Bottle dataList;

    calibrationData *calibData;

    bool isFileValid;
    int totalCalibs;
    int indexCalib;
    int sendIndex;
    
    cv::Scalar colour;

    bool gotGoodMatch;
    bool completedCalibration;
    double percentage;
    bool displayOverlay;

    double percentageThresh;
    
    bool configDone;

    std::mutex mtx;

public:
    /********************************************************/

    Processing( const std::string &moduleName, const std::string &fileNamePath, const std::string &filePath )
    {
        this->moduleName = moduleName;
        this->fileNamePath = fileNamePath;
        this->filePath = filePath;
        configDone = false;
    }

    /********************************************************/
    ~Processing()
    {
    };

    /********************************************************/
    bool setPercentage(const double value)
    {
        percentageThresh = value;
        return true;
    }

    /********************************************************/
    bool changeDisplay(const std::string& value)
    {
        bool returnVal = false;

        if (value=="on")
        {
            displayOverlay = true;
            returnVal = true;
        }
        else if (value=="off")
        {
            displayOverlay = false;
            returnVal = true;
        }
        else
            yInfo() << "error setting value for landmarks";

        return returnVal;
    }

    /********************************************************/
    bool reset()
    {
        yDebug() << "resetting all values";
        indexCalib = 0;
        percentage = 0;

        gotGoodMatch = false;
        completedCalibration = false;
        displayOverlay = false;

        return true;
    }

    /********************************************************/
    void overlayImage(cv::Mat* src, cv::Mat* overlay, const cv::Point& location)
    {
        for (int y = std::max(location.y, 0); y < src->rows; ++y)
        {
            int fY = y - location.y;

            if (fY >= overlay->rows)
                break;

            for (int x = std::max(location.x, 0); x < src->cols; ++x)
            {
                int fX = x - location.x;

                if (fX >= overlay->cols)
                    break;

                double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

                for (int c = 0; opacity > 0 && c < src->channels(); ++c)
                {
                    unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
                    unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
                    src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
                }
            }
        }
    }

    /********************************************************/
    bool open(){

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open( "/" + moduleName + "/imageLeft:i" );
        inPortRight.open("/"+ moduleName + "/imageRight:i");
        outPortLeft.open("/"+ moduleName + "/imageLeft:o");
        outPortRight.open("/"+ moduleName + "/imageRight:o");
        dispOutPort.open("/" + moduleName + "/display:o");
        targetPort.open("/"+ moduleName + "/target:o");
        rpcClient.open("/"+moduleName+"/rpcClient");
        templOutPort.open("/"+moduleName+"/template:o");

        yarp::os::Network::connect("/icub/cam/left", BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::getName().c_str());
        yarp::os::Network::connect("/icub/cam/right", inPortRight.getName().c_str());
        yarp::os::Network::connect(dispOutPort.getName().c_str(), "/display");
        yarp::os::Network::connect(templOutPort.getName().c_str(), "/templImage");
        yarp::os::Network::connect(rpcClient.getName().c_str(), "/yarpdataplayer/rpc:i");

        sendIndex = -1;
        indexCalib = 0;
        totalCalibs = 0;
        percentage = 0.0;

        gotGoodMatch = false;
        completedCalibration = false;
        displayOverlay = false;

        percentageThresh = 90.0;

        colour = {0,0,255};

        yDebug() << "will now read from" << fileNamePath;

        std::ifstream infile(fileNamePath);
        std::fstream str;
        str.open (fileNamePath, std::ios_base::in);
        if (str.is_open()){
            isFileValid = true;
            std::string line;
            while (getline(str, line))
                totalCalibs++;

            yDebug() << "Got" << totalCalibs << "Number of Calibration \nCalibrating ...\n\n ";

            calibData = new (std::nothrow) calibrationData[totalCalibs];
            int index= 0;
            str.clear();
            str.seekg(0);

            while( getline( str, line ) ) {
                yarp::os::Bottle b( line );
                calibData[index].topLeft = cv::Point(b.get(0).asList()->get(0).asInt(), b.get(0).asList()->get(1).asInt());
                calibData[index].topRight = cv::Point(b.get(0).asList()->get(2).asInt(), b.get(0).asList()->get(3).asInt());
                calibData[index].bottomLeft = cv::Point(b.get(1).asList()->get(0).asInt(), b.get(1).asList()->get(1).asInt());
                calibData[index].bottomRight = cv::Point(b.get(1).asList()->get(2).asInt(), b.get(1).asList()->get(3).asInt());
                calibData[index].imageName = b.get(2).asList()->get(0).asString();

                index ++;
            }
            str.close();
        }
        else
        {
            isFileValid = false;
            yError() << "File is invalid, please check...";
        }

        if (isFileValid)
        {
            for (size_t x = 0; x < totalCalibs; x++)
            {
                std::string path = filePath + calibData[x].imageName;

                //load original image
                cv::Mat tmp_img = cv::imread(path, cv::IMREAD_GRAYSCALE);
                cv::flip(tmp_img, calibData[x].image, 1); 

                calibData[x].topLeft.x = calibData[x].image.cols - calibData[x].topLeft.x;
                calibData[x].topRight.x = calibData[x].image.cols - calibData[x].topRight.x;
                calibData[x].bottomLeft.x = calibData[x].image.cols - calibData[x].bottomLeft.x;
                calibData[x].bottomRight.x = calibData[x].image.cols - calibData[x].bottomRight.x;
                
                if( calibData[x].image.empty())
                {
                   yDebug() << "Could not open or find the images!";
                   return -1;
                }

                //create the mask and the resuting image
                calibData[x].imageMask = cv::Mat::zeros(cv::Size(calibData[x].image.size()), CV_8UC1);
                calibData[x].resultImage = cv::Mat::zeros(cv::Size(calibData[x].image.size()), calibData[x].image.type());

                //get coordinates
                std::vector< std::vector<cv::Point> >  coordinates;
                coordinates.push_back(std::vector<cv::Point>());
                coordinates[0].push_back(calibData[x].topLeft);
                coordinates[0].push_back(calibData[x].topRight);
                coordinates[0].push_back(calibData[x].bottomRight);
                coordinates[0].push_back(calibData[x].bottomLeft);

                //draw mask
                drawContours( calibData[x].imageMask, coordinates,0, cv::Scalar(255), cv::FILLED, 8 );

                //get the resulting roi onto black image
                calibData[x].image.copyTo(calibData[x].resultImage, calibData[x].imageMask);
            }
        }

//        testImg=cv::imread("/home/vvasco/new-evt-stream.png", cv::IMREAD_GRAYSCALE);
//        int dilate_niter = 1;
//        int erode_niter = 1;

//        int morph_size = 1; // kernel size praticamente
//        int open_size = 1;

//         cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5 ), cv::Point( -1, -1 ) );
//         cv::morphologyEx( testImg, testImg, cv::MORPH_CLOSE, element );

//         cv::Mat element_open = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ), cv::Point( -1, -1 ) );
//         cv::morphologyEx( testImg, testImg, cv::MORPH_OPEN, element_open );

        //cv::Mat(5, 5, CV_8UC1)
//        cv::dilate(testImg, testImg, cv::Mat(), cv::Point(-1,-1), dilate_niter, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
//        cv::erode(testImg, testImg, cv::Mat(), cv::Point(-1,-1), erode_niter, cv::BORDER_ISOLATED, cv::morphologyDefaultBorderValue());

        configDone = true;
        return isFileValid;
    }

    /********************************************************/
    void close()
    {
        inPortRight.close();
        outPortLeft.close();
        outPortRight.close();
        targetPort.close();
        dispOutPort.close();
        templOutPort.close();

        if (isFileValid)
            delete[] calibData;

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
    }

    /********************************************************/
    void setLabel(cv::Mat& im, const std::string label, const cv::Point& pt, cv::Scalar& scalar)
    {
        int fontface = cv::FONT_HERSHEY_DUPLEX;

        cv::rectangle(im, cv::Point(0, 0),  cv::Point(60, 50), CV_RGB(0,0,0), -1);

        std::string newLabel = label + "%";

        int labelpos = 0;
        int labelPercent = std::stoi (newLabel);
        if (labelPercent > 9 )
            labelpos = 5;

        std::string calibLabel = std::to_string(indexCalib) + "/" + std::to_string(totalCalibs);

        cv::putText(im, newLabel, cv::Point(pt.x-labelpos, pt.y), fontface, 0.6, scalar, 2);

        int xpos = 0;
        if (indexCalib > 9 && totalCalibs > 9 )
            xpos = 10;
        else if (indexCalib > 9 || totalCalibs > 9 )
            xpos = 5;

        cv::putText(im, calibLabel, cv::Point(pt.x-xpos, pt.y+20), fontface, 0.5, cv::Scalar(200, 200, 200), 2);
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &inImage )
    {
        if (isFileValid && configDone)
        {
            const std::lock_guard<std::mutex> lock(mtx);
            
            yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImageLeft  = outPortLeft.prepare();
            yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImageRight  = outPortRight.prepare();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> &dispImage  = dispOutPort.prepare();
            yarp::sig::ImageOf<yarp::sig::PixelMono> &templImage  = templOutPort.prepare();

            yarp::os::Bottle &outTargets = targetPort.prepare();

            outImageLeft.resize(inImage.width(), inImage.height());
            outImageRight.resize(inImage.width(), inImage.height());

            outImageLeft.zero();
            outImageRight.zero();

            yarp::sig::ImageOf<yarp::sig::PixelRgb> *rightImage = inPortRight.read();

            imgMatRight =  yarp::cv::toCvMat(*rightImage);

            imgMatLeft = yarp::cv::toCvMat(inImage);

            cv::flip(imgMatLeft, imgMat_flipped, 1);

            if (!completedCalibration)
            {
                cv::Mat mask(imgMat_flipped.rows, imgMat_flipped.cols, CV_8UC1, cv::Scalar(0));

                //get coordinates
                std::vector< std::vector<cv::Point> >  coordinates;
                coordinates.push_back(std::vector<cv::Point>());
                coordinates[0].push_back(calibData[indexCalib].topLeft);
                coordinates[0].push_back(calibData[indexCalib].topRight);
                coordinates[0].push_back(calibData[indexCalib].bottomRight);
                coordinates[0].push_back(calibData[indexCalib].bottomLeft);

                //draw mask
                drawContours( mask, coordinates, 0, cv::Scalar(255), cv::FILLED, 8 );

                //filter
                cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5 ), cv::Point( -1, -1 ) );
                cv::morphologyEx( result, imgMat_flipped, cv::MORPH_CLOSE, element );

                //get the resulting roi onto black image
                proc = imgMat_flipped.clone();
                proc.copyTo(result, mask);

//                std::string calibname = "calib" + std::to_string(indexCalib) + ".png";
//                imwrite(calibname, calibData[indexCalib].resultImage);

//                std::string name = "stream" + std::to_string(indexCalib) + ".png";
//                imwrite(name, result);

                cvtColor(result, result, cv::COLOR_RGB2GRAY);

                cv::Size patternsize(8,6); //interior number of corners

                bool patternfound = findChessboardCorners(result, patternsize, corners,
                                     cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
                                     | cv::CALIB_CB_FAST_CHECK);

                cv::threshold(result, result, 100, 255, cv::THRESH_BINARY);
                cv::threshold(calibData[indexCalib].resultImage,calibData[indexCalib].resultImage, 100, 255, cv::THRESH_BINARY);

                if(patternfound)
                {
                    yDebug()<<"Pattern found!";

                    //cornerSubPix(result, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    //cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

                    drawChessboardCorners(imgMat_flipped, patternsize, cv::Mat(corners), patternfound);

                    //compare images
                    cv::compare(result, calibData[indexCalib].resultImage, finalImage, cv::CMP_EQ);
                    bitwise_not(finalImage,finalImage);

                    int nonZeroCompare = countNonZero(finalImage);

                    percentage = ( (double)(finalImage.rows * finalImage.cols) - nonZeroCompare) / (finalImage.rows * finalImage.cols);

                    yDebug() << "Percentage" << percentage << "nonZeroCompare" << nonZeroCompare;

                    if(percentage >= (percentageThresh / 100) ) {

                        colour = { 1, 218, 30 };

                        gotGoodMatch = true;

                        //std::string comparename = "compare" + std::to_string(indexCalib) + ".png";
                        //imwrite(comparename, finalImage);

                        //std::string calibname = "calib" + std::to_string(indexCalib) + ".png";
                        //imwrite(calibname, calibData[indexCalib].resultImage);

                        //std::string name = "stream" + std::to_string(indexCalib) + ".png";
                        //imwrite(name, result);

                        outImageLeft.resize(imgMat_flipped.size().width, imgMat_flipped.size().height);
                        outImageLeft = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(imgMatLeft);
                        outPortLeft.write();

                        outImageRight.resize(imgMat_flipped.size().width, imgMat_flipped.size().height);
                        outImageRight = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(imgMatRight);
                        outPortRight.write();
                    }
                }
            }

            if (isFileValid && !completedCalibration)
            {
                line(imgMat_flipped, calibData[indexCalib].topLeft, calibData[indexCalib].topRight, colour, 3);
                line(imgMat_flipped, calibData[indexCalib].topRight, calibData[indexCalib].bottomRight, colour, 3);
                line(imgMat_flipped, calibData[indexCalib].bottomRight, calibData[indexCalib].bottomLeft, colour, 3);
                line(imgMat_flipped, calibData[indexCalib].bottomLeft, calibData[indexCalib].topLeft, colour, 3);

                if (displayOverlay)
                {
                    calibData[indexCalib].resultImage = calibData[indexCalib].resultImage - cv::Scalar(0, 0, 0, 127);
                    overlayImage(&imgMat_flipped, &calibData[indexCalib].resultImage, cv::Point());
                }

                if (percentage >= 0 && percentage < 1 )
                {
                    percentage = percentage * 100;
                }
                else
                    percentage = 0;

                int percent = (int)percentage;

                setLabel(imgMat_flipped, std::to_string(percent), cvPoint(15,20), colour);
            }

            outTargets.clear();

            if (outTargets.size() >0 )
                targetPort.write();

            dispImage.resize(imgMat_flipped.size().width, imgMat_flipped.size().height);
            dispImage = yarp::cv::fromCvMat<yarp::sig::PixelBgr>(imgMat_flipped);
            dispOutPort.write();

            if (sendIndex < indexCalib && !completedCalibration)
            {
                templImage.resize(calibData[indexCalib].resultImage.size().width, calibData[indexCalib].resultImage.size().height);
                templImage = yarp::cv::fromCvMat<yarp::sig::PixelMono>(calibData[indexCalib].resultImage);
                templOutPort.write();
                sendIndex = indexCalib;
            }

            if(gotGoodMatch)
            {
                gotGoodMatch = false;
                colour = {0,0,255};
                std::cout << std::endl;

                yarp::os::Bottle cmd, reply;
                cmd.addString("pause");
                rpcClient.write(cmd,reply);

                yarp::os::Time::delay(2.0);

                cmd.clear();
                reply.clear();
                cmd.addString("play");
                rpcClient.write(cmd,reply);
                 
                if (indexCalib <= totalCalibs-1)
                    indexCalib ++;

                if (indexCalib==totalCalibs)
                {
                    std::cout << "Finished all calibrations" << std::endl;
                    completedCalibration = true;
                }
            }
            
        }
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public calibSupervisor_IDL
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
        std::string moduleName = rf.check("name", yarp::os::Value("calibSupervisor"), "module name (string)").asString();
        std::string fileName = rf.check("file", yarp::os::Value("calibrations.ini"), "file name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        std::string fileNamePath = rf.findFile(fileName);
        std::string filePath = fileNamePath;

        //get only the path
        if (filePath.find(fileName) != std::string::npos)
        {
            size_t p = -1;
            while ((p = filePath.find(fileName)) != std::string::npos) filePath.replace(p, fileName.length(), "");
        }

        yInfo() << "Found file " << fileNamePath;
        yInfo() << "And its filePath  " << filePath;

        processing = new Processing( moduleName, fileNamePath, filePath );

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

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
    
    /**********************************************************/
    bool displayOverlay(const std::string& value)
    {
        bool returnVal = processing->changeDisplay(value);
        return returnVal;
    }
    
    /**********************************************************/
    bool restart()
    {
        bool returnVal = processing->reset();
        return returnVal;
    }

    /**********************************************************/
    bool setPercentage(const double value)
    {
        bool returnVal = processing->setPercentage(value);
        return returnVal;
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
    rf.setDefaultContext("calibSupervisor");
    rf.setVerbose();
    rf.setDefaultContext(rf.getContext());
    rf.configure(argc,argv);

    return module.runModule(rf);
}
//empty line to make gcc happy

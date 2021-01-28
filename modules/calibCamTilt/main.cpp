/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco  Laura Cavaliere Vadim Tikhanoff 
 * email:  valentina.vasco@iit.it laura.cavaliere@iit.it vadim.tikhanoff@iit.it 
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

#include <vector>
#include <iostream>
#include <deque>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/SoundFile.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <regex>
#include <yarp/cv/Cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>

#include "calibCamTilt_IDL.h"

//Object Detection Using SURF detector
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
//-----------------------------------

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;



/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    std::string moduleName;  
    
    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > left_outPort, right_outPort, right_outPort_after;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  right_inPort;
    cv::Mat out_left_scene, out_right_scene, obj_template, H;
    cv::Point left_c, right_c;
    bool select_ROI = false;
    bool points_found = false;
    int shift_left, shift_right;
    std::mutex mtx;
    int top, bottom, left, right;
    cv::Point arrowDown_pt1_last{0,0}, arrowDown_pt2_last{0,0}, arrowUp_pt1_last{0,0}, arrowUp_pt2_last{0,0};
    int centroid_diff_last{-1};
  

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
    bool open()
    {
        this->useCallback();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open( "/" + moduleName + "/leftImage:i" );
        right_inPort.open( "/" + moduleName + "/rightImage:i" );
        left_outPort.open("/" + moduleName + "/leftImage:o");
        right_outPort.open("/" + moduleName + "/rightImage:o");
        right_outPort_after.open("/" + moduleName + "/rightImageAfter:o");
        shift_left = 0;
        shift_right = 0;
        return true;
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
        left_outPort.close();
        right_outPort.close();
        right_inPort.close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
        left_outPort.interrupt();
        right_outPort.interrupt();
        right_inPort.interrupt();
    }
    
    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &img )
    {   
     
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* right_img = right_inPort.read();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &left_outImage  = left_outPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &right_outImage  = right_outPort.prepare();

        cv::Mat left_scene = yarp::cv::toCvMat(img);  
        cv::Mat right_scene = yarp::cv::toCvMat(*right_img);
       
        std::lock_guard<std::mutex> lg(mtx);

        // For now the right image is the left image shifted (only to test the module)
        cv::Mat sft_right_scene = cv::Mat::zeros(right_scene.size(), right_scene.type());
        right_scene(cv::Rect(0,shift_right, right_scene.cols,right_scene.rows-shift_right)).copyTo(sft_right_scene(cv::Rect(0,0,right_scene.cols,right_scene.rows-shift_right)));
    
        cv::Mat sft_left_scene = cv::Mat::zeros(left_scene.size(), left_scene.type());
        left_scene(cv::Rect(0,shift_left, left_scene.cols,left_scene.rows-shift_left)).copyTo(sft_left_scene(cv::Rect(0,0,left_scene.cols,left_scene.rows-shift_left)));


        if (!select_ROI) {
            namedWindow("Select ROI", 2);
            //resizeWindow("Select ROI",320,240);
            Rect2d templ = selectROI("Select ROI", left_scene);  
            left_scene(templ).copyTo(obj_template); // Copying otherwise the template might change
            if (obj_template.empty()) {
                yWarning() << "The template selected was empty";
                return;
            }
            imshow("Crop image",obj_template);
            waitKey(0);
            select_ROI = true;
        }
         
        out_left_scene = CameraPanCalibration(obj_template, sft_left_scene, &left_c );
        if (select_ROI)
        {
            out_right_scene = CameraPanCalibration(obj_template, sft_right_scene, &right_c );
        
            //double centroid_diff = cv::norm(cv::Mat(left_c),cv::Mat(right_c));
            int centroid_diff = abs(left_c.y-right_c.y);
            yInfo() << "Difference between centroids: " << centroid_diff;
            
            int left_width=out_left_scene.size().width;
            int left_height=out_left_scene.size().height;
            int right_width=out_right_scene.size().width;
            int right_height=out_right_scene.size().height; 

            cv::Point arrowDown_pt1, arrowDown_pt2;
            arrowDown_pt1.x = out_left_scene.cols/8;
            arrowDown_pt1.y = 4.5*(out_left_scene.rows/6);
            arrowDown_pt2.x = out_left_scene.cols/8;
            arrowDown_pt2.y = 5.5*(out_left_scene.rows/6);

            cv::Point arrowUp_pt1, arrowUp_pt2;
            arrowUp_pt1.x = out_left_scene.cols/8;
            arrowUp_pt1.y = 5.5*(out_left_scene.rows/6);
            arrowUp_pt2.x = out_left_scene.cols/8;
            arrowUp_pt2.y = 4.5*(out_left_scene.rows/6);

            if (points_found) {
                cv::Scalar color;
                if (centroid_diff< 3){
                    color = cv::Scalar(23,187,76);
                }
                else{
                    color = cv::Scalar(10,15,184);
                }

                // Grid on the left image
                for(int i = 0; i < left_height; i += (left_height/3))
                    cv::line(out_left_scene,Point(0,i),Point(left_width,i),color, 2);
                cv::line(out_left_scene,Point(0,left_height-1),Point(left_width,left_height-1),color, 2);

                for(int i = 0; i < left_width; i += (left_width/3))
                    cv::line(out_left_scene,Point(i,0),Point(i,left_height),color, 2);

                // Grid on the right image
                for(int i = 0; i < right_height; i += (right_height/3))
                    cv::line(out_right_scene,Point(0,i),Point(right_width,i),color, 2);
                cv::line(out_right_scene,Point(0,right_height-1),Point(right_width,right_height-1),color, 2);

                for(int i = 0; i < right_width; i += (right_width/3))
                    cv::line(out_right_scene,Point(i,0),Point(i,right_height),color, 2);

                if(left_c.y < right_c.y){
                    cv::arrowedLine(out_left_scene, arrowUp_pt1, arrowUp_pt2, color, 4, 8, 0, 0.3); // draw arrow!
                    cv::arrowedLine(out_right_scene, arrowDown_pt1, arrowDown_pt2, color, 4, 8, 0, 0.3);
                }
                else{
                    cv::arrowedLine(out_right_scene, arrowUp_pt1, arrowUp_pt2, color, 4, 8, 0, 0.3); // draw arrow!
                    cv::arrowedLine(out_left_scene, arrowDown_pt1, arrowDown_pt2, color, 4, 8, 0, 0.3);
                }

                cv::circle(out_right_scene, right_c, 5, color, -1, 8);
                cv::circle(out_left_scene, left_c, 5, color, -1, 8);

                std::string str = std::to_string (centroid_diff);
//                str.erase ( str.find_last_not_of('0') + 1, std::string::npos );
//                str.erase(std::remove(str.begin(), str.end(), '.'), str.end());
                cv::putText(out_left_scene, str, Point(out_left_scene.cols/6, 5.25*(out_left_scene.rows/6)), FONT_HERSHEY_SIMPLEX , 1, color, 2, 8, false) ;
                cv::putText(out_right_scene, str, Point(out_right_scene.cols/6, 5.25*(out_right_scene.rows/6)), FONT_HERSHEY_SIMPLEX , 1,  color, 2, 8, false) ;

                arrowUp_pt1_last = arrowUp_pt1;
                arrowUp_pt2_last = arrowUp_pt2;
                arrowDown_pt1_last = arrowDown_pt1;
                arrowDown_pt2_last = arrowDown_pt2;
                centroid_diff_last = centroid_diff;
            }
            else {
                if (centroid_diff_last < 0) {
                    yInfo() << "No correspondences found for the first image";
                }
                else {
                    cv::Scalar color;
                    if (centroid_diff_last < 3){
                        color = cv::Scalar(23,187,76);
                    }
                    else{
                        color = cv::Scalar(10,15,184);
                    }

                    // Grid on the left image
                    for(int i = 0; i < left_height; i += (left_height/3))
                        cv::line(out_left_scene,Point(0,i),Point(left_width,i),color, 2);
                    cv::line(out_left_scene,Point(0,left_height-1),Point(left_width,left_height-1),color, 2);

                    for(int i = 0; i < left_width; i += (left_width/3))
                        cv::line(out_left_scene,Point(i,0),Point(i,left_height),color, 2);

                    // Grid on the right image
                    for(int i = 0; i < right_height; i += (right_height/3))
                        cv::line(out_right_scene,Point(0,i),Point(right_width,i),color, 2);
                    cv::line(out_right_scene,Point(0,right_height-1),Point(right_width,right_height-1),color, 2);

                    for(int i = 0; i < right_width; i += (right_width/3))
                        cv::line(out_right_scene,Point(i,0),Point(i,right_height),color, 2);

                    if(left_c.y < right_c.y){
                        cv::arrowedLine(out_left_scene, arrowUp_pt1_last, arrowUp_pt2_last, color, 4, 8, 0, 0.3); // draw arrow!
                        cv::arrowedLine(out_right_scene, arrowDown_pt1_last, arrowDown_pt2_last, color, 4, 8, 0, 0.3);
                    }
                    else{
                        cv::arrowedLine(out_right_scene, arrowUp_pt1_last, arrowUp_pt2_last, color, 4, 8, 0, 0.3); // draw arrow!
                        cv::arrowedLine(out_left_scene, arrowDown_pt1_last, arrowDown_pt2_last, color, 4, 8, 0, 0.3);
                    }

                    cv::circle(out_right_scene, right_c, 5, color, -1, 8);
                    cv::circle(out_left_scene, left_c, 5, color, -1, 8);

                    std::string str = std::to_string (centroid_diff_last);
//                    str.erase ( str.find_last_not_of('0') + 1, std::string::npos );
//                    str.erase(std::remove(str.begin(), str.end(), '.'), str.end());
                    cv::putText(out_left_scene, str, Point(out_left_scene.cols/6, 5.25*(out_left_scene.rows/6)), FONT_HERSHEY_SIMPLEX , 1, color, 2, 8, false) ;
                    cv::putText(out_right_scene, str, Point(out_right_scene.cols/6, 5.25*(out_right_scene.rows/6)), FONT_HERSHEY_SIMPLEX , 1,  color, 2, 8, false) ;
                }
            }

            left_outImage.resize( out_left_scene.rows, out_left_scene.cols);
            left_outImage=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(out_left_scene);
            left_outPort.write(); 

            right_outImage.resize( out_right_scene.rows, out_right_scene.cols);
            right_outImage=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(out_right_scene);
            right_outPort.write(); 
        }
       
     
    }

    /********************************************************/
    cv::Mat CameraPanCalibration(const cv::Mat &image_obj, const cv::Mat &image_scene, cv::Point* center )
    {  
  
        //Check whether images have been loaded
        if( !image_obj.data)
        { 
            cout<< " --(!) Error reading image1 " << endl;
        }
        if( !image_scene.data)
        { 
            cout<< " --(!) Error reading image2 " << endl;
        }
 
        //-- Step 1: Detect the keypoints using SURF Detector and Calculate descriptors 
        int minHessian = 400;
        //Ptr<SURF> detector = SURF::create( minHessian );
        Ptr<SIFT> detector = SIFT::create(minHessian);
        vector<KeyPoint> keypoints_obj,keypoints_scene;
        Mat descriptors_obj, descriptors_scene;
        detector->detectAndCompute(image_obj, noArray(), keypoints_obj, descriptors_obj);
        detector->detectAndCompute(image_scene, noArray(), keypoints_scene, descriptors_scene);
    
        //-- Step 2: Matching descriptor vectors  
        //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( DescriptorMatcher::BRUTEFORCE );
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( DescriptorMatcher::MatcherType::FLANNBASED );
//        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( DescriptorMatcher::FLANNBASED );

        std::vector< std::vector<DMatch> > all_matches;
        std::vector< DMatch > good_matches;
        matcher->knnMatch( descriptors_obj, descriptors_scene, all_matches, 2 );
        //        matcher->match( descriptors_obj, descriptors_scene, good_matches );


        //-- Step 3: Localize the object
        vector<Point2f> obj;
        vector<Point2f> scene;

        const float ratio_thresh = 0.7f;
        for( int i = 0; i < all_matches.size(); i++ )
        {
            //-- Step 4: Get the keypoints from the  matches
            //-- Filter matches using the Lowe's ratio test
            if (all_matches[i][0].distance < ratio_thresh * all_matches[i][1].distance)
            {
                good_matches.push_back( all_matches[i][0] );
                obj.push_back( keypoints_obj [all_matches[i][0].queryIdx ].pt );
                scene.push_back( keypoints_scene[ all_matches[i][0].trainIdx ].pt );
            }
        }

        yInfo() << "Number of good matches: " << good_matches.size();

//        Mat img_matches;
//        drawMatches( image_obj, keypoints_obj, image_scene, keypoints_scene,
//                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//        imshow("Original", img_matches);
//        waitKey(0);

        if (obj.size() > 4) {
            //-- Step 5:FindHomography
            H = findHomography( obj, scene, RANSAC );
            points_found = true;
        }
        else
        {   
            yDebug() << "Not enough corresponding points";
            points_found = false;
        }      

        //-- Step 6: Get the corners of the object which needs to be detected.
        vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( image_obj.cols, 0 );
        obj_corners[2] = cvPoint( image_obj.cols, image_obj.rows ); 
        obj_corners[3] = cvPoint( 0, image_obj.rows );

        //-- Step 7: Get the corners of the object form the scene(background image)
        std::vector<Point2f> scene_corners(4);

         
        //-- Step 8:Get the perspectiveTransform
        if ( !H.empty()) {
            perspectiveTransform( obj_corners, scene_corners, H);

            //Bounding Box Centroid
            *center = Point(scene_corners[3].x + (scene_corners[2].x - scene_corners[3].x )/2, scene_corners[0].y + (scene_corners[3].y - scene_corners[0].y)/2);
//            cv::circle(image_scene, *center, 5, cv::Scalar(0,255,0), -1, 8);
            
        }
        
        
        return image_scene;
   }

    /********************************************************/
    bool setOffset(const string& cam, const int32_t n_pixel)
    {   
        std::lock_guard<std::mutex> lg(mtx);
        if (cam=="left"){
            shift_left = n_pixel;
        }
        if (cam=="right"){
            shift_right = n_pixel;
        }

        return true;
    }
     /********************************************************/
     bool reset()
     {  
         std::lock_guard<std::mutex> lg(mtx);
         select_ROI = false;
         points_found = false;
         centroid_diff_last = -1;
         return true;
     }

    /********************************************************/
    bool stop_acquisition()
    {
        return true;
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public calibCamTilt_IDL
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
        std::string moduleName = rf.check("name", yarp::os::Value("CameraPanCalibration"), "module name (string)").asString();
        
        yDebug() << "Module name" << moduleName;
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        processing = new Processing( moduleName );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->close();
        //processing->stop();

        if (processing) delete processing;
        std::cout << "The module is closed." << std::endl;
        rpcPort.close();
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool quit()
    {
        closing=true;
        return true;
    }

    /********************************************************/
    bool setOffset(const string& cam, const int32_t n_pixel)
    {   
        yDebug() << cam.c_str() ;
        processing->setOffset(cam, n_pixel);
        return true;
    }
     /********************************************************/
    bool reset()
    {
         processing->reset();
        return true;
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

    rf.setVerbose( true );
    rf.setDefaultContext( "calibCamTilt" );
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefault("name","calibCamTilt");
    rf.configure(argc,argv);

    return module.runModule(rf);
}

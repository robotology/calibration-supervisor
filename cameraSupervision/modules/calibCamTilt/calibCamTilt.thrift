# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
#
# calibCamTilt.thrift

/**
* calibCamTilt_IDL
*
* IDL Interface calibCamTilt.
*/

struct Bottle {
} (
  yarp.name = "yarp::os::Bottle"
  yarp.includefile = "yarp/os/Bottle.h"
)

service calibCamTilt_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();


    /**
     * Set the offset in order to align the image.
     * @param cam: left or right; pixel: number of shifting pixels
     * @return true/false on success/failure
     */
    bool setOffset(1:string cam, 2:i32 pixel);


    bool reset();
    
}

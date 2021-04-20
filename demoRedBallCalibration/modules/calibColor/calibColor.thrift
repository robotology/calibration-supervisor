# Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# calibColor.thrift

/**
* calibColor_IDL
*
* IDL Interface to \ref Calib Supervisor Module.
*/
service calibColor_IDL
{
    /**
    * Set the value of the percentage threshold
    @param value specifies its value
    * @return true/false on success/failure
    */
    bool setPercentage(1:double value)
    
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}

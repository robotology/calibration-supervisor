# Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# calib-supervisor.thrift

/**
* calib-supervisor_IDL
*
* IDL Interface to \ref Calib Supervisor Module.
*/
service calibSupervisor_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
    
    /**
    * Select to draw the chessboard overlay of not (defaults overlay off)
    @param value specifies its value (on | off)
    * @return true/false on success/failure
    */
    bool displayOverlay(1:string on);
    
    /**
    * restart the supervised calibration procedure
    * @return true/false on success/failure
    */
    bool restart();
}

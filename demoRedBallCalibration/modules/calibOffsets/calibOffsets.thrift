# Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# calibOffsets.thrift

/**
* calibOffsets_IDL
*
* IDL Interface to \ref Calib Offsets Module.
*/
service calibOffsets_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
    */
    bool quit();

    /**
     * Look at arm and calibrate.
     * @param part to calibrate.
     * @param timeout in seconds (default 30 s).
     * @return true/false on success/failure
    */
    bool lookAndCalibrate(1:string part, 2:i32 timeout=30);

    /**
     * Home arms and gaze.
     * @return true/false on success/failure
    */
    bool home();

    /**
     * Get reaching offset.
     * @param part to get the offset.
     * @return reaching offset (x, y, z) with respect to robot root.
    */
    list<double> getOffset(1:string part);

    /**
     * Reset calibration offsets and clear output file.
     * @return true/false on success/failure.
    */
    bool reset();

    /**
     * True if file has been written.
     * @param part to be saved in the output file (left / right / both).
     * @return true/false on success/failure.
    */
    bool writeToFile(1:string part);


}

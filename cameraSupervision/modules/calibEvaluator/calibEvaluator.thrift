# Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# calib-evaluator.thrift

/**
* calib-evaluator_IDL
*
* IDL Interface to \ref Calib Evaluator Module.
*/
service calibEvaluator_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();

    /**
     * Get the calibration score.
     * @return calibration score
     */
    double getScore();

    /**
     * Set the matching threshold.
     * @return true/false on success/failure
    */
    bool setThreshold(1:double threshold)

    /**
     * Reset match.
     * @return true/false on success/failure
     */
    bool reset();
}

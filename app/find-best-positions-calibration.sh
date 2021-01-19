#!/bin/bash

# to have dot separated values instead of comma separated values
#LANG=en_US

# launch the demo
run() {
    CANDIDATES=$1
    echo "Generating $CANDIDATES candidate positions"
    echo "Running yarp"
    yarpserver --write --silent &

    echo "Running world"
    gazebo camera-calibration-chessboard.sdf &

    echo "Waiting for ports"
    yarp wait /icubSim/head/state:o
    yarp wait /icubSim/inertial

    echo "Running gaze"
    iKinGazeCtrl --context camera-calibration-best-pos --from iKinGazeCtrl.ini --torso off &

    echo "Waiting for chessboard"
    yarp wait /chessboard/mover:i
    yarp wait /iKinGazeCtrl/x:o
    yarp wait /iKinGazeCtrl/angles:o
    yarp wait /iKinGazeCtrl/q:o
    yarp wait /iKinGazeCtrl/events:o

    echo "Running stereoCalib"
    stereoCalib --robotName icubSim --context camera-calibration-best-pos --from icubSimEyes.ini &
    movePattern --random false &

    echo "Connect all"
    yarpmanager-console --application ${ICUBcontrib_DIR}/share/ICUBcontrib/applications/camera-calibration/camera-calibration-gazebo-candidate-pos-app.xml --run --connect --exit --silent

    for i in $( eval echo {1..$CANDIDATES} )
    do
      echo "Running calibration for set $i"
      echo "start" | yarp rpc /movePattern/rpc
      while true; do
        check=$(echo "isRunning" | yarp rpc /movePattern/rpc | awk '{print $2}')
        if [ $check -eq "0" ]; then
          echo "Calibration done!"
          sleep 5
          break
        fi
      done
    done

    sleep 1
    declare -a modules=("movePattern" "stereoCalib" "yarpview")
    for module in ${modules[@]}; do
        killall ${module}
    done

    sleep 5
    declare -a modules=("iKinGazeCtrl")
    for module in ${modules[@]}; do
        killall ${module}
    done

    sleep 5
    declare -a modules=("gzclient" "gzserver" "yarpserver")
    for module in ${modules[@]}; do
        killall ${module}
    done  
}

# clean up hanging up resources
clean() {
    declare -a modules=("movePattern" "yarpview" "stereoCalib" \
                        "iKinGazeCtrl" "gzclient" "gzserver" "yarpserver")
    for module in ${modules[@]}; do
        killall -9 ${module}
    done
}

################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
echo "********************************************************************************"
echo ""

$1 $2

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    exit 1
fi

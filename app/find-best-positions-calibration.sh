#!/bin/bash

# to have dot separated values instead of comma separated values
#LANG=en_US

# find-best-positions-calibration.sh run camera-calibration-best-pos/event-cameras 200 0.1734 0.13

# launch the demo
run() {
    CONTEXT=$1
    CANDIDATES=$2
    BOARD_WIDTH=${3:-0.24}
    BOARD_HEIGHT=${4:-0.18}
    
    echo "Running for context $CONTEXT"    
    echo "Generating $CANDIDATES candidate positions"
    echo "Running yarp"
    yarpserver --write --silent &

    echo "Running world"
    gazebo $CONTEXT/camera-calibration-chessboard.sdf &

    echo "Waiting for ports"
    yarp wait /icubSim/head/state:o
    yarp wait /icubSim/inertial

    echo "Running gaze"
    iKinGazeCtrl --context $CONTEXT --from iKinGazeCtrl.ini --torso off &

    echo "Waiting for chessboard"
    yarp wait /chessboard/mover:i
    yarp wait /iKinGazeCtrl/x:o
    yarp wait /iKinGazeCtrl/angles:o
    yarp wait /iKinGazeCtrl/q:o
    yarp wait /iKinGazeCtrl/events:o

    echo "Running movePattern"
    if [[ $CONTEXT == *"event-cameras"* ]]; then
        movePattern --context $CONTEXT --random false --maxx 0.005 --maxy 0.02 --maxangle 15.0 --board_width $BOARD_WIDTH --board_height $BOARD_HEIGHT &
    else
        movePattern --context $CONTEXT --random false &
    fi    

    for i in $( eval echo {1..$CANDIDATES} )
    do
      echo "Running calibration for set $i"
      echo "Running stereoCalib"
      stereoCalib --robotName icubSim --context $CONTEXT --from icubSimEyes.ini &
      yarp wait /stereoCalib/cmd
    
      echo "Connect all"
      yarpmanager-console --application ${ICUBcontrib_DIR}/share/ICUBcontrib/applications/camera-calibration-supervisor/camera-calibration-best-pos/camera-calibration-gazebo-candidate-pos-app.xml --run --connect --exit --silent
      echo "start" | yarp rpc /movePattern/rpc
      while true; do
        check=$(echo "isRunning" | yarp rpc /movePattern/rpc | awk '{print $2}')
        if [ $check -eq "0" ]; then
          echo "Calibration done!"
          sleep 5
          break
        fi
      done
      sleep 1
      echo "Closing stereoCalib"
      killall stereoCalib
      sleep 2
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

$1 $2 $3 $4 $5

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    exit 1
fi

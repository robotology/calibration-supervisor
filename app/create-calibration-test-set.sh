#!/bin/bash

# to have dot separated values instead of comma separated values
#LANG=en_US

# launch the demo
run() {
    TEST_SET=$1
    NIMAGES=$2
    yarpserver --write --silent &

    gazebo camera-calibration-chessboard-closer.sdf &

    yarp wait /icubSim/head/state:o
    yarp wait /icubSim/inertial
    iKinGazeCtrl --context camera-calibration-best-pos --from iKinGazeCtrl.ini --torso off &
 
    yarp wait /chessboard/mover:i
    yarp wait /iKinGazeCtrl/x:o
    yarp wait /iKinGazeCtrl/angles:o
    yarp wait /iKinGazeCtrl/q:o
    yarp wait /iKinGazeCtrl/events:o

    movePattern --period 0.1 --save false --create_test true --nimages $NIMAGES --maxx 0.0 --maxy 0.0 &

    yarp wait /movePattern/gt:o
    yarp wait /movePattern/distorted:o   

    yarpmanager-console --application ${ICUBcontrib_DIR}/share/ICUBcontrib/applications/camera-calibration/camera-calibration-gazebo-test-img-app.xml --run --connect --exit --silent

    echo "Start!"
    for i in $( eval echo {1..$TEST_SET} )
    do
      echo "start" | yarp rpc /movePattern/rpc
      echo "Saving test set $i"
      while true; do
        check=$(echo "isRunning" | yarp rpc /movePattern/rpc | awk '{print $2}')
        if [ $check -eq "0" ]; then
          echo "Images saved!"
          break
        fi
      done
    done
 
    sleep 1
    declare -a modules=("movePattern" "yarpdatadumper" "yarpview")
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
    declare -a modules=("movePattern" "yarpview" "yarpdatadumper" \
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

$1 $2 $3

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    exit 1
fi

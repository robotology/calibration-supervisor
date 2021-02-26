#!/bin/bash

# to have dot separated values instead of comma separated values
#LANG=en_US

# evaluate-calibration.sh run camera-calibration-best-pos/event-cameras /home/vvasco/dev/robotology/camera-calibration-supervisor/testsets/event-cameras 200

# launch the demo
run() {
    CONTEXT=$1
    FOLDER=$2
    CANDIDATES=$3
    yarpserver --write --silent &

    yarpdataplayer &
    yarp wait /yarpdataplayer/rpc:i
    echo "load $FOLDER" | yarp rpc /yarpdataplayer/rpc:i
    
    yarp wait /movePattern/gt:o
    yarp wait /movePattern/distorted:o

    calibEvaluator &    
    for i in $( eval echo {1..$CANDIDATES} )
    do
      dstfile="/home/vvasco/.local/share/yarp/contexts/$CONTEXT/candidatePos_$i/score.txt"
      calibfile="/home/vvasco/.local/share/yarp/contexts/$CONTEXT/candidatePos_$i/outputCalib.ini"
      echo $calibfile
      if [ -s $calibfile ]; then
          camCalib --robotName icubSim --name /camCalib/left --context $CONTEXT/candidatePos_$i --from outputCalib.ini --group CAMERA_CALIBRATION_LEFT --CAMERA_CALIBRATION_LEFT::drawCenterCross 0 &
      yarpmanager-console --application ${ICUBcontrib_DIR}/share/ICUBcontrib/applications/camera-calibration-supervisor/$CONTEXT/evaluate-calibration.xml --run --connect --exit --silent
          echo "play" | yarp rpc /yarpdataplayer/rpc:i    
          echo "Start!"
          while true; do
              check=$(echo "getSliderPercentage" | yarp rpc /yarpdataplayer/rpc:i | awk '{print $2}')
              if [ $check -eq "100" ]; then
                  echo "Done!"
                  echo "Saving score..."
                  score=$(echo "getScore" | yarp rpc /calibEvaluator/rpc | awk '{print $2}')
                  echo $score
                  echo $score >| $dstfile
                  sleep 1
                  echo "Saved score!"
                  echo "reset" | yarp rpc /calibEvaluator/rpc
                  break
              fi
          done
          echo "Done loop!"   
          sleep 1
          echo "Closing camCalib"
          killall camCalib
          sleep 1
      else
        echo "outputCalib is empty"
        echo "Skipping"  
        echo 0.0 >| $dstfile
      fi
   done 
 
   sleep 1
    declare -a modules=("yarpdataplayer" "calibEvaluator" "yarpview")
    for module in ${modules[@]}; do
        killall ${module}
    done

    sleep 5
    declare -a modules=("yarpserver")
    for module in ${modules[@]}; do
        killall ${module}
    done  
}

# clean up hanging up resources
clean() {
    declare -a modules=("yarpdataplayer" "camCalib" "calibEvaluator" "yarpview" \
                        "yarpserver")
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

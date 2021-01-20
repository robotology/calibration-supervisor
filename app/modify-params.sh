#!/bin/bash

function copyParams () {
    local file=$1
    local -n icubEyesElem=$2
    local -n outputElem=$3
    
    for i in {0..2}
    do
        index=3
        echo "Running for ${icubEyesElem[$i,0]} ..."
        for (( c=${icubEyesElem[$i,1]}+1; c<=${icubEyesElem[$((i+1)),1]}-1; c++ ))
        do
            tmp="$(sed $c!d $file)"
            if [ ! -z "$tmp" ]
            then
                #echo "line $c ${outputElem[$i,$index]} --- ${tmp:0:2} wrt ${outputElem[$i,$index]:0:2} in index $index"
                if [[ ${tmp:0:2} == ${outputElem[$i,$index]:0:2} ]];
                then 
                    sed "${c}s/.*/${outputElem[$i,$index]}/" $icubEyesFile  > tmp.txt
                    mv tmp.txt $icubEyesFile
                    index=$((index+1))
                fi
            fi
        done
    done 
}

function getLine () {
    local str="$1"
    local file="$2"
    local result="$(grep -n "$str" $file | head -n 1 | cut -d: -f1)"
    echo "$result"
}

function getFileParameters () {

    local file=$1
    local -n arr=$2
    local endLine="$(wc -l $file | awk '{ print $1 }' )"
    
    arr[0,0]="CAMERA_CALIBRATION_RIGHT"
    arr[0,1]=$(getLine "${arr[0,0]}" "$file")
    arr[1,0]="CAMERA_CALIBRATION_LEFT"
    arr[1,1]=$(getLine "${arr[1,0]}" "$file")
    arr[2,0]="STEREO_DISPARITY"
    arr[2,1]=$(getLine "${arr[2,0]}" "$file")
    arr[3,0]="ENDOFFILE"
    arr[3,1]=$((endLine+1))

    for i in {0..2}
    do 
        index=3
        for (( c=${arr[$i,1]}+1; c<=${arr[$((i+1)),1]}-1; c++ ))
        do
            tmp="$(sed $c!d $file)"
            if [ ! -z "$tmp" ]
            then
                arr[$i,$index]=$tmp  
                index=$((index+1)) 
            fi
            arr[$i,2]=$index
        done
    done
}

###############################################################################
# "MAIN" FUNCTION:                                                            #
###############################################################################

icubEyesFile=$1
outputFile=$2
calibContext=$3

if [[ $# -lt 2 ]] ; then
    echo "No options were passed!"

    echo "You need to run this script with nameOfiCubEyes.ini and nameofOutputFile.ini"
    exit 1
fi

echo " "
echo "Running script...with params $icubEyesFile $outputFile $calibContext"
echo " "
declare -A outputElements
declare -A icubEyesElements

echo "Running stereoCalib"
stereoCalib --context $calibContext --from $icubEyesFile /dev/null 2>&1 & 

FILE=$outputFile
while [ ! -f $outputFile ]
do
  sleep 2 # or less like 0.2
  echo "waiting for file ..."
done
ls -l $outputFile

echo "Got the file waiting for safety..."
sleep 5

getFileParameters $outputFile outputElements
getFileParameters $icubEyesFile icubEyesElements

#echo "Got ${outputElements[0,0]} ${outputElements[0,1]} ${outputElements[0,2]} ${outputElements[0,5]} ${outputElements[0,6]}"
#echo "Got ${outputElements[1,0]} ${outputElements[1,1]} ${outputElements[1,2]} ${outputElements[1,5]} ${outputElements[1,6]}"

copyParams $icubEyesFile icubEyesElements outputElements 

camCalib --from $icubEyesFile /dev/null 2>&1

echo " "
echo "Script completed successfully..."

exit 0

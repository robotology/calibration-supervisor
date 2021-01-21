#!/bin/bash

function copyParams () {
    local file=$1
    local -n icubEyesElem=$2
    local -n outputElem=$3

    for i in $( seq 0 $sizeOfElements )
    do
        index=3
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

    if [ $sizeOfElements = 1 ]
    then
        c=$(( icubEyesElem[2,1] ))
        NumLines=$((outputElem[3,1]-outputElem[2,1]))

        echo "" >> $icubEyesFile
    
        index=2

        for i in $( seq 0 $((NumLines-1)) )
        do    
            if [ $i = 1 ]
            then
                sed "${c}s/.*/[${outputElem[2,0]}]/" $icubEyesFile  > tmp.txt
                mv tmp.txt $icubEyesFile
            else
                echo "" >> $icubEyesFile
                sed "$((c+$i))s/.*/${outputElem[2,$index]}/" $icubEyesFile  > tmp.txt
                mv tmp.txt $icubEyesFile
                index=$((index+1))
            fi
        done
    fi
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
    
    #add two end lines
    echo "" >> $file
    echo "" >> $file

    local endLine="$(wc -l $file | awk '{ print $1 }' )"
    
    arr[0,0]="CAMERA_CALIBRATION_RIGHT"
    arr[0,1]=$(getLine "${arr[0,0]}" "$file")
    arr[1,0]="CAMERA_CALIBRATION_LEFT"
    arr[1,1]=$(getLine "${arr[1,0]}" "$file")
    arr[2,0]="STEREO_DISPARITY"
    arr[2,1]=$(getLine "${arr[2,0]}" "$file")

    if [ -z "${arr[2,1]}" ]
    then
        echo "STEREO_DISPARITY not available, will create it"
        arr[2,0]="ENDOFFILE"
        arr[2,1]=$((endLine+1))
    else
        arr[3,0]="ENDOFFILE"
        arr[3,1]=$((endLine+1))
    fi

    tmp=${#arr[@]}
    sizeOfElements=$((tmp / 4))

    for i in $( seq 0 $sizeOfElements )
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
outputFile=/root/.local/share/yarp/contexts/$3/$2
calibContext=$3
robotName=$4
if [[ $robotName == "" ]]
then
    echo "Robot name not specified"
    robotName=icub
fi

if [[ $# -lt 3 ]] ; then
    echo "No options were passed!"

    echo "You need to run this script with nameOfiCubEyes.ini nameofOutputFile.ini and context"
    exit 1
fi

echo " "
echo "Running script...with params $icubEyesFile $outputFile $calibContext $robotName"
echo " "
declare -A outputElements
declare -A icubEyesElements

echo "Running stereoCalib"
stereoCalib --robotName $robotName --context $calibContext --from $icubEyesFile > /dev/null 2>&1 & 

FILE=$outputFile
while [ ! -f $outputFile ]
do
  sleep 2 # or less like 0.2
  echo "waiting for file ..."
done
#ls -l $outputFile

echo "Got the file waiting for safety..."
sleep 5

getFileParameters $outputFile outputElements
getFileParameters $icubEyesFile icubEyesElements

#echo "Got ${outputElements[0,0]} ${outputElements[0,1]} ${outputElements[0,2]} ${outputElements[0,5]} ${outputElements[0,6]}"
#echo "Got ${outputElements[1,0]} ${outputElements[1,1]} ${outputElements[1,2]} ${outputElements[1,5]} ${outputElements[1,6]}"

copyParams $icubEyesFile icubEyesElements outputElements 

#clean up files
sed -i '$ d' $outputFile
#sed -i '$ d' $icubEyesFile
#sed -i '' -e '$ d' $outputFile

camCalib --name /camCalib/right --from $icubEyesFile --group CAMERA_CALIBRATION_RIGHT > /dev/null 2>&1 &
camCalib --name /camCalib/left --from $icubEyesFile --group CAMERA_CALIBRATION_LEFT > /dev/null 2>&1

echo " "
echo "Script completed successfully..."

exit 0

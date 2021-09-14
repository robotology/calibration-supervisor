#!/bin/bash

function copyParams () {
    local file=$1
    local -n icubEyesElem=$2
    local -n outputElem=$3
    
    #looping over the three sections (CAMERA_CALIBRATION_LEFT, CAMERA_CALIBRATION_RIGHT, STEREO_DISPARITY)
    for i in $( seq 0 $numberOfSections )
    do
        group_name_icubeyes=${icubEyesElem[$i,0]}
        for (( c=${icubEyesElem[$i,1]}+1; c<=${icubEyesElem[$((i+1)),1]}-1; c++ ))
        do
            tmp="$(sed $c!d $file)"
            if [ ! -z "$tmp" ]
            then
                #value of the current parameter in the current section
                tmp_param=$(echo ${tmp}| cut -d' ' -f 1)
                
                #looking for the matching group name in outputCalib
                for k in $( seq 0 $numberOfSections )
                do
                    group_name_outputcalib=${outputElem[$k,0]}
                    if [[ $group_name_icubeyes == $group_name_outputcalib ]];
                    then
                        #echo "$i $k Copying $group_name_outputcalib to $group_name_icubeyes"
                        for (( j=3; j<=3+${outputElem[$k,2]}; j++ ))
                        do
                            out_param=$(echo ${outputElem[$k,$j]}| cut -d' ' -f 1)
                            if [[ $tmp_param == $out_param ]];
                            then
                                echo "Copying ${outputElem[$k,$j]} in section $group_name_icubeyes" 
                                sed "${c}s/.*/${outputElem[$k,$j]}/" $icubEyesFile  > tmp.txt
                                cp tmp.txt $icubEyesFile
                            fi
                        done
                    fi
                done               
            fi
        done
    done 

    # if [ $numberOfSections = 1 ]
    # then
    #     c=$(( icubEyesElem[2,1] ))
    #     NumLines=$((outputElem[3,1]-outputElem[2,1]))

    #     echo "" >> $icubEyesFile
    
    #     index=2

    #     for i in $( seq 0 $((NumLines-1)) )
    #     do    
    #         if [ $i = 1 ]
    #         then
    #             sed "${c}s/.*/[${outputElem[2,0]}]/" $icubEyesFile  > tmp.txt
    #             mv tmp.txt $icubEyesFile
    #         else
    #             echo "" >> $icubEyesFile
    #             sed "$((c+$i))s/.*/${outputElem[2,$index]}/" $icubEyesFile  > tmp.txt
    #             mv tmp.txt $icubEyesFile
    #             index=$((index+1))
    #         fi
    #     done
    # fi
}

function getLine () {
    local str="$1"
    local file="$2"
    local result="$(grep -n "$str" $file | head -n 1 | cut -d: -f1)"
    echo "$result"
}

function getFileParameters () {

    # THIS FUNCTION WILL CREATE THE FOLLOWING STRUCTURE (example for camera_calibration_left):
    # arr[0,0]=CAMERA_CALIBRATION_LEFT
    # arr[0,1]=index of the line containing CAMERA_CALIBRATION_LEFT
    # arr[0,2]=index of the last line of CAMERA_CALIBRATION_LEFT section
    # arr[0,3<index<arr[0,2]]=content of each line inside CAMERA_CALIBRATION_LEFT section

    local file=$1
    local -n arr=$2

    local endLine="$(wc -l $file | awk '{ print $1 }' )"
    
    # MANAGING THE ORDER OF THE TWO SECTIONS : CAMERA_CALIBRATION_LEFT AND CAMERA_CALIBRATION_RIGHT
    index_left=$(getLine "CAMERA_CALIBRATION_LEFT" "$file")
    index_right=$(getLine "CAMERA_CALIBRATION_RIGHT" "$file")
    if [[ $index_left < $index_right ]]
    then
      arr[0,0]="CAMERA_CALIBRATION_LEFT"
      arr[0,1]=$index_left 
      arr[1,0]="CAMERA_CALIBRATION_RIGHT"
      arr[1,1]=$index_right 
    else
      arr[1,0]="CAMERA_CALIBRATION_LEFT"
      arr[1,1]=$index_left
      arr[0,0]="CAMERA_CALIBRATION_RIGHT"
      arr[0,1]=$index_right 
    fi

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
    
    arrSize=${#arr[@]}
    #at this stage, each section in arr has 3 elements => the array size is 8
    numberOfSections=$((arrSize / 4))
    for i in $( seq 0 $numberOfSections) 
    do 
        index=3 
        for (( c=${arr[$i,1]}+1; c<=${arr[$((i+1)),1]}-1; c++ )) 
        do
            tmp="$(sed $c!d $file)"          
            if [ ! -z "$tmp" ] 
            then
                arr[$i,$index]="$tmp" 
                index=$((index+1)) 
            fi
            arr[$i,2]=$index 
        done
    done
}

function getFileParametersAddLines () {

    # THIS FUNCTION WILL CREATE THE FOLLOWING STRUCTURE (example for camera_calibration_left):
    # arr[0,0]=CAMERA_CALIBRATION_LEFT
    # arr[0,1]=index of the line containing CAMERA_CALIBRATION_LEFT
    # arr[0,2]=index of the last line of CAMERA_CALIBRATION_LEFT section
    # arr[0,3<index<arr[0,2]]=content of each line inside CAMERA_CALIBRATION_LEFT section

    local file=$1
    local -n arr=$2
    
    #add two end lines
    echo "" >> $file
    echo "" >> $file

    local endLine="$(wc -l $file | awk '{ print $1 }' )"
    
    # MANAGING THE ORDER OF THE TWO SECTIONS : CAMERA_CALIBRATION_LEFT AND CAMERA_CALIBRATION_RIGHT
    index_left=$(getLine "CAMERA_CALIBRATION_LEFT" "$file")
    index_right=$(getLine "CAMERA_CALIBRATION_RIGHT" "$file")
    if [[ $index_left < $index_right ]]
    then
      arr[0,0]="CAMERA_CALIBRATION_LEFT"
      arr[0,1]=$index_left 
      arr[1,0]="CAMERA_CALIBRATION_RIGHT"
      arr[1,1]=$index_right 
    else
      arr[1,0]="CAMERA_CALIBRATION_LEFT"
      arr[1,1]=$index_left
      arr[0,0]="CAMERA_CALIBRATION_RIGHT"
      arr[0,1]=$index_right 
    fi

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
    numberOfSections=$((tmp / 4)) 

    added_lines=0
    for i in $( seq 0 $numberOfSections) 
    do 
        proj_found=true #false
        draw_found=true #false
        index=3 
        for (( c=${arr[$i,1]}+1; c<=${arr[$((i+1)),1]}-1; c++ )) 
        do
            tmp="$(sed $c!d $file)"
            if [[ $tmp == *"projection"* ]]; then
               #echo "projection is there!"
               proj_found=true
            fi
            if [[ $tmp == *"drawCenterCross"* ]]; then
               #echo "drawCenterCross is there!"
               draw_found=true
            fi
            
            if [ ! -z "$tmp" ] 
            then
                arr[$i,$index]="$tmp" 
                index=$((index+1)) 
            fi
            arr[$i,2]=$index 
        done
        if [[ ($proj_found != true || $draw_found != true) && ($i -eq 0 || $i -eq 1) ]]
        then  
	   if [[ ($i -eq 1) ]]
	   then
	    arr[$i,1]=$((${arr[$i,1]}+$added_lines))
	   fi	   
	   index_line=$((${arr[$i,1]}+1))   
	   first_line="projection         pinhole"
	   second_line="drawCenterCross    0"
	   if [[ $proj_found != true ]]
	   then
	   	(awk 'NR=='"$(($index_line+1))"' {print}1' $file | tee tmp.txt > /dev/null) && mv tmp.txt $file
	   	(awk 'NR=='"$(($index_line+1))"' {$0='"\"$first_line\""'} { print }' "$file" | tee tmp.txt > /dev/null) && mv tmp.txt $file
	   	added_lines=$(($added_lines + 1))
	   fi
	   if [[ $draw_found != true ]]
	   then
	   	(awk 'NR=='"$(($index_line+2))"' {print}1' $file | tee tmp.txt > /dev/null) && mv tmp.txt $file
	   	(awk 'NR=='"$(($index_line+2))"' {$0='"\"$second_line\""'} { print }' "$file" | tee tmp.txt > /dev/null) && mv tmp.txt $file
	   	added_lines=$(($added_lines + 1))
	   fi
        fi        
    done
}

###############################################################################
# "MAIN" FUNCTION:                                                            #
###############################################################################

calibContext=$3
mono=$4
robotName=$5

icubEyesFile=$(yarp resource --context $calibContext --from $1 | awk -F'"' '{print $2}' )
echo "Using file $icubEyesFile"

resourcePath=$(echo "$icubEyesFile" | sed 's|\(.*\)/.*|\1|')
outputFile=$resourcePath/$2

echo "stereoCalib writes the following file: $outputFile"

if test -f "$outputFile"; then
    echo "$outputFile exists."
    echo "Removing $outputFile."
    rm $outputFile
fi

if [[ $mono == "" ]]
then
    echo "mono param not specified"
    echo "Running stereo by default"
    mono=0
else 
    echo $mono
    if $mono; then
        mono=1
    else 
        mono=0
    fi
fi

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
echo "Running script...with params $icubEyesFile $outputFile $calibContext $mono $robotName"
echo " "
declare -A outputElements
declare -A icubEyesElements

echo "Running stereoCalib with monoCalib $mono"
stereoCalib --robotName $robotName --context $calibContext --from $icubEyesFile --STEREO_CALIBRATION_CONFIGURATION::numberOfPairs 30 --STEREO_CALIBRATION_CONFIGURATION::MonoCalib $mono & 

while [ ! -f $outputFile ]
do
  sleep 2 # or less like 0.2
  echo "waiting for file ..."
done
##ls -l $outputFile

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

sleep 3

camCalib --name /camCalib/right --from $icubEyesFile --group CAMERA_CALIBRATION_RIGHT > /dev/null 2>&1 &
camCalib --name /camCalib/left --from $icubEyesFile --group CAMERA_CALIBRATION_LEFT > /dev/null 2>&1

echo " "
echo "Script completed successfully..."

exit 0

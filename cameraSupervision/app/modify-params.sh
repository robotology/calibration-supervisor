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
                tmp_param=$(echo ${tmp}| cut -d' ' -f 1)
                out_param=$(echo ${outputElem[$i,$index]}| cut -d' ' -f 1)
                if [[ $tmp_param == $out_param ]];
                #if [[ ${tmp:0:2} == ${outputElem[$i,$index]:0:2} ]];
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
    echo "STEREO_DISPARITY ${arr[2,1]}"
   

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
    added_lines=0
    for i in $( seq 0 $sizeOfElements) 
    do 
        proj_found=false
        draw_found=false
        index=3 
        for (( c=${arr[$i,1]}+1; c<=${arr[$((i+1)),1]}-1; c++ )) 
        do
            tmp="$(sed $c!d $file)"
            if [[ $tmp == *"projection"* ]]; then
               echo "projection is there!"
               proj_found=true
            fi
            if [[ $tmp == *"drawCenterCross"* ]]; then
               echo "drawCenterCross is there!"
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

icubEyesFile=$(yarp resource --context $calibContext --from $1 | awk -F'"' '{print $2}' )
echo "Using file $icubEyesFile"

#resourcePath=$(echo "$icubEyesFile" | sed 's|\(.*\)/.*|\1|')
#outputFile=$resourcePath/$2
outputFile=$(yarp resource --context $calibContext --from $2 | awk -F'"' '{print $2}' )
echo "stereoCalib writes the following file: $outputFile"

if [[ $# -lt 3 ]] ; then
    echo "No options were passed!"

    echo "You need to run this script with nameOfiCubEyes.ini nameofOutputFile.ini and context"
    echo "If you want to specify a custom iCubEyes.ini file, please specify the full path"
    exit 1
fi

echo " "
echo "Running script...with params $icubEyesFile $outputFile $calibContext"
echo " "
declare -A outputElements
declare -A icubEyesElements

echo "Creating outputElements"
getFileParameters $outputFile outputElements

echo "Creating icubEyesElements"
getFileParameters $icubEyesFile icubEyesElements

#echo "Got ${outputElements[0,0]} ${outputElements[0,1]} ${outputElements[0,2]} ${outputElements[0,3]} ${outputElements[0,4]} ${outputElements[0,5]} ${outputElements[0,6]} ${outputElements[0,7]} ${outputElements[0,8]}"
#echo "Got ${outputElements[1,0]} ${outputElements[1,1]} ${outputElements[1,2]} ${outputElements[1,5]} ${outputElements[1,6]}"


#echo "Got ${icubEyesElements[0,0]} ${icubEyesElements[0,1]} ${icubEyesElements[0,2]} ${icubEyesElements[0,3]} ${icubEyesElements[0,4]} ${icubEyesElements[0,5]} ${icubEyesElements[0,6]} ${icubEyesElements[0,7]}"

copyParams $icubEyesFile icubEyesElements outputElements 

#echo "Got ${icubEyesElements[0,0]} ${icubEyesElements[0,1]} ${icubEyesElements[0,2]} ${icubEyesElements[0,3]} ${icubEyesElements[0,4]} ${icubEyesElements[0,5]} ${icubEyesElements[0,6]} ${icubEyesElements[0,7]}"

#clean up files
#sed -i '$ d' $outputFile
#sed -i '$ d' $icubEyesFile
#sed -i '' -e '$ d' $outputFile

sleep 3

echo " "
echo "Script completed successfully..."

exit 0

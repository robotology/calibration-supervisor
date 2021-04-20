#!/bin/bash

function copyParams () {
     local file=$1
     local -n left_off_m=$2
     local -n right_off_m=$3
     local -n conf_m=$4

    # FORMATTING CONFIG.INI INTO TMP.TXT
     echo "$(<$file)" | tr -s " " ' ' | sed 's/ /\t/' > tmp.txt 
 
    # PUTTING ALL THE OFFSETS FROM TMP.TXT IN A MATRIX 
    conf_m[0,0]="[left_arm]"
    conf_m[1,0]="reach_offset"
    conf_m[1,1]="$(getLine "${conf_m[1,0]}" "tmp.txt" "1")"
    conf_m[2,0]="grasp_offset"
    conf_m[2,1]="$(getLine "${conf_m[2,0]}" "tmp.txt" "1")"
    conf_m[3,0]="[right_arm]"
    conf_m[4,0]="reach_offset"
    conf_m[4,1]="$(getLine "${conf_m[4,0]}" "tmp.txt" "2")"
    conf_m[5,0]="grasp_offset"
    conf_m[5,1]="$(getLine "${conf_m[5,0]}" "tmp.txt" "2")"
    
    # REPLACING OFFSETS IN RIGHT_ARM SECTION
    right_index=$(grep -n "\[right_arm\]" "$file" | cut -d : -f1)
    reach_right_index=$(tail -n +$right_index "$file" | grep -n "reach_offset" | sed -n 1p | cut -d : -f1)
    new_reach_right_index=$(($reach_right_index+$right_index-1))
    grasp_right_index=$(tail -n +$right_index "$file" | grep -n "grasp_offset" | sed -n 1p | cut -d : -f1)
    new_grasp_right_index=$(($grasp_right_index+$right_index-1))
    if [[ "${right_off_m[0,0]}" = "[right_arm]" ]]   
    then 
    	 new_reach_right_line="${conf_m[4,0]}\t${right_off_m[1,1]}" 
   	 (awk 'NR=='"$new_reach_right_index"' {$0='"\"$new_reach_right_line\""'} { print }' "$file" | tee tmp.txt > /dev/null) && mv tmp.txt $file #Only for record (i.e. line) number "$new_reach_right_index, replace the whole record with the string '"\"$new_reach_right_line\""'. Then for every input record, print the record;
   	 new_grasp_right_line="${conf_m[5,0]}\t${right_off_m[2,1]}"
    	(awk 'NR=='"$new_grasp_right_index"' {$0='"\"$new_grasp_right_line\""'} { print }' "$file" | tee tmp.txt > /dev/null) && mv tmp.txt $file 
    fi 
    
     # REPLACING OFFSETS IN LEFT_ARM SECTION
    left_index=$(grep -n "\[left_arm\]" "$file" | cut -d : -f1)
    reach_left_index=$(tail -n +$left_index "$file" | grep -n "reach_offset" | sed -n 1p | cut -d : -f1) 
    new_reach_left_index=$(($reach_left_index+$left_index-1))
    grasp_left_index=$(tail -n +$left_index "$file" | grep -n "grasp_offset" | sed -n 1p | cut -d : -f1) 
    new_grasp_left_index=$(($grasp_left_index+$left_index-1))
    if [[ "${left_off_m[0,0]}" = "[left_arm]" ]] 
    then 
      new_reach_left_line="${conf_m[1,0]}\t${left_off_m[1,1]}" 
      (awk 'NR=='"$new_reach_left_index"' {$0='"\"$new_reach_left_line\""'} { print }' "$file" | tee tmp.txt > /dev/null) && mv tmp.txt $file 
      new_grasp_left_line="${conf_m[2,0]}\t${left_off_m[2,1]}" 
      (awk 'NR=='"$new_grasp_left_index"' {$0='"\"$new_grasp_left_line\""'} { print }' "$file" | tee tmp.txt > /dev/null) && mv tmp.txt $file 
    fi	
}

function getLine () {
    local str="$1"
    local file="$2"
    local lineNumber="$3"
    local result="$(grep -n "$str" "$file" | sed -n "$lineNumber"p | cut -d$'\t' -f2)" 
    echo "$result" 
}

function getFileParameters () {
   
    local file=$1
    local -n left_matrix=$2
    local -n right_matrix=$3
   
    # PUTTING ALL THE OFFSETS FROM THE FILE RETURNED BY CALIBOFFSETS IN A MATRIX 
    left_index=$(grep -n "\[left_arm\]" "$file" | cut -d : -f1)
    if [[ ! -z "$left_index" ]]
    then
        left_found=true
        left_matrix[0,0]="[left_arm]"
        left_matrix[1,0]="reach_offset"  
        left_matrix[1,1]=$(tail -n +$left_index "$file" | grep -n "reach_offset" | sed -n 1p |  cut -d$'\t' -f2) 
        left_matrix[2,0]="grasp_offset"
        left_matrix[2,1]=$(tail -n +$left_index "$file" | grep -n "grasp_offset" | sed -n 1p |  cut -d$'\t' -f2) 

    fi
    right_index=$(grep -n "\[right_arm\]" "$file" | cut -d : -f1)
    if [[ ! -z "$right_index" ]]
    then
        right_matrix[0,0]="[right_arm]"
        right_matrix[1,0]="reach_offset"  
 	right_matrix[1,1]=$(tail -n +$right_index "$file" | grep -n "reach_offset" | sed -n 1p | cut -d$'\t' -f2) 
 	right_matrix[2,0]="grasp_offset"
        right_matrix[2,1]=$(tail -n +$right_index "$file" | grep -n "grasp_offset" | sed -n 1p | cut -d$'\t' -f2) 

    fi
}

###############################################################################
# "MAIN" FUNCTION:                                                            #
###############################################################################


 if [[ $# -lt 1 ]] ; then
     echo "No options were passed!"
     echo "Please insert the calibOffsetsResults.txt file resulting from calibOffsets"

     exit 1
 fi

echo " "
echo "Running script...with params $1"
echo " "

declare -A leftOffsetsMatrix 
declare -A rightOffsetsMatrix 
declare -A configIniMatrix

calibOffsets &

#calibResultFile=$(yarp resource --context calibOffsets --from $1 | awk -F'"' '{print $2}' ) 
calibResultFile=$1
while [ ! -f $calibResultFile ]
do
  sleep 2 # or less like 0.2
  echo "waiting for file ..."
done

getFileParameters $calibResultFile leftOffsetsMatrix rightOffsetsMatrix

configIniFile=$(yarp resource --context demoRedBall --from config.ini | awk -F'"' '{print $2}' ) 
echo "Using file $configIniFile"

copyParams $configIniFile leftOffsetsMatrix rightOffsetsMatrix configIniMatrix 

sleep 3

demoRedBall --from $configIniFile

echo " "
echo "Script completed successfully..."

exit 0



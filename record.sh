#!/bin/bash

# Define bag and csv filename:
VIDEO=false
filename=
input=$1
if [ -n "$input" ]
then
    filename=$1
else
    # read in args:
    if [ -a ".recordargs" ]
    then
	argv=$(cat .recordargs)
	eval set -- "$argv"
	while getopts ":xf:" opt; do
	    case "$opt" in
		x)
		    VIDEO=true
		    ;;
		f)
		    filename=${OPTARG}
		    ;;
	    esac
	done
    else
	echo "File .recordargs not found!"
	exit 1
    fi
fi
echo "Using '${filename}' as filename"
echo "Using '${VIDEO}' for video arg"

# If filename.* exists, move to backup#.*
if [ -a ${filename}.bag ]
then
    echo "Moving old bag files"
    num=`ls | grep "${filename}.*bag" | wc -l`
    mv ${filename}.bag ${filename}_backup_"$num".bag
fi
if [ -a ${filename}_kinect.avi ]
then
    echo "Moving old movie files"
    num=`ls | grep "${filename}.*avi" | wc -l`
    mv ${filename}_kinect.avi ${filename}_kinect_backup_"$num".avi
fi


# Wait for a user to press a button:
echo "Press any button to start recording data..."
read -n 1 -s
echo "Beginning recording..."
# Start recording bag file:
rosbag record --quiet -O ${filename}.bag -e "(.*)meas_config" "(.*)ref_config" \
    "(.*)filt_state" "(.*)filt_config" "(.*)serial_commands" "(.*)serviced_values" \
    "(.*)post_covariance" "(.*)object1_position" "(.*)robot_kinect_position" \
    "(.*)tf" "(.*)start_time" "(.*)optimization_data" "(.*)mass_ref_point"&
# start recording Kinect video if we should:
if ${VIDEO}
then
    rosrun receding_planar_sys video_recorder _filename:=${filename}_kinect.avi _fps:=30 image:=/camera/rgb/image_color &
fi
sleep 1
echo "Now press any button to stop recording..."
read -n 1 -s
echo "Stopping recording, now killing recording process..."
# Stop recording:
killall -2 record
killall -2 video_recorder
sleep 2
echo "Generating csv file..."
# Generate default csv files:
info=`rosbag info ${filename}.bag`
meas=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'meas_config'`
filt=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'filt_config'`
ref=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'ref_config'`
ser=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'serial_commands'`
rostopic echo -p -b ${filename}.bag $filt > ${filename}_filt.txt 
rostopic echo -p -b ${filename}.bag $meas > ${filename}_meas.txt 
rostopic echo -p -b ${filename}.bag $ref > ${filename}_ref.txt 
rostopic echo -p -b ${filename}.bag $ser > ${filename}_ser.txt 
echo "Done creating bag and csv file"

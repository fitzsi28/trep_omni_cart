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
rosbag record --quiet -O ${filename}.bag -e "(.*)cart_point" "(.*)mass_point" \
    "(.*)omni1_force_feedback" "(.*)trep_sys" "(.*)omni1_button" \
    "(.*)visualization_marker_array" "(.*)tf"&
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
cart=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'cart_point'`
ff=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'omni1_force_feedback'`
mass=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'mass_point'`
trep=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'trep_sys'`
rostopic echo -p -b ${filename}.bag $ff > ${filename}_ff.txt 
rostopic echo -p -b ${filename}.bag $cart > ${filename}_cart.txt 
rostopic echo -p -b ${filename}.bag $mass > ${filename}_mass.txt 
rostopic echo -p -b ${filename}.bag $trep > ${filename}_trep.txt 
echo "Done creating bag and csv file"

#!/bin/sh

# Run this script in the `build` directory

echo 'Object Detection HOG Training Script'

if [ $# -lt 1 ]; then  
    echo 'usage : ../scripts/train_object_detect_hog.sh <object_name>'
	exit 1
fi
OBJECT=$1

DATADIR_PATH=../data/object_hog
IMGDIR_PATH=$DATADIR_PATH/${OBJECT}_imgs
XML_PATH=$DATADIR_PATH/${OBJECT}.xml
HOG_PATH=$DATADIR_PATH/${OBJECT}.hog

TMPXML_PATH=$DATADIR_PATH/${OBJECT}_tmp.xml

yn_prompt() {
    echo -n " input y/n > "
    read ans
    case $ans in
        y)
            return 0
            ;;
        n)
            return 1
            ;;
        *)
            yn_prompt
            ;;
    esac
}

mkdir -p $DATADIR_PATH  # Create data directories


echo '== Capture camera images =='
capture_img=0
if [ -d $IMGDIR_PATH ]; then
    echo Image directory is already created. Capture more?
    yn_prompt
    capture_img=$?
fi
if [ $capture_img -eq 0 ]; then
    mkdir -p $IMGDIR_PATH
    echo Capture images and quit to go to next step.
    ./bin/release/tool_camera_capture $IMGDIR_PATH
fi
echo


echo '== Create (or merge) dataset xml =='
if [ -f $XML_PATH ]; then
    echo ' >> Merge'
    ./bin/release/tool_imglab -c $TMPXML_PATH $IMGDIR_PATH
    ./bin/release/tool_imglab --add $XML_PATH $TMPXML_PATH 
    mv merged.xml $XML_PATH
    rm $TMPXML_PATH
else
    echo ' >> Create'
    ./bin/release/tool_imglab -c $XML_PATH $IMGDIR_PATH
fi
echo


echo '== Set bounding boxes =='
echo Set object rectangles, save xml, and close window to go to next step.
./bin/release/tool_imglab $XML_PATH
echo


echo '== Start to train =='
do_train=0
if [ -f $HOG_PATH ]; then
    echo Trained hog is already created. Retrain?
    yn_prompt
    do_train=$?
fi
if [ $do_train -eq 0 ]; then
    ./bin/release/tool_train_hog $XML_PATH $HOG_PATH
fi
echo

echo Finished !

#!/bin/sh

# Run this script in the `build` directory

echo 'Object Classification SVM Training Script'

OBJECTS='apple green_apple hand'

DATADIR_PATH=../data/object_hog
XML_PATHS=''
SVM_PATH=$DATADIR_PATH/classify.svm

# Create jointed xml paths
for object in $OBJECTS; do
    TMP_XMP_PATH=$DATADIR_PATH/${object}.xml
    if [ -f $TMPXML_PATH ]; then
        XML_PATHS="$XML_PATHS;$TMP_XMP_PATH"
    else
        echo "Invalid xml path: $TMP_XMP_PATH"
        exit 1
    fi
done

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

echo '== Start to train =='
do_train=0
if [ -f $SVM_PATH ]; then
    echo Trained svm is already created. Retrain?
    yn_prompt
    do_train=$?
fi
if [ $do_train -eq 0 ]; then
    ./bin/release/tool_train_svm $XML_PATHS $SVM_PATH
fi
echo

echo Finished !

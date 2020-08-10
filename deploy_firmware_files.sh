#!/bin/bash

CURRENT_VERSION="1.1.1"
FIRMWARE_FOLDER="/Users/OKoester/Documents/Arduino/Marlin-2-0-x-Anycubic-i3-MEGA-S-Master/.pio/build/"
OUTPUT_FOLDER="/Users/OKoester/Desktop/i3_FIRMWARE"

mkdir $OUTPUT_FOLDER

cd $FIRMWARE_FOLDER
for dir in $FIRMWARE_FOLDER/*/     # list directories in the form "/tmp/dirname/"
do
    dir=${dir%*/}      # remove the trailing "/"
    echo ${dir##*/}    # print everything after the final "<--/"
    cp ${dir##*/}/firmware.hex /$OUTPUT_FOLDER/${dir##*/}_v$CURRENT_VERSION.hex
done
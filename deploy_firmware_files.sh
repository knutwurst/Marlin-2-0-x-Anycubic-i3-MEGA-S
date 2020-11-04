#!/bin/bash

BASE_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
CURRENT_USER="$(whoami)"

VERSION_FILE="$BASE_PATH/Marlin/src/inc/Version.h"
FIRMWARE_FOLDER="$BASE_PATH/.pio/build/"
OUTPUT_FOLDER="/Users/$CURRENT_USER/Desktop/i3_FIRMWARE"

CUSTOM_BUILD_VERSION=$(egrep -o "([0-9]{1,}\.)+[a-zA-Z0-9_.-]{1,}" $VERSION_FILE -m2 | tail -n1)

if [ -d "$$OUTPUT_FOLDER/v$CUSTOM_BUILD_VERSION" ]; then
    echo "$$OUTPUT_FOLDER/$CUSTOM_BUILD_VERSION already exists."
else 
    mkdir -p $OUTPUT_FOLDER/$CUSTOM_BUILD_VERSION
fi

echo "Knutwurst's Mega Firmware Version: $CUSTOM_BUILD_VERSION"

pushd $FIRMWARE_FOLDER
  for dir in $FIRMWARE_FOLDER/*/
  do
      dir=${dir%*/}
      echo ${dir##*/}
      cp ${dir##*/}/firmware.hex /$OUTPUT_FOLDER/$CUSTOM_BUILD_VERSION/${dir##*/}_v$CUSTOM_BUILD_VERSION.hex
  done
popd

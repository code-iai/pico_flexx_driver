#!/bin/bash

if [ "$1" == "" ] || [ "$1" == "-h" ] || [ "$1" == "-?" ] || [ "$1" == "--help" ]; then
  echo "Usage: `basename $0` [source] <destination>"
  echo "  source       - path to the extracted royale sdk, for example"
  echo "                 '~/Downloads/libroyale/libroyale-1.0.5.40-LINUX-64Bit/'  "
  echo "  destination  - install destination, default '/usr/local'"
  exit 0
fi

if [ -d "$1" ]; then
  SRC="$1"
else
  echo "directory '$1' does not exist."
  exit -1
fi

if [ "$2" != "" ]; then
  DEST="$2"
else
  DEST="/usr/local"
fi

if [ ! -d "$DEST" ]; then
  echo "destination '$DEST' does not exist."
  exit -1
fi

echo "source: $SRC"
echo "destination: $DEST"

UDEV="/etc/udev/rules.d/10-royale-ubuntu.rules"
UDEV_SDK="$SRC/driver/udev/10-royale-ubuntu.rules"

if [ ! -e "$UDEV_SDK" ]; then
  echo "'$UDEV_SDK' not found in royale sdk."
  exit -1
fi

if [ ! -e "${SRC}/include/royale.hpp" ] || [ ! -d "${SRC}/include/royale" ]; then
  echo "includes not found in royale sdk."
  exit -1
fi

if [ ! -e "${SRC}/bin/libroyale.so" ] || [ ! -e "${SRC}/bin/$(readlink '${SRC}/bin/libroyale.so')" ]; then
  echo "library not found in royale sdk."
  exit -1
fi

if [ ! -e "$UDEV" ]; then
  echo "copying: '$UDEV_SDK' to: '$UDEV'"
  sudo cp "$UDEV_SDK" "$UDEV"
fi

if [ -e "$DEST/include/royale.hpp" ] || [ -d "$DEST/include/royale" ]; then
  echo "removing old royale includes"
  rm -rf $DEST/include/royale*
fi

if [ -e "$DEST/lib/libroyale.so" ]; then
  echo "removing old royale lib"
  rm -f $DEST/lib/libroyale.so*
fi

echo "copying new royale includes"
cp -r ${SRC}/include/* ${DEST}/include/
echo "copying new royale lib"
cp -P ${SRC}/bin/libroyale.so* ${DEST}/lib/


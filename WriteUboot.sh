#!/bin/bash
#-------------------------------------------------------------------------
# Some definitions that might change
IMAGES_PATH=.
UBOOT_FILE=miniflash.bin

# diverses
bold=`tput bold`
normal=`tput sgr0`

#=========================================================================
function usage()
{
bn=`basename $0`

cat << EOF

usage:
$bn <SDCard device> <uboot filename>
       Default uboot filename: $UBOOT_FILE

Example: sudo $0 /dev/sde ./u-boot.bin 
         
EOF
}

#=========================================================================
echo ""

#-------------------------------------------------------------------------
# First check the if root?
userid=`id -u`
#if [ $userid -ne "0" ]; then
#	echo ""	
#	echo $bold"Seems that you're not root"$normal
#	echo ""	
#	exit 1
#fi

#-------------------------------------------------------------------------
# SDCard parameter given?
SDCARD_DEV=$1

if [ -z $SDCARD_DEV ]; then
	echo ""	
	echo $bold"ERROR: Must set the destination device"$normal
	usage
	echo ""
	exit 1
fi

#-------------------------------------------------------------------------
# uboot filename parameter given - then use it
if [ $2 ]; then
	UBOOT_FILE=$2
fi

#-------------------------------------------------------------------------
# Check if U-Boot exists
if [ ! -e $UBOOT_FILE ]; then
	echo ""	
	echo $bold"ERROR: $UBOOT_FILE not found"$normal
	echo ""
	exit 1
fi

#-------------------------------------------------------------------------
# Make sure we are not trying to write to the primary partition
if [ $SDCARD_DEV = "/dev/sda" ]; then
	echo ""
	echo $bold"ERROR: Forbidden to write to /dev/sda"$normal
	echo ""
	exit 1
fi

#-------------------------------------------------------------------------
# Check that we are writing to a block device
if [ ! -b $SDCARD_DEV ]; then
	echo ""
	echo $bold"ERROR: $SDCARD_DEV not found or no block device"$normal
	echo ""
	lsblk
	exit 1
fi

#-------------------------------------------------------------------------
# Unmount all mounted device partitions
echo ""
echo $bold"Unmount all mounted partitions..."$normal
echo ""
sudo umount ${SDCARD_DEV}* && sleep 2

#-------------------------------------------------------------------------
# Copy u-boot and kernel
echo ""
echo $bold"Copying U-Boot($UBOOT_FILE)..."$normal
echo ""
sudo dd if=$UBOOT_FILE of=${SDCARD_DEV} bs=1k seek=33 skip=0 conv=fsync || exit 1


#-------------------------------------------------------------------------
# Finally
sleep 5
echo $bold"Flushing file buffers..."$normal
sudo sync

echo $bold"OK!"$normal
exit 0


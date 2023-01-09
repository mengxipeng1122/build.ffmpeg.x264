
export ARCHS=armv7a
if [ "$NDKPATH" = "" ]; then
	export NDKPATH=/mnt/axly/hackings/android-utils/android-ndk-r15c/
fi

if [ ! -e $NDKPATH ]; then
	echo "$NDKPATH not exist, please check  Android NDK "
	exit -5;
fi

./build.sh


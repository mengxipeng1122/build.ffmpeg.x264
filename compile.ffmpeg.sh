
export ARCHS=armv7a
if [ "$ANDROID_NDK" = "" ]; then
	export ANDROID_NDK=/mnt/axly/hackings/android-utils/android-ndk-r15c/
fi

if [ ! -e $ANDROID_NDK ]; then
	echo "$ANDROID_NDK not exist, please check  Android NDK "
	exit -5;
fi

./build.sh


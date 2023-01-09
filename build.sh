#!/bin/bash
#
# Author: Renato L. F. Cunha <renatoc@gmail.com>
# This file is available according to the MIT license. Please refer to the
# LICENSE file for details.
#
# This script builds a version of FFmpeg with h.264 support enabled by means of
# the libx264 library.
#

if [ ! -e x264 ]; then
    echo Fetching x264...
    git clone --depth 1 https://github.com/yixia/x264.git x264
else
    echo Updating x264...
    (cd x264 && git pull)
fi

if [ ! -e ffmpeg ]; then
    echo Fetching ffmpeg
    git clone --depth 1 https://github.com/FFmpeg/FFmpeg.git ffmpeg
else
    echo Updating ffmpeg... 
    (cd ffmpeg && git pull)
fi

if [ "$ARCHS" = "" ]; then
    ARCHS="armv6 armv7a neon"
    echo ARCHS not defined. Using \"$ARCHS\"
fi

if [ "$NDKPATH" = "" ]; then
    if [ -d /opt/android-ndk ]; then
        NDKPATH=/opt/android-ndk
    elif [ -d $HOME/android-ndk ]; then
        NDKPATH=$HOME/android-ndk
    else
        echo NDKPATH not defined. Aborting.
        exit 1
    fi
fi

SYSROOT=$NDKPATH/platforms/android-19/arch-arm
# Expand the prebuilt/* path into the correct one
if [ "$(uname -m)" = "x86_64" ]; then
    TOOLCHAIN=`echo $NDKPATH/toolchains/arm-linux-androideabi-4.9/prebuilt/*-x86_64`
else
    TOOLCHAIN=`echo $NDKPATH/toolchains/arm-linux-androideabi-4.9/prebuilt/*-x86`
fi
export PATH=$TOOLCHAIN/bin:$PATH
echo $PATH

echo Using toolchain $TOOLCHAIN

if [ ! -e build ]; then
    mkdir build
fi

if [ ! -e dist ]; then
    mkdir dist
fi

. module.sh

CROSS_FLAGS="--sysroot=$SYSROOT --cross-prefix=arm-linux-androideabi-"
FFMPEG_FLAGS="$CROSS_FLAGS \
    --target-os=linux \
    --enable-gpl \
    --arch=arm \
    --enable-pic \
    --enable-shared \
    --disable-doc \
${COMMON_FF_CFG_FLAGS}
"

for arch in $ARCHS; do
    echo Building for $arch
    rm -fr build/"$arch"

    X264_DEST=build/$arch/x264
    FFMPEG_DEST=build/$arch/ffmpeg

    case $arch in
        armv7a)
            EXTRA_CFLAGS="-march=armv7-a -mfloat-abi=softfp -fPIC -DANDROID"
            EXTRA_LDFLAGS=""
            ;;
        neon)
            EXTRA_CFLAGS="-march=armv7-a -mfloat-abi=softfp -fPIC -DANDROID"
            EXTRA_CFLAGS="$EXTRA_CFLAGS -mfpu=neon"
            EXTRA_LDFLAGS="-Wl,--fix-cortex-a8"
            ;;
        armv6)
            EXTRA_CFLAGS="-march=armv6"
            EXTRA_LDFLAGS=""
            ;;
        *)
            echo Unknown platform $arch. Setting -march=$arch. Expect problems.
            EXTRA_CFLAGS="-march=$arch"
            EXTRA_LDFLAGS=""
            ;;
    esac

    # (
    #     cd x264 && pwd && CFLAGS="$EXTRA_CFLAGS" LDFLAGS="$EXTRA_LDFLAGS" \
    #     ./configure $CROSS_FLAGS --enable-static --host=arm-linux-androideabi --enable-shared --disable-asm --disable-opencl \
    #     --prefix=../$X264_DEST --enable-shared && make clean && make -j 4 && make install
    # ) || echo Failed to build x264

    EXTRA_CFLAGS="$EXTRA_CFLAGS -I../$X264_DEST/include"
    EXTRA_LDFLAGS="$EXTRA_LDFLAGS -L../$X264_DEST/lib"

    (
        cd ffmpeg && \
        ./configure $FFMPEG_FLAGS --extra-cflags="$EXTRA_CFLAGS" \
        --extra-ldflags="$EXTRA_LDFLAGS" --prefix=../$FFMPEG_DEST && \
        make clean && make -j 4 && make install
    ) || echo Failed to build ffmpeg

    VERSION=$( (cd ffmpeg && git rev-list HEAD -n 1 | cut -c 1-12) )

    (
        #cp -vfr $X264_DEST/* $tmpdir/$ffdir && \
        #tar cjvf $ffdir-bare.tar.bz2 $ffdir/bin $ffdir/lib/libx264* && \

        tmpdir=$(mktemp -d) && \
        ffdir=ffmpeg-$arch-$VERSION
        mkdir -p $tmpdir/$ffdir && \
        cp -vfr $FFMPEG_DEST/* $tmpdir/$ffdir && \
        cd $tmpdir && \
        tar cjvf $ffdir.tar.bz2 $ffdir && \
        cd - && \
        mv $tmpdir/$ffdir.tar.bz2 dist/ && \
        rm -fr $tmpdir
    ) || echo Failed to package ffmpeg + x264

done

# vim: set ts=4 sw=4 noexpandtab softtabstop=4

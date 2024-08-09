#! /bin/bash

BUILDDIR=build
BuildModel=86
function goBuildDir()
{
	if test -d $BUILDDIR;then
	  rm -r $BUILDDIR
	fi

	if ! test -d $BUILDDIR;then
	  mkdir $BUILDDIR
	fi
	cd $BUILDDIR
}
function build()
{

	goBuildDir

	cmake .. -DCMAKE_PLATFORM=$BuildModel
	
	make -j8

}

if (( $1 == 220));then
	GCC_DIR=/opt/gcc-linaro-6.2.1-2016.11-x86_64_aarch64-linux-gnu
	export CC=$GCC_DIR/bin/aarch64-linux-gnu-gcc
	export CXX=$GCC_DIR/bin/aarch64-linux-gnu-g++
	BuildModel=220
elif (( $1 == 11));then
	GCC_DIR=/opt/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu
	export CC=$GCC_DIR/bin/aarch64-linux-gnu-gcc
	export CXX=$GCC_DIR/bin/aarch64-linux-gnu-g++
	BuildModel=11
elif (( $1 == 1046));then
	GCC_DIR=/opt/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu
	export CC=$GCC_DIR/bin/aarch64-linux-gnu-gcc
	export CXX=$GCC_DIR/bin/aarch64-linux-gnu-g++
	BuildModel=1046
elif (( $1 == 86));then
	BuildModel=86
fi
echo "*************************************BuildModel:$BuildModel*************************************"
build
echo "*************************************$BUILDDIR done*************************************"

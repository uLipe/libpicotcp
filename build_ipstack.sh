#! /bin/sh

rm -rf ../ipstack
cd picotcp

CROSS_COMPILE=arm-none-eabi- ARCH=cortexm4-hardfloat make -j8 

mkdir ipstack && cd ipstack

cp -r ../build/include	include
cp -r ../build/lib	lib
cp -r ../build/modules	modules
cp -r ../../netif	netif
cp -r ../../picotcp-bsd bsd

cd ..	
make clean

cp  -r ipstack  ../../ipstack
rm -rf ipstack


cd ..










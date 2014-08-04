cberryfb
========

Linux framebuffer driver for the admatec C-Berry LCD module


## installation (cross compile on Linux)

for other platforms or distribution specific info follow the instructions on http://elinux.org/RPi_Kernel_Compilation


### setup the compiler

set the build directory. (you can change the path as you like)

    export BUILD_DIR=~/pi

create the build directory.

    mkdir $BUILD_DIR
    cd $BUILD_DIR


download the pre-built bmc2708 compiler.

    git clone git://github.com/raspberrypi/tools.git --depth 1


set the cross compiler prefix.

    export CCPREFIX=$BUILD_DIR/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-


### get the kernel source

    git clone --depth 1 git://github.com/raspberrypi/linux.git



### configure the kernel

get the pi's current kernel config (execute on the pi).

    ssh root@pi
    zcat /proc/config.gz > /root/config
    exit


you could use https://github.com/Hexxeh/rpi-update to update to the latest kernel.
this way you might not have to update as much kernel options in the next step as with the older kernel. but it is really not necessary.


copy the config to your build directory (execute on the build machine)

    scp root@pi:/root/config linux/.config



update the kernel config (make sure the above mentioned cross compiler prefix is exported)

    cd $BUILD_DIR/linux
    make ARCH=arm CROSS_COMPILE=${CCPREFIX} oldconfig


### add cberryfb

get the cberryfb source

    git clone https://github.com/u-voelkel/cberryfb.git drivers/video/cberryfb/


append to `drivers/video/Makefile`

    obj-$(CONFIG_FB_CBERRY)		  += cberryfb/

append to `drivers/video/Kconfig` (before `endmenu`)

    source "drivers/video/cberryfb/Kconfig"


select the driver

    make ARCH=arm CROSS_COMPILE=${CCPREFIX} menuconfig
    → Device Drivers → Graphics support
    <M> C-Berry LCD frame buffer support
    Exit and save


Note: you could also add the cberry sources before make oldconfig and then select it when running it.



### compile the kernel

simply run

    make ARCH=arm CROSS_COMPILE=${CCPREFIX}
    make ARCH=arm CROSS_COMPILE=${CCPREFIX} modules

if you have multiple cpus or cores you can use the `-j` option to make use of them. a rule of thumb is number of cpus/cores + 1. so on a dual core machine it would be

    make -j 3 ARCH=arm CROSS_COMPILE=${CCPREFIX}

go get yourself a coffee or a sandwich. It might take some time.



### install the kernel

first you should backup the existing kernel on the pi.

    cd /boot
    mv kernel.img kernel_old.img

copy the kernel to the pi.

    scp $BUILD_DIR/linux/arch/arm/boot/zImage root@pi:/boot/kernel.img



### install the modules

install the modules in a temporary directory.

    export MODULES_TEMP=$BUILD_DIR/modules
    mkdir $MODULES_TEMP
    make ARCH=arm CROSS_COMPILE=${CCPREFIX} INSTALL_MOD_PATH=${MODULES_TEMP} modules_install


copy the modules to the pi

    rsync --progress -a --no-l -vhe ssh $BUILD_DIR/modules/lib/ root@pi:/lib/


### done

you are done. reboot the pi to use the new kernel.

    shutdown -r now


## usage

the module is called "cberryfb"

    modprobe cberryfb
    dmesg

the last line should look like this: `fb1: admatec C-Berry LCD framebuffer device`


run the X-server

    FRAMEBUFFER=/dev/fb1 startx

mplayer

    mplayer -nolirc -vo fbdev2:/dev/fb1 -vf scale=320:-3 video.mpg

display an image using fbi

    fbi -d /dev/fb1 -T 1 -noverbose -a image.bmp


console usage is not yet supported



### backlight
You can control the backlight using the files under `/sys/class/backlight/cberryfb/`

    cat /sys/class/backlight/cberryfb/actual_brightness
    cat /sys/class/backlight/cberryfb/max_brightness

    echo "100" > /sys/class/backlight/cberryfb/brightness
    echo "0" > /sys/class/backlight/cberryfb/brightness
    echo "255" > /sys/class/backlight/cberryfb/brightness



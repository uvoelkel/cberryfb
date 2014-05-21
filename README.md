cberryfb
========

Linux frame buffer driver for the admatec C-Berry LCD module


## Installation

### get the kernel source and setup the compiler

follow the instructions on http://elinux.org/RPi_Kernel_Compilation

### add cberryfb

append to drivers/video/Makefile
    obj-$(CONFIG_FB_CBERRY)		  += cberryfb/

append to drivers/video/Kconfig
    source "drivers/video/cberryfb/Kconfig"

### compile the kernel

follow the instructions on http://elinux.org/RPi_Kernel_Compilation#Perform_the_compilation
make sure to run the "make menuconfig" step and select the cberryfb module


## Usage

the module is called "cberryfb"


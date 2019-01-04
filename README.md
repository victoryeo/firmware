This firmware repo contains all firmware related source code

i2c6408a.c - it uses i2c to drive an io expander. TCA6408a is a TI i2c to 8 pin gpio IO expander. 
This code uses i2c to driver software uart. Software uart is also known as bitbang.

my_udc.c - it is a usb device side driver for Linux.

file_storage.c - it is a USB file storage gadget driver for Linux.
    

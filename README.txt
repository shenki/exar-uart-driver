Vizzini USB Serial
==================

Version 1.0, 06/11/2013

The source code is compatible with these kernel versions (and probably others):
    3.5.0 and newer.


Installation
------------

* Make sure the Vizzini device is unplugged from the Linux host.

* Make sure that the cdc-acm driver and any previously loaded vizzini
  driver modules are not loaded.

	# rmmod cdc_acm
	# rmmod vizzini
	# modprobe -r usbserial

* Install the vizzini driver module.

	# modprobe usbserial
	# insmod ./vizzini.ko

* Plug Vizzini into the host.  You should see four devices created,
  typically /dev/ttyUSB[0-3].


Operation
---------

The vizzini driver presents a standard Linux TTY interface that can be
configured and manipulated with the usual APIs (tcgetattr(),
tcsetattr(), ioctl(), read(), write(), etc).

The normal supported character modes are 7N1, 7N2, 7P1, 7P2, 8N1, 8N2,
8P1, 8P2, with odd, even, mark and space parity.

Vizzini also supports 9N1 and 9N2.  It is enabled by using the CS5
character size.  In this mode a 9-bit character can be written to the
device with two bytes.  Bits 0..7 of the character are taken from the
first byte, and bit-8 of the character is taken from bit-0 of the
second byte.

Similarly, a 9-bit character can be read from the device as a pair of
bytes.  Bits 0..7 of the character are in the first byte, and bit-8 of
the character is taken from bit-0 of the second byte.  Bits 1..7 of
the second byte are undefined.


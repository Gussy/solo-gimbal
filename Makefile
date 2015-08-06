all: GimbalFirmware Bootloader

GimbalFirmware:
	$(MAKE) -C GimbalFirmware
Bootloader: GimbalFirmware
	$(MAKE) -C Bootloader

clean:
	$(MAKE) -C GimbalFirmware clean
	$(MAKE) -C Bootloader clean

.PHONY: all clean GimbalFirmware Bootloader
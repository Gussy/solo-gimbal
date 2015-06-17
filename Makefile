all: GimbalFirmware AZBootloader Bootloader

GimbalFirmware:
	$(MAKE) -C GimbalFirmware
AZBootloader: GimbalFirmware
	$(MAKE) -C AZBootloader
Bootloader: GimbalFirmware
	$(MAKE) -C Bootloader

clean:
	$(MAKE) -C GimbalFirmware clean
	$(MAKE) -C AZBootloader clean
	$(MAKE) -C Bootloader clean

.PHONY: all clean GimbalFirmware AZBootloader Bootloader
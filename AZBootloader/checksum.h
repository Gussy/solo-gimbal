#ifndef CHECKSUM_H_
#define CHECKSUM_H_

#define BOOTLOADER_KEY_VALUE_8BIT	0x08AA

void reset_datapointer(void);
int verify_data_checksum(void);

extern unsigned int location;

#endif /* CHECKSUM_H_ */

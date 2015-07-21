#ifndef CHECKSUM_H_
#define CHECKSUM_H_

#define BOOTLOADER_KEY_VALUE_8BIT	0x08AA

Uint16 read_Data_and_Send();
void reset_datapointer(void);
int verify_data_checksum(void);

#endif /* CHECKSUM_H_ */

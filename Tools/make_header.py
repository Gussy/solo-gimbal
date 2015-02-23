#!/usr/bin/python

"""
Utility for generating a C-style header file containing a binary
application image as an array.

Used for packaging application firmware with the bootloader firmware.

@author: Angus Peart (angus@3dr.com)
"""

import sys, argparse

def load_binary(filename):
	'''Load binary image file into a byte array'''
	with open(filename, "rb") as f:
		return bytearray(f.read())

def add_checksum(checksum, word):
	'''Simple XOR checksum'''
	checksum ^= word
	return checksum

def write_data_header(filename, data):
	'''Write data.h file to be bundled with bootloader firmware'''
	bytes_per_line = 12
	checksum = 0xFFFF
	with open(filename, 'w') as f:
		# Write out the standard header
		f.write("#pragma	DATA_SECTION(DATA,\".dlcode\");\n")
		f.write("extern const unsigned short DATA[] = {\n")
		
		for i in range(len(data)):
			f.write("\t")

			# Output word to file and add to checksum
			f.write("0x%04X," % data[i])
			checksum = add_checksum(checksum, data[i])

			# Carrige return every 12 bytes
			if (i+1) % bytes_per_line == 0:
				f.write("\n")

		# Write the checksum bytes
		f.write(" 0x%04X, 0x%04X, 0x0000};\n" % (checksum&0xFFFF, (checksum&0xFFFF)>>16))

def bytearray_to_wordarray(data):
	'''Converts an 8-bit byte array into a 16-bit word array'''
	wordarray = list()

	for i in range(len(data)/2):
		# Calculate 16 bit word from two bytes
		msb = data[(i*2)+0]
		lsb = data[(i*2)+1]
		word = (msb << 8) | lsb
		wordarray.append(word)

	return wordarray

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("infile", help="binary hex file to generate C header file from")
	parser.add_argument("outfile", help="filename for generated C header file")
	args = parser.parse_args()

	binary = load_binary(args.infile)

	write_data_header(args.outfile, bytearray_to_wordarray(binary))

if __name__ == '__main__':
	main()
#!/usr/bin/env python

import argparse, re, sys

def getSection(text, name):
	re_section = re.compile(ur"^ *(?P<name>%s) *(?P<origin>[0-9a-f]+) *(?P<length>[0-9a-f]+) *(?P<used>[0-9a-f]+) *(?P<unused>[0-9a-f]+).*$" % name, re.MULTILINE)
	section = [m.groupdict() for m in re_section.finditer(text)][0]
	section['origin'] = int(section['origin'], 16)
	section['length'] = int(section['length'], 16)
	section['used'] = int(section['used'], 16)
	section['unused'] = int(section['unused'], 16)
	section['percentage'] = float(float(section['used']) / float(section['length'])) * 100
	return section

if __name__ == '__main__':
	# Parse commandline arguments
	parser = argparse.ArgumentParser(description="Utility for generating code usage metrics")
	parser.add_argument("mapfile", help="input memory map file")
	parser.add_argument("--project", help="project [AZBootloader, Bootloader, PM_Sensorless_F2806x]", type=str)
	args = parser.parse_args()

	# Read the map file
	mapFile = open(args.mapfile, 'r')
	mapFileText = mapFile.read()
	mapFile.close()

	if args.project == 'AZBootloader':
		sections = ['RAMM0', 'FLASHA', 'FLASHBCDEFG']
	elif args.project == 'PM_Sensorless_F2806x':
		sections = ['RAMM0', 'progRAM']
	elif args.project == 'Bootloader':
		sections = ['RAMM0', 'FLASHA']
	else:
		print("project '%s' not supported" % args.project)
		sys.exit(1)
	
	outputBuffer = list()
	outputBuffer.append("\nMemory usage statistics for '%s'" % args.mapfile)

	tableData = list()
	tableData.append(['Section', 'Length', 'Used', 'Unused', 'Utilization'])
	for s in sections:
		info = getSection(mapFileText, s)
		tableData.append([s, info['length'], info['used'], info['unused'], "{0:.2f}%".format(info['percentage'])])

	for row in tableData:
		outputBuffer.append("{: <15} {: <8} {: <8} {: <8} {: <15}".format(*row))
	outputBuffer.append('')

	# Print all output at once to avoid making a mess of parallel build outputs
	print('\n'.join(outputBuffer))
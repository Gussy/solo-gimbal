import argparse, re, os, csv, json

parser = argparse.ArgumentParser()
parser.add_argument("dir", help="input directory", default=None)
parser.add_argument("outfile", help="input directory", default=None)
args = parser.parse_args()

all_results = list()
for fn in os.listdir(args.dir):
    filename = os.path.join(args.dir, fn)
    if os.path.isfile(filename):

        # Skip files that aren't JSON
        fileExtension = filename.split(".")[-1].lower()
        if fileExtension != "json":
            continue

        with open(filename, 'r') as f:
            params = json.load(f)

            results = dict()

            results['serial_number'] = params['serial_number']

            results['joint_x'] = params['joint']['x']
            results['joint_y'] = params['joint']['y']
            results['joint_z'] = params['joint']['z']

            results['gyro_x'] = params['gyro']['x']
            results['gyro_y'] = params['gyro']['y']
            results['gyro_z'] = params['gyro']['z']

            results['pitch_icept'] = params['pitch']['icept']
            results['pitch_slope'] = params['pitch']['slope']
            results['roll_icept'] = params['roll']['icept']
            results['roll_slope'] = params['roll']['slope']
            results['yaw_icept'] = params['yaw']['icept']
            results['yaw_slope'] = params['yaw']['slope']

            all_results.append(results)

listWriter = csv.DictWriter(
   open(args.outfile, 'wb'),
   fieldnames=all_results[0].keys(),
   delimiter=',',
   quotechar='|',
   quoting=csv.QUOTE_MINIMAL
)

listWriter.writeheader()

for result in all_results:
    listWriter.writerow(result)

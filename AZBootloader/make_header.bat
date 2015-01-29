cd ..
del data.h
C:\ti\ccsv5\tools\compiler\c2000_6.2.0\bin\hex2000.exe "H:\Repos\3DRobotics\git\3DRGimbal\F2806x_RAM\PM_Sensorless_F2806x.out" -o data.hh -boot -gpio8 -a 
Z:\Software\Perl\bin\perl.exe make_header.pl
del data.hh
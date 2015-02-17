cd ..
del data.h
C:\ti\ccsv6\tools\compiler\c2000_6.2.10\bin\hex2000.exe "GimbalFirmware\F2806x_RAM\PM_Sensorless_F2806x.out" -o data.bin -boot -gpio8 -b 
python tools\make_header.py data.bin data.h
del data.bin
pause
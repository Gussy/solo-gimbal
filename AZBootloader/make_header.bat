cd ..
del data.h
C:\ti\ccsv6\tools\compiler\c2000_6.2.10\bin\hex2000.exe "C:\Users\angus_000\Google Drive\Gimbal\solo-gimbal-aes\GimbalFirmware\F2806x_RAM\PM_Sensorless_F2806x.out" -o data.hh -boot -gpio8 -a 
C:\Perl64\bin\perl.exe make_header.pl
del data.hh
pause
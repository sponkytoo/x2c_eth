@echo off

set PROJECT_ROOT=C:\microchip\harmony\v1_07_01\apps\x2c\Blinky_Microchip_PIC32MZEF_MEB2\firmware\Blinky_Microchip_PIC32MZEF_MEB2.X
set MPLABX_ROOT=C:\Program Files (x86)\Microchip\MPLABX\v3.26
set DEVICE=P32MZ2048EFH144
set TOOL=SBUR153373130
set HEXFILE=/dist/default/production/Blinky_Microchip_PIC32MZEF_MEB2.X.production.hex

:: log file to current users' desktop
::set BATCHLOG=%USERPROFILE%\Desktop\batchlog.txt
set BATCHLOG=%PROJECT_ROOT%\batchlog.txt
set MAKEFILE=%PROJECT_ROOT%\Makefile


:: make command
set MAKECMD="%MPLABX_ROOT%\gnuBins\GnuWin32\bin\make" -f %PROJECT_ROOT%\Makefile 

date /T > %BATCHLOG%
time /T >> %BATCHLOG%
echo Current directory: %cd% >> %BATCHLOG%
echo Makefile: %MAKEFILE% >> %BATCHLOG%
echo Hexfile: %HEXFILE% >> %BATCHLOG%



cd %PROJECT_ROOT%

ECHO DATE /T
ECHO TIME /T
ECHO *****************************************************
ECHO ** make the project                                **
ECHO *****************************************************
echo Execute make: >> %BATCHLOG%
echo %MAKECMD% >> %BATCHLOG%

%MAKECMD%

echo *****************************************************
echo ** program the device                              **
java -jar "%MPLABX_ROOT%\mplab_ipe\ipecmd.jar" /%DEVICE% /"F.%HEXFILE%" /T%TOOL% /M /OL

::pause

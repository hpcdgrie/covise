@echo off
set COVISEDIR=%1%
set ODDLOTDIR=%COVISEDIR%
set VCPKG_ROOT=d:\src\gitbase\vcpkg
set QT_AUTO_SCREEN_SCALE_FACTOR=1
set EXTERNLIBS=%3%
set ARCHSUFFIX=%2%
call %COVISEDIR%\winenv.bat %ARCHSUFFIX%
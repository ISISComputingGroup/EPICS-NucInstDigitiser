@echo off
setlocal
set "PATH=%~dp0..\..\libzmq\master\bin\%EPICS_HOST_ARCH%;%PATH%"
%~dp0bin\%EPICS_HOST_ARCH%\nidg_stream.exe %*

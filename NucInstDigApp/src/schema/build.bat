setlocal
set "PATH=%~dp0..\..\..\..\..\flatbuffers\master\bin\%EPICS_HOST_ARCH%;%PATH%"
flatc.exe --cpp frame_metadata_v2.fbs dat2_digitizer_analog_trace_v2.fbs dev2_digitizer_event_v2.fbs
copy /y *.h ..

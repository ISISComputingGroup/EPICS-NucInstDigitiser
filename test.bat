call nidg_send.bat "execute_cmd" "tcp://172.16.105.186:5557" "start_acquisition"
call nidg_send.bat "get_parameter" "tcp://172.16.105.186:5557" "mp.gain"  0
call nidg_send.bat "read_data" "tcp://172.16.105.186:5557" "get_darkcount_spectra"

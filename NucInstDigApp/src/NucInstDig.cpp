#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <strstream>
#include <map>
#include <vector>
#include <iomanip>
#include <sys/timeb.h>
#include <numeric>
#include <boost/algorithm/string.hpp>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>

#include <zmq.hpp>
#include <flatbuffers/flatbuffers.h>
#include "dat1_digitizer_analog_trace_v1_generated.h"
#include "dev1_digitizer_event_v1_generated.h"

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <initHooks.h>
#include <iocsh.h>

#include <macLib.h>
#include <epicsGuard.h>

#include "envDefs.h"
#include "errlog.h"

#include "utilities.h"
#include "pugixml.hpp"

#include "NucInstDig.h"
#include <epicsExport.h>

static epicsThreadOnceId onceId = EPICS_THREAD_ONCE_INIT;

static const char *driverName="NucInstDig";

void NucInstDig::setADAcquire(int addr, int acquire)
{
    int adstatus;
    int acquiring;
    int imageMode;
    asynStatus status = asynSuccess;

    /* Ensure that ADStatus is set correctly before we set ADAcquire.*/
		getIntegerParam(addr, ADStatus, &adstatus);
		getIntegerParam(addr, ADAcquire, &acquiring);
		getIntegerParam(addr, ADImageMode, &imageMode);
		  if (acquire && !acquiring) {
			setStringParam(addr, ADStatusMessage, "Acquiring data");
			setIntegerParam(addr, ADStatus, ADStatusAcquire); 
			setIntegerParam(addr, ADAcquire, 1); 
            setIntegerParam(addr, ADNumImagesCounter, 0);
		  }
		  if (!acquire && acquiring) {
			setIntegerParam(addr, ADAcquire, 0); 
			setStringParam(addr, ADStatusMessage, "Acquisition stopped");
			if (imageMode == ADImageContinuous) {
			  setIntegerParam(addr, ADStatus, ADStatusIdle);
			} else {
			  setIntegerParam(addr, ADStatus, ADStatusAborted);
			}
		  }
}

void NucInstDig::setShutter(int addr, int open)
{
    int shutterMode;

    getIntegerParam(addr, ADShutterMode, &shutterMode);
    if (shutterMode == ADShutterModeDetector) {
        /* Simulate a shutter by just changing the status readback */
        setIntegerParam(addr, ADShutterStatus, open);
    } else {
        /* For no shutter or EPICS shutter call the base class method */
        ADDriver::setShutter(open);
    }
}


void NucInstDig::setup()
{
      rapidjson::Value value;  
      try {
        executeCmd("stop_acquisition");
      }
      catch(...) {
          ;
      }
//    s=sdk.get_parameter("dgtz.info.section")
//    section = int(float(s))
//    sn = sdk.get_parameter("system.serialnumber")
//    swver = sdk.get_parameter("system.swversion")
//    compile_data = sdk.get_parameter("system.compile_data")
//    fwver = sdk.get_parameter("dgtz.probes.fwver")

      getParameter("dgtz.probes.fwver", value);
      std::cerr << "fwver " << value.GetString() << std::endl;      


//    print("sn: " + sn + "  section: " + str(section))
//    print("SW-VER: " + swver + " (" + compile_data + ")  -- FPGA-VER: "  + fwver)

    // digitizer configuration
    // the delay minimum between trigger and status packet redout
    setParameter("dgtz.send_delay", "0");
    //set signal polarity to negative "pos" / "neg"
    setParameter("in.polarity", "neg");
    // pre-trigger buffer len (us)
    setParameter("dgtz.pre", 0.5);
    // post-trigger acquisition buffer (us)
    setParameter("dgtz.post", 20);
    // delay on external trigger (us)
    setParameter("dgtz.trg_delay", 0);
    // trigger source: "ext_trigger", "self_le", "self_de", "periodic", "manual", "lemo_0"
    setParameter("trg.mode", "periodic");
    // internal trigger mode: "or", "and2", "and"
    setParameter("trg.self_coinc", "or");
    // periodic trigger rate (Hz)
    setParameter("trg.self_rate", 50);
    // trigger inibition (ns)
    setParameter("trg.trigger_inib", 10);

    // parameter those are different for each channel of the digitizer
    for(int i=0; i<8; ++i) {
        // trigger threshold (LSB)
        setParameter("trg.threshold", 2000, i);
        // trigger mask (LSB)
        setParameter("trg.mask", 0, i);
        // digital input offset
        //setParameter("in.offset", -1350, i)
        setParameter("in.offset", 0, i);
        // trigger threshold (LSB)
        //setParameter("in.chmap", CH_MAP[idx*8+i], i);
        // for us idx=0 and CH_MAP = range(0,32)
        setParameter("in.chmap", 8 * m_dig_idx + i, i);        
    }

    // configure BIAS
    setParameter("stave.BIAS.enable", "false",0);
    setParameter("stave.BIAS.V", 59,0);
    setParameter("stave.BIAS.max_v", 62,0);
    setParameter("stave.BIAS.max_i", 3,0);

    setParameter("stave.BIAS.enable", "false",1);
    setParameter("stave.BIAS.V", 52,1);
    setParameter("stave.BIAS.max_v", 58,1);
    setParameter("stave.BIAS.max_i", 3,1);

    //    # configure BIAS compensation
    for(int i=0; i<24; ++i) {
        // correction_mode "off", "auto", "manual"
        setParameter("stave.BIAS.correction_mode", "manual",i);
        setParameter("stave.BIAS.correction_manual", 0.2,i);
        // SiPM correction coefficient in V/°C
        setParameter("stave.BIAS.correction_coeff", 0.035);
        // offset in V
        setParameter("stave.offset", 0,i);
    }


    // multi-photon processing
    // spectrum readout mode: "manual", "auto"
    setParameter("mp.spectrum_readout_mode", "manual");
    // spectrum algorithm: "charge_integrator", "peak_holder"
    setParameter("mp.spectrum_mode", "charge_integrator");
    // periodic spectrum send and reset
    setParameter("mp.auto_spectrum_time", 5000);
    // acquisition gate length (ms)
    setParameter("mp.gate_len", 30);
    // delay from trigger (ms)
    setParameter("mp.delay_from_trigger", 0.5);
    // self trigger inibhit (us)
    setParameter("mp.trigger_inib", 0);
    // baseline length (samples)
    setParameter("mp.bl_len", 0.05);
    // baseline hold (us)
    setParameter("mp.bl_hold", 0.05);
    // integration pre trigger
    setParameter("mp.int_pre", 0.015);//      #25u 0.015 #50u 0.025
    // integration post trigger
    setParameter("mp.int_post", 0.100);//  #25u 0.015 #50u 0.1
    // peak detector search window
    setParameter("mp.peak_detector_window", 0.2);
    for(int i=0; i<8; ++i) {
        // enable multi-photon spectrum
        setParameter("mp.enable", "true", i);
        // gain of charge integration
        setParameter("mp.gain", 2, i);
        // single photon threshold
        setParameter("mp.threshold", 15, i);

        setParameter("mp.offset", 150, i);
        // offset of spectrum
    }

    // configure io
    // configure digitizer lemo mode "in_h", "in_50", "out"
    setParameter("dgtz.lemo.mode", "in_50", 0);
    setParameter("dgtz.lemo.mode", "out", 1);

    // configure digitizer lemo output source
    // gnd", "high", "t0_out", "trigger_out", "tot_ch0",
    // "tot_ch1","tot_ch2","tot_ch3","tot_ch4","tot_ch5","tot_ch6","tot_ch7",
    // "run", "busy", "acquisition","mp_gate"
    setParameter("dgtz.lemo.source", "tot_ch0", 0);
    setParameter("dgtz.lemo.source", "trigger_out", 1);

    // configure digitizer sync_out output source
    // gnd", "high", "t0_out", "trigger_out", "tot_ch0",
    // "tot_ch1","tot_ch2","tot_ch3","tot_ch4","tot_ch5","tot_ch6","tot_ch7",
    // "run", "busy", "acquisition","mp_gate"
    //setParameter("dgtz.sync.outmode", "tot_ch0", 0);
    //setParameter("dgtz.sync.outmode", "tot_ch1", 1);
    //setParameter("dgtz.sync.outmode", "tot_ch2", 2);
    //setParameter("dgtz.sync.outmode", "tot_ch3", 3);
    //setParameter("dgtz.sync.outmode", "tot_ch4", 4);
    //setParameter("dgtz.sync.outmode", "tot_ch5", 5);
    //setParameter("dgtz.sync.outmode", "tot_ch6", 6);
    //setParameter("dgtz.sync.outmode", "tot_ch7", 7);
    for(int i=0; i<8; ++i) {
        setParameter("dgtz.sync.outmode", "gnd", i);
    }
    setParameter("dgtz.sync.outmode", "trigger_out", 0);

    // configure base lemo mode "in", "out"
    for(int i=0; i<16; ++i) {
        setParameter("base.lemo.mode", "in", i);
    }

    // configure base lemo source
    // "gnd", "high", "t0_out", "common_trigger", "clk_in", "busy", "status_packet",
    // "sync_a_0","sync_a_1", "sync_a_2", "sync_a_3", "sync_a_4", "sync_a_5", "sync_a_6", "sync_a_7",
    // "sync_b_0","sync_b_1", "sync_b_2", "sync_b_3", "sync_b_4", "sync_b_5", "sync_b_6", "sync_b_7",
    // "sync_c_0","sync_c_1", "sync_c_2", "sync_c_3", "sync_c_4", "sync_c_5", "sync_c_6", "sync_c_7",
    // "sync_d_0","sync_d_1", "sync_d_2", "sync_d_3", "sync_d_4", "sync_d_5", "sync_d_6", "sync_d_7"

    //for(int i=0; i<8; ++i) {
    //    setParameter("base.lemo.source", "sync_a_" + std::to_string(i), i);
    //}
    //for(int i=0; i<8; ++i) {
    //    setParameter("base.lemo.source", "sync_b_" + std::to_string(i), i);
    //}
    for(int i=0; i<16; ++i) {
        setParameter("base.lemo.source", "gnd", i);
    }

    // configure base sync source
    // "gnd", "high", "common_trigger",
    // "lemo_0", "lemo_1", "lemo_2", "lemo_3", "lemo_4", "lemo_5", "lemo_6", "lemo_7",
    // "lemo_8", "lemo_9", "lemo_10", "lemo_11", "lemo_12", "lemo_13", "lemo_14", "lemo_15"
    for(int i=0; i<8; ++i) {
        setParameter("base.sync.outmode", "gnd", i);
    }

    setParameter("base.stave.power", "true", 0);
    setParameter("base.stave.power", "true", 1);

    setParameter("base.pulsegen.freq", 100000.0, 0);

    // configure status packet frame source uart
    // "lemo_0", "lemo_4", "lemo_8", "lemo_12", "rj45_lvds_0", "lvds_0", "lvds_8", "lvds_16", "lvds_24"
    //  "frame_usart_tx"
    setParameter("base.sp_rx.frame_source", "frame_usart_tx", 0);
    
   // configure status packet veto source uart
    // "lemo_1", "lemo_5", "lemo_9", "lemo_13", "rj45_lvds_1", "lvds_1", "lvds_9", "lvds_17", "lvds_25"
    // "veto_usart_tx"
    setParameter("base.sp_rx.veto_source", "veto_usart_tx", 0);
    // configure T0 source
    // "gnd", "high", "t0_self", "rj45_lvds_0", "rj45_lvds_1", "rj45_lvds_2", "rj45_lvds_3",
    // "lvds_0", "lvds_1", "lvds_2", "lvds_3", "lvds_28", "lvds_29", "lvds_30", "lvds_31",
    // "lemo_0", "lemo_1", "lemo_2", "lemo_3", "lemo_4", "lemo_5", "lemo_6", "lemo_7",
    // "lemo_8", "lemo_9", "lemo_10", "lemo_11", "lemo_12", "lemo_13", "lemo_14", "lemo_15"
    setParameter("base.t0.source", "lemo_0", 0);

    setParameter("base.t0.freq", 25.0, 0);
    
        // configure common_clk source
    // "clk_int", "clk_ext", "rj45_lvds_0", "rj45_lvds_1", "rj45_lvds_2", "rj45_lvds_3",
    // "lvds_16", "lvds_26"
    setParameter("base.common_clock.source", "clk_int", 0);


    // configure adc_sync source
    // "internal", "gnd", "high", "rj45_lvds_0", "rj45_lvds_1", "rj45_lvds_2", "rj45_lvds_3",
    // "lvds_6", "lvds_14", "lvds_23", "lvds_27",
    // "lemo_0", "lemo_1", "lemo_2", "lemo_3", "lemo_4", "lemo_5", "lemo_6", "lemo_7",
    // "lemo_8", "lemo_9", "lemo_10", "lemo_11", "lemo_12", "lemo_13", "lemo_14", "lemo_15"

    setParameter("base.adc_sync.source", "internal", 0);
    
    
    // configure emulator
    // trigger inibition (ns)
    for(int i=0; i<8; ++i) {
        setParameter("dgtz.emu.amp", 100+100*i,i);
        setParameter("dgtz.emu.period", 5000,i);
    }
    setParameter("dgtz.emu.noiseamp", 0);
    setParameter("dgtz.emu.offset", 0);
    setParameter("dgtz.emu.enable_pulse", "false");
    setParameter("dgtz.emu.enable", "false");

    // test parameter readback
    for(int i=0; i<16; ++i) {
        getParameter("base.lemo.mode", value);
        std::cerr << i << " " << value.GetString() << std::endl;
        getParameter("base.lemo.source", value);
        std::cerr << i << " " << value.GetString() << std::endl;
    }
    for(int i=0; i<8; ++i) {
        getParameter("base.sync.outmode", value);
        std::cerr << i << " " << value.GetString() << std::endl;
    }
    //for i in range(0, 8):
        //print(sdk.get_parameter("base.sync.outmode"),i);

    //print(float(sdk.get_parameter("mp.gain")));

    // configure event processor (fixed threshold)
    setParameter("sw_process.enable", "false");
    setParameter("sw_process.threshold", 520, 0);
    setParameter("sw_process.threshold", 500, 1);
    setParameter("sw_process.threshold", 520, 2);
    setParameter("sw_process.threshold", 460, 3);
    setParameter("sw_process.threshold", 3000, 4);
    setParameter("sw_process.threshold", 3000, 5);
    setParameter("sw_process.threshold", 3000, 6);
    setParameter("sw_process.threshold", 3000, 7);
    setParameter("sw_process.hist", 10);
    setParameter("base.stave.power", "true", 0);

    executeCmd("configure_dgtz");
    executeCmd("configure_base");
    executeCmd("configure_hv");
    executeCmd("configure_staves");

    executeCmd("reset_darkcount_spectra");

    // print(sdk.read_data("get_darkcount_spectra"))
    // print(sdk.read_data("get_waveforms"))
    executeCmd("start_acquisition");

}


asynStatus NucInstDig::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    const char* functionName = "writeFloat64";
    asynStatus stat = asynSuccess;
    const char *paramName = NULL;
    int function = pasynUser->reason;
	getParamName(function, &paramName);
    try {
        if (function < FIRST_NUCINSTDIG_PARAM)
        {
            return ADDriver::writeFloat64(pasynUser, value);
        }
        else
        {
            auto it = m_param_data.find(function);
            if (it != m_param_data.end())
            {
                const ParamData* p = it->second;
                setParameter(p->name, value, p->chan);
            }
        }    
        if (stat == asynSuccess)
        {
            stat = asynPortDriver::writeFloat64(pasynUser, value); // to update parameter and do callbacks
        }
        return stat;
    }
    catch(const std::exception& ex)
    {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%f, error=%s", 
                  driverName, functionName, stat, function, paramName, value, ex.what());
        return asynError;
    }
}

asynStatus NucInstDig::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const char* functionName = "writeInt32";
    asynStatus stat = asynSuccess;
    const char *paramName = NULL;
    int function = pasynUser->reason;
    int addr = 0;
	getAddress(pasynUser, &addr);
	getParamName(function, &paramName);
    try {
        if (function == ADAcquire)
        {            
            setADAcquire(addr, value);
            // fall through to next line to call base class
        }
        if (function < FIRST_NUCINSTDIG_PARAM)
        {
            return ADDriver::writeInt32(pasynUser, value);
        }
        asynStatus stat = asynSuccess;
        if (function == P_startAcquisition) {
            std::cerr << "Starting digitiser acquisition" << std::endl;
            executeCmd("start_acquisition", "");
        }
        else if (function == P_stopAcquisition) {
            std::cerr << "Stopping digitiser acquisition" << std::endl;
            executeCmd("stop_acquisition", "");
        }
        else if (function == P_resetDCSpectra) {
            executeCmd("reset_darkcount_spectra", "");
        }
        else if (function == P_configDGTZ) {
            executeCmd("configure_dgtz", "");
        }
        else if (function == P_configBASE) {
            executeCmd("configure_base", "");
        }
        else if (function == P_configHV) {
            executeCmd("configure_hv", "");
        }
        else if (function == P_configSTAVES) {
            executeCmd("configure_staves", "");
        }
        else if (function == P_setup) {
            setup();
        }
        else if (function >= P_DCSpecIdx[0] && function <= P_DCSpecIdx[3]) {
            int idx = function - P_DCSpecIdx[0];
            m_DCSpecIdx[idx] = value;
        }
        else if (function >= P_traceIdx[0] && function <= P_traceIdx[3]) {
            int idx = function - P_traceIdx[0];
            m_traceIdx[idx] = value;
        }
        else
        {
            auto it = m_param_data.find(function);
            if (it != m_param_data.end())
            {
                const ParamData* p = it->second;
                setParameter(p->name, value, p->chan);
            }
        }
        if (stat == asynSuccess)
        {
            stat = asynPortDriver::writeInt32(pasynUser, value); // to update parameter and do callbacks
        }
        return stat;
    }
    catch(const std::exception& ex)
    {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%d, error=%s", 
                  driverName, functionName, stat, function, paramName, value, ex.what());
        return asynError;
    }
}

asynStatus NucInstDig::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
	int function = pasynUser->reason;
	if (function < FIRST_NUCINSTDIG_PARAM)
	{
		return ADDriver::readFloat64Array(pasynUser, value, nElements, nIn);
	}
    asynStatus stat = asynSuccess;
	callParamCallbacks();
	doCallbacksFloat64Array(value, *nIn, function, 0);
    return stat;
}

asynStatus NucInstDig::readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn)
{
	int function = pasynUser->reason;
	if (function < FIRST_NUCINSTDIG_PARAM)
	{
		return ADDriver::readInt32Array(pasynUser, value, nElements, nIn);
	}
    asynStatus stat = asynSuccess;
	callParamCallbacks();
	doCallbacksInt32Array(value, *nIn, function, 0);
    return stat;
}

asynStatus NucInstDig::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
	int function = pasynUser->reason;
	const char *functionName = "readFloat64";
    const char *paramName = NULL;
	getParamName(function, &paramName);
	try
	{
	    if (function < FIRST_NUCINSTDIG_PARAM)
	    {
	        return ADDriver::readFloat64(pasynUser, value);
	    }
	    else
	    {
	        return asynPortDriver::readFloat64(pasynUser, value);
        }
	}
	catch(const std::exception& ex)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: function=%d, name=%s, error=%s", 
                  driverName, functionName, function, paramName, ex.what());
		return asynError;
	}
}

asynStatus NucInstDig::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
	int function = pasynUser->reason;
	const char *functionName = "readInt32";
    const char *paramName = NULL;
	getParamName(function, &paramName);
	try
	{
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName, functionName, function, paramName, *value);
	    if (function < FIRST_NUCINSTDIG_PARAM)
	    {
	        return ADDriver::readInt32(pasynUser, value);
	    }
        else
        {
	        return asynPortDriver::readInt32(pasynUser, value);
        }
	}
	catch(const std::exception& ex)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: function=%d, name=%s, error=%s", 
                  driverName, functionName, function, paramName, ex.what());
		return asynError;
	}
}

asynStatus NucInstDig::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
	int function = pasynUser->reason;
	int status=0;
	const char *functionName = "readOctet";
    const char *paramName = NULL;
	std::string value_s;
	getParamName(function, &paramName);
	try
	{
	    if (function < FIRST_NUCINSTDIG_PARAM)
	    {
	        return ADDriver::readOctet(pasynUser, value, maxChars, nActual, eomReason);
	    }
	    else
	    {
	        return asynPortDriver::readOctet(pasynUser, value, maxChars, nActual, eomReason);
        }
	}
	catch(const std::exception& ex)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=\"%s\", error=%s", 
                  driverName, functionName, status, function, paramName, value_s.c_str(), ex.what());
		*nActual = 0;
		if (eomReason) { *eomReason = ASYN_EOM_END; }
		value[0] = '\0';
		return asynError;
	}
}

asynStatus NucInstDig::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName = NULL;
	getParamName(function, &paramName);
    const char* functionName = "writeOctet";
	std::string value_s;
    // we might get an embedded NULL from channel access char waveform records
    if ( (maxChars > 0) && (value[maxChars-1] == '\0') )
    {
        value_s.assign(value, maxChars-1);
    }
    else
    {
        value_s.assign(value, maxChars);
    }
	try
	{
	    if (function < FIRST_NUCINSTDIG_PARAM)
		{
		    status = ADDriver::writeOctet(pasynUser, value_s.c_str(), value_s.size(), nActual);
		}
        else
        {
            auto it = m_param_data.find(function);
            if (it != m_param_data.end())
            {
                const ParamData* p = it->second;
                setParameter(p->name, value_s, p->chan);
            }
        }    
		status = asynPortDriver::writeOctet(pasynUser, value_s.c_str(), value_s.size(), nActual); // update parameters and do callbacks
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%s\n", 
              driverName, functionName, function, paramName, value_s.c_str());
        if (status == asynSuccess && *nActual == value_s.size())
        {
		    *nActual = maxChars;   // to keep result happy in case we skipped an embedded trailing NULL when creating value_s
        }
		return status;
	}
	catch(const std::exception& ex)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%s, error=%s", 
                  driverName, functionName, status, function, paramName, value_s.c_str(), ex.what());
		*nActual = 0;
		return asynError;
	}
	catch(...)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%s, error=unknow exception", 
                  driverName, functionName, status, function, paramName, value_s.c_str());
		*nActual = 0;
		return asynError;
	}
}


void NucInstDig::updateTraces()
{
    epicsTimeStamp last_update, now;
    epicsTimeGetCurrent(&last_update);
    while(true)
    {
        try {
        zmq::message_t reply{};
        m_zmq_stream_socket.recv(reply, zmq::recv_flags::none);
        auto msg = GetDigitizerAnalogTraceMessage(reply.data());
        auto channels = msg->channels();
        for(int i=0; i<channels->size(); ++i) {
            int chan = channels->Get(i)->channel();
            auto voltages = channels->Get(i)->voltage();
            {
                epicsGuard<epicsMutex> _lock(m_tracesLock);
                m_nVoltage = voltages->size();
                m_traces.resize(m_NTRACE * m_nVoltage);
                if (chan < m_NTRACE) {
                    for(int k=0; k<m_nVoltage; ++k) {
                        m_traces[chan * m_nVoltage + k] = voltages->Get(k);
                    }
                }
            }
            for(size_t j=0; j<4; ++j) {
                if (chan == m_traceIdx[j]) {
                    m_traceX[j].resize(m_nVoltage);
                    m_traceY[j].resize(m_nVoltage);
                    for(int k=0; k<m_nVoltage; ++k) {
                       m_traceX[j][k] = k;
                       m_traceY[j][k] = m_traces[chan * m_nVoltage + k];
                    }
                }
            }
        }
        epicsTimeGetCurrent(&now);
        if (epicsTimeDiffInSeconds(&now, &last_update) > 0.5)
        {
            lock();
            for(size_t j=0; j<4; ++j) {
                doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_traceX[j].data()), m_traceX[j].size(), P_traceX[j], 0);
                doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_traceY[j].data()), m_traceY[j].size(), P_traceY[j], 0);
            }
            unlock();
            last_update = now;
        }
        }
        catch(const std::exception& ex)
        {
            std::cerr << "updaetTraces " << ex.what() << std::endl;
            epicsThreadSleep(3.0);            
        }
        catch(...)
        {
            std::cerr << "update traces exception"  << std::endl;
            epicsThreadSleep(3.0);            
        }
    }
}


void NucInstDig::updateAD()
{
    static const char* functionName = "isisdaePoller4";
	int acquiring, enable;
	int all_acquiring, all_enable;
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    NDArray *pImage;
    double acquireTime, acquirePeriod, delay, updateTime;
    epicsTimeStamp startTime, endTime;
    double elapsedTime;
    std::vector<int> old_acquiring(maxAddr, 0);
	std::vector<epicsTimeStamp> last_update(maxAddr);

	memset(&(last_update[0]), 0, maxAddr * sizeof(epicsTimeStamp));	
	while(true)
	{
		all_acquiring = all_enable = 0;
		for(int i=0; i<maxAddr; ++i)
		{
		    epicsGuard<NucInstDig> _lock(*this);
			try 
			{
				acquiring = enable = 0;
                if (i == 0) {
                    getIntegerParam(P_readDCSpectra, &enable);
                } else {
                    enable = 1; // traces always enabled
                }
				getIntegerParam(i, ADAcquire, &acquiring);
				getDoubleParam(i, ADAcquirePeriod, &acquirePeriod);
				
				all_acquiring |= acquiring;
				all_enable |= enable;
				if (acquiring == 0 || enable == 0)
				{
					old_acquiring[i] = acquiring;
	//				epicsThreadSleep( acquirePeriod );
					continue;
				}
				if (old_acquiring[i] == 0)
				{
					setIntegerParam(i, ADNumImagesCounter, 0);
					old_acquiring[i] = acquiring;
				}
				setIntegerParam(i, ADStatus, ADStatusAcquire); 
				epicsTimeGetCurrent(&startTime);
				getIntegerParam(i, ADImageMode, &imageMode);

				/* Get the exposure parameters */
				getDoubleParam(i, ADAcquireTime, &acquireTime);  // not really used

				setShutter(i, ADShutterOpen);
				callParamCallbacks(i, i);
				
				/* Update the image */
                if (i == 0) {
                    epicsGuard<epicsMutex> _lock(m_dcLock);
				    status = computeImage(i, m_dcSpectra, m_nDCPts, m_nDCSpec);
                }
                else if (i == 1) {
                    epicsGuard<epicsMutex> _lock(m_tracesLock);
				    status = computeImage(i, m_traces, m_nVoltage, m_NTRACE);
                }

	//            if (status) continue;

				// could sleep to make up to acquireTime
			
				/* Close the shutter */
				setShutter(i, ADShutterClosed);
			
				setIntegerParam(i, ADStatus, ADStatusReadout);
				/* Call the callbacks to update any changes */
				callParamCallbacks(i, i);

				pImage = this->pArrays[i];
				if (pImage == NULL)
				{
					continue;
				}

				/* Get the current parameters */
				getIntegerParam(i, NDArrayCounter, &imageCounter);
				getIntegerParam(i, ADNumImages, &numImages);
				getIntegerParam(i, ADNumImagesCounter, &numImagesCounter);
				getIntegerParam(i, NDArrayCallbacks, &arrayCallbacks);
				++imageCounter;
				++numImagesCounter;
				setIntegerParam(i, NDArrayCounter, imageCounter);
				setIntegerParam(i, ADNumImagesCounter, numImagesCounter);

				/* Put the frame number and time stamp into the buffer */
				pImage->uniqueId = imageCounter;
				pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
				updateTimeStamp(&pImage->epicsTS);

				/* Get any attributes that have been defined for this driver */
				this->getAttributes(pImage->pAttributeList);

				if (arrayCallbacks) {
				  /* Call the NDArray callback */
				  /* Must release the lock here, or we can get into a deadlock, because we can
				   * block on the plugin lock, and the plugin can be calling us */
				  epicsGuardRelease<NucInstDig> _unlock(_lock);
				  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s: calling imageData callback addr %d\n", driverName, functionName, i);
				  doCallbacksGenericPointer(pImage, NDArrayData, i);
				}
				epicsTimeGetCurrent(&endTime);
				elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
				updateTime = epicsTimeDiffInSeconds(&endTime, &(last_update[i]));
				last_update[i] = endTime;
				/* Call the callbacks to update any changes */
				callParamCallbacks(i, i);
				/* sleep for the acquire period minus elapsed time. */
				delay = acquirePeriod - elapsedTime;
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s: delay=%f\n",
						driverName, functionName, delay);
				if (delay >= 0.0) {
					/* We set the status to waiting to indicate we are in the period delay */
					setIntegerParam(i, ADStatus, ADStatusWaiting);
					callParamCallbacks(i, i);
					{
						epicsGuardRelease<NucInstDig> _unlock(_lock);
						epicsThreadSleep(delay);
					}
					setIntegerParam(i, ADStatus, ADStatusIdle);
					callParamCallbacks(i, i);  
				}
			}
			catch(...)
			{
				std::cerr << "Exception in pollerThread4" << std::endl;
			}
        }
		if (all_enable == 0 || all_acquiring == 0)
		{
			epicsThreadSleep(1.0);
		}
		else
		{
			epicsThreadSleep(1.0);
		}
	}
}

/** Computes the new image data */
int NucInstDig::computeImage(int addr, const std::vector<double>& data, int nx, int ny)
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int itemp;
    int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY;
    int xDim=0, yDim=1, colorDim=-1;
    int maxSizeX, maxSizeY;
    int colorMode;
    int ndims=0;
    NDDimension_t dimsOut[3];
    size_t dims[3];
    NDArrayInfo_t arrayInfo;
    NDArray *pImage;
    const char* functionName = "computeImage";

    /* NOTE: The caller of this function must have taken the mutex */

    status |= getIntegerParam(addr, ADBinX,         &binX);
    status |= getIntegerParam(addr, ADBinY,         &binY);
    status |= getIntegerParam(addr, ADMinX,         &minX);
    status |= getIntegerParam(addr, ADMinY,         &minY);
    status |= getIntegerParam(addr, ADSizeX,        &sizeX);
    status |= getIntegerParam(addr, ADSizeY,        &sizeY);
    status |= getIntegerParam(addr, ADReverseX,     &reverseX);
    status |= getIntegerParam(addr, ADReverseY,     &reverseY);
    status |= getIntegerParam(addr, ADMaxSizeX,     &maxSizeX);
    status |= getIntegerParam(addr, ADMaxSizeY,     &maxSizeY);
    status |= getIntegerParam(addr, NDColorMode,    &colorMode);
    status |= getIntegerParam(addr, NDDataType,     &itemp); 
	dataType = (NDDataType_t)itemp;
	if (status)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: error getting parameters\n",
			driverName, functionName);
		return (status);
	}
    if (maxSizeX != nx)
    {
        maxSizeX = nx;
        status |= setIntegerParam(addr, ADMaxSizeX, maxSizeX);
    }
    if (sizeX != nx)
    {
        sizeX = nx;
        status |= setIntegerParam(addr, ADSizeX, sizeX);
    }
    if (maxSizeY != ny)
    {
        maxSizeY = ny;
        status |= setIntegerParam(addr, ADMaxSizeY, maxSizeY);
    }
    if (sizeY != ny)
    {
        sizeY = ny;
        status |= setIntegerParam(addr, ADSizeY, sizeY);
    }

    /* Make sure parameters are consistent, fix them if they are not */
    if (binX < 1) {
        binX = 1;
        status |= setIntegerParam(addr, ADBinX, binX);
    }
    if (binY < 1) {
        binY = 1;
        status |= setIntegerParam(addr, ADBinY, binY);
    }
    if (minX < 0) {
        minX = 0;
        status |= setIntegerParam(addr, ADMinX, minX);
    }
    if (minY < 0) {
        minY = 0;
        status |= setIntegerParam(addr, ADMinY, minY);
    }
    if (minX > maxSizeX-1) {
        minX = maxSizeX-1;
        status |= setIntegerParam(addr, ADMinX, minX);
    }
    if (minY > maxSizeY-1) {
        minY = maxSizeY-1;
        status |= setIntegerParam(addr, ADMinY, minY);
    }
    if (minX+sizeX > maxSizeX) {
        sizeX = maxSizeX-minX;
        status |= setIntegerParam(addr, ADSizeX, sizeX);
    }
    if (minY+sizeY > maxSizeY) {
        sizeY = maxSizeY-minY;
        status |= setIntegerParam(addr, ADSizeY, sizeY);
    }
    
	if (status)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: error setting parameters\n",
			driverName, functionName);
		return (status);
	}

    if (sizeX == 0 || sizeY == 0)
    {
        return asynSuccess;
    }

    switch (colorMode) {
        case NDColorModeMono:
            ndims = 2;
            xDim = 0;
            yDim = 1;
            break;
        case NDColorModeRGB1:
            ndims = 3;
            colorDim = 0;
            xDim     = 1;
            yDim     = 2;
            break;
        case NDColorModeRGB2:
            ndims = 3;
            colorDim = 1;
            xDim     = 0;
            yDim     = 2;
            break;
        case NDColorModeRGB3:
            ndims = 3;
            colorDim = 2;
            xDim     = 0;
            yDim     = 1;
            break;
    }

// we could be more efficient
//    if (resetImage) {
    /* Free the previous raw buffer */
        if (m_pRaw) m_pRaw->release();
        /* Allocate the raw buffer we use to compute images. */
        dims[xDim] = maxSizeX;
        dims[yDim] = maxSizeY;
        if (ndims > 2) dims[colorDim] = 3;
        m_pRaw = this->pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);

        if (!m_pRaw) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error allocating raw buffer\n",
                      driverName, functionName);
            return(status);
        }
//    }

    switch (dataType) {
        case NDInt8:
            status |= computeArray<epicsInt8>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDUInt8:
            status |= computeArray<epicsUInt8>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDInt16:
            status |= computeArray<epicsInt16>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDUInt16:
            status |= computeArray<epicsUInt16>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDInt32:
            status |= computeArray<epicsInt32>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDUInt32:
            status |= computeArray<epicsUInt32>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDInt64:
            status |= computeArray<epicsInt64>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDUInt64:
            status |= computeArray<epicsUInt64>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDFloat32:
            status |= computeArray<epicsFloat32>(addr, data, maxSizeX, maxSizeY);
            break;
        case NDFloat64:
            status |= computeArray<epicsFloat64>(addr, data, maxSizeX, maxSizeY);
            break;
    }

    /* Extract the region of interest with binning.
     * If the entire image is being used (no ROI or binning) that's OK because
     * convertImage detects that case and is very efficient */
    m_pRaw->initDimension(&dimsOut[xDim], sizeX);
    m_pRaw->initDimension(&dimsOut[yDim], sizeY);
    if (ndims > 2) m_pRaw->initDimension(&dimsOut[colorDim], 3);
    dimsOut[xDim].binning = binX;
    dimsOut[xDim].offset  = minX;
    dimsOut[xDim].reverse = reverseX;
    dimsOut[yDim].binning = binY;
    dimsOut[yDim].offset  = minY;
    dimsOut[yDim].reverse = reverseY;

    /* We save the most recent image buffer so it can be used in the read() function.
     * Now release it before getting a new version. */	 
    if (this->pArrays[addr]) this->pArrays[addr]->release();
    status = this->pNDArrayPool->convert(m_pRaw,
                                         &this->pArrays[addr],
                                         dataType,
                                         dimsOut);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error allocating buffer in convert()\n",
                    driverName, functionName);
        return(status);
    }
    pImage = this->pArrays[addr];
    pImage->getInfo(&arrayInfo);
    status = asynSuccess;
    status |= setIntegerParam(addr, NDArraySize,  (int)arrayInfo.totalBytes);
    status |= setIntegerParam(addr, NDArraySizeX, (int)pImage->dims[xDim].size);
    status |= setIntegerParam(addr, NDArraySizeY, (int)pImage->dims[yDim].size);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error setting parameters\n",
                    driverName, functionName);
    return(status);
}



// supplied array of x,y,t
template <typename epicsType> 
int NucInstDig::computeArray(int addr, const std::vector<double>& data, int sizeX, int sizeY)
{
    epicsType *pMono=NULL, *pRed=NULL, *pGreen=NULL, *pBlue=NULL;
    int columnStep=0, rowStep=0, colorMode;
    int status = asynSuccess;
    double exposureTime, gain;
    int i, j, k;

    status = getDoubleParam (ADGain,        &gain);
    status = getIntegerParam(NDColorMode,   &colorMode);
    status = getDoubleParam (ADAcquireTime, &exposureTime);

    switch (colorMode) {
        case NDColorModeMono:
            pMono = (epicsType *)m_pRaw->pData;
            break;
        case NDColorModeRGB1:
            columnStep = 3;
            rowStep = 0;
            pRed   = (epicsType *)m_pRaw->pData;
            pGreen = (epicsType *)m_pRaw->pData+1;
            pBlue  = (epicsType *)m_pRaw->pData+2;
            break;
        case NDColorModeRGB2:
            columnStep = 1;
            rowStep = 2 * sizeX;
            pRed   = (epicsType *)m_pRaw->pData;
            pGreen = (epicsType *)m_pRaw->pData + sizeX;
            pBlue  = (epicsType *)m_pRaw->pData + 2*sizeX;
            break;
        case NDColorModeRGB3:
            columnStep = 1;
            rowStep = 0;
            pRed   = (epicsType *)m_pRaw->pData;
            pGreen = (epicsType *)m_pRaw->pData + sizeX*sizeY;
            pBlue  = (epicsType *)m_pRaw->pData + 2*sizeX*sizeY;
            break;
    }
    m_pRaw->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
	memset(m_pRaw->pData, 0, m_pRaw->dataSize);
    k = 0;
	for (i=0; i<sizeY; i++) {
		switch (colorMode) {
			case NDColorModeMono:
				for (j=0; j<sizeX; j++) {
					pMono[k] = static_cast<epicsType>(gain * data[k]);
					++k;
				}
				break;
			case NDColorModeRGB1:
			case NDColorModeRGB2:
			case NDColorModeRGB3:
				for (j=0; j<sizeX; j++) {
					pRed[k] = pGreen[k] = pBlue[k] = static_cast<epicsType>(gain * data[k]);
					pRed   += columnStep;
					pGreen += columnStep;
					pBlue  += columnStep;
					++k;
				}
				pRed   += rowStep;
				pGreen += rowStep;
				pBlue  += rowStep;
				break;
		}
	}
    return(status);
}

    
void NucInstDig::updateDCSpectra()
{
    while(true)
    {
        int read_spectra = 0;
        epicsThreadSleep(1.0);
        lock();
        getIntegerParam(P_readDCSpectra, &read_spectra);
        unlock();
        if (read_spectra == 0) {
            continue;
        }
        try {
            {
                epicsGuard<epicsMutex> _lock(m_dcLock);
                readData2d("get_darkcount_spectra", "", m_dcSpectra, m_nDCSpec, m_nDCPts);
            }
            for(size_t j=0; j<4; ++j) {
                int idx = m_DCSpecIdx[j];
                if (idx >= 0 && idx < m_nDCSpec) {
                    m_DCSpecX[j].resize(m_nDCPts);
                    m_DCSpecY[j].resize(m_nDCPts);
                    for(int k=0; k<m_nDCPts; ++k) {
                        m_DCSpecX[j][k] = k;
                        m_DCSpecY[j][k] = m_dcSpectra[idx * m_nDCPts + k];
                    }
                    lock();
                    doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_DCSpecX[j].data()), m_DCSpecX[j].size(), P_DCSpecX[j], 0);
                    doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_DCSpecY[j].data()), m_DCSpecY[j].size(), P_DCSpecY[j], 0);
                    unlock();
                }
            }
        }
        catch(const std::exception& ex)
        {
            std::cerr << "update dc spectra " << ex.what() << std::endl;
            epicsThreadSleep(3.0);            
        }
        catch(...)
        {
            std::cerr << "update dc spectra exception"  << std::endl;
            epicsThreadSleep(3.0);            
        }
    }
}
    
void NucInstDig::updateEvents()
{
    while(true)
    {
        int read_events = 0;
        epicsThreadSleep(1.0);
        lock();
        getIntegerParam(P_readEvents, &read_events);
        unlock();
        if (read_events == 0) {
            continue;
        }
        try {
        zmq::message_t reply{};
        m_zmq_events_socket.recv(reply, zmq::recv_flags::none);
        auto msg = GetDigitizerEventListMessage(reply.data());
        auto channels = msg->channel();
        auto times = msg->time();
        auto voltages = msg->voltage();
        size_t nevents = channels->size();
        std::cerr << "nevents " << nevents << std::endl;
        if (nevents > 5) {
            nevents = 5;
        }
        for(size_t i=0; i<nevents; ++i) {
            std::cerr << channels->Get(i) << " " << times->Get(i) << " " << voltages->Get(i) << std::endl;
        }
        }
        catch(const std::exception& ex)
        {
            std::cerr << "update events " << ex.what() << std::endl;
            epicsThreadSleep(3.0);            
        }
        catch(...)
        {
            std::cerr << "update events exception"  << std::endl;
            epicsThreadSleep(3.0);            
        }
    }
}

void NucInstDig::executeCmd(const std::string& name, const std::string& args)
{
    rapidjson::Document doc_recv;
    execute("execute_cmd", name, args, "", doc_recv);    
}

void NucInstDig::readData2d(const std::string& name, const std::string& args, std::vector<double>& dataOut, size_t& nspec, size_t& npts)
{
    rapidjson::Document doc_recv;
    dataOut.resize(0);
    nspec = npts = 0;
    execute("execute_read_command", name, args, "", doc_recv);
    const rapidjson::Value& data = doc_recv["data"];
    if (data.IsArray())
    {
        nspec = data.Size();
        const rapidjson::Value& spec0 = data[0];
        npts = spec0.Size();
        dataOut.resize(nspec * npts);
        for (rapidjson::SizeType i = 0; i < nspec; ++i)
        {
            const rapidjson::Value& spec = data[i];
            for (rapidjson::SizeType j = 0; j < spec.Size(); ++j)
            {
                dataOut[i * npts + j] = spec[j].GetDouble();
            }
        }
    }
}

void NucInstDig::getParameter(const std::string& name, rapidjson::Value& value, int idx)
{
    char idxStr[16];
    rapidjson::Document doc_recv;
    sprintf(idxStr, "%d", idx);
    execute("get_parameter", name, idxStr, "", doc_recv);
    value = doc_recv["value"];
}

void NucInstDig::setParameter(const std::string& name, const std::string& value, int idx)
{
    char idxStr[16];
    rapidjson::Document doc_recv;
    sprintf(idxStr, "%d", idx);
    execute("set_parameter", name, value, idxStr, doc_recv);    
}

void NucInstDig::setParameter(const std::string& name, double value, int idx)
{
    char idxStr[16], valueStr[16];
    rapidjson::Document doc_recv;
    sprintf(idxStr, "%d", idx);
    sprintf(valueStr, "%f", value);
    execute("set_parameter", name, valueStr, idxStr, doc_recv);
}

void NucInstDig::setParameter(const std::string& name, int value, int idx)
{
    char idxStr[16], valueStr[16];
    rapidjson::Document doc_recv;
    sprintf(idxStr, "%d", idx);
    sprintf(valueStr, "%d", value);
    execute("set_parameter", name, valueStr, idxStr, doc_recv);
}

void NucInstDig::execute(const std::string& type, const std::string& name, const std::string& arg1, const std::string& arg2, rapidjson::Document& doc_recv)
{
    epicsGuard<epicsMutex> _lock(m_executeLock);
    rapidjson::Document doc_send;
    rapidjson::Value arg1v, arg2v;
    rapidjson::Value typev(type.c_str(), doc_send.GetAllocator());
    rapidjson::Value namev(name.c_str(), doc_send.GetAllocator());
    doc_send.SetObject();
    doc_send.AddMember("command", typev, doc_send.GetAllocator());
    doc_send.AddMember("name", namev, doc_send.GetAllocator());
    if (type == "execute_cmd")
    {
        arg1v.SetString(arg1.c_str(), doc_send.GetAllocator());
        doc_send.AddMember("args", arg1v, doc_send.GetAllocator());
    }
    else if (type == "get_parameter")
    {
        arg1v.SetInt(atol(arg1.c_str()));
        doc_send.AddMember("idx", arg1v, doc_send.GetAllocator());
    }
    else if (type == "set_parameter")
    {
        arg1v.SetString(arg1.c_str(), doc_send.GetAllocator());
        arg2v.SetInt(atol(arg2.c_str()));
        doc_send.AddMember("value", arg1v, doc_send.GetAllocator());
        doc_send.AddMember("idx", arg2v, doc_send.GetAllocator());
    }
    else if (type == "execute_read_command")
    {
        arg1v.SetString(arg1.c_str(), doc_send.GetAllocator());
        doc_send.AddMember("args", arg1v, doc_send.GetAllocator());
    }
    else
    {
        throw std::runtime_error("unknown command");
    }

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    doc_send.Accept(writer);
    std::string sendstr = sb.GetString();
//    std::cout << "Sending " << sendstr << std::endl;
    m_zmq_cmd_socket.send(zmq::buffer(sendstr), zmq::send_flags::none);
    zmq::message_t reply{};
    m_zmq_cmd_socket.recv(reply, zmq::recv_flags::none);
//    std::cout << "Received " << reply.to_string() << std::endl;
    doc_recv.Parse(reply.to_string().c_str());
    if (doc_recv["response"] != "ok")
    {
        std::ostrstream oss;
        oss << "Sent " << sendstr << " Error: code=" << doc_recv["error_code"].GetInt() << " message=" << doc_recv["message"].GetString();
        std::cerr << oss.str() << std::endl;   // print as long message may get truncated on asyn print
        throw std::runtime_error(oss.str());
    }
}

void NucInstDig::createNParams(const char* name, asynParamType type, int* param, int n)
{
    char buffer[256]; 
    for(int i=0; i<n; ++i)
    {
        sprintf(buffer, name, i+1); 
        createParam(buffer, type, &(param[i]));
    }
}

/// Constructor for the isisdaeDriver class.
/// Calls constructor for the asynPortDriver base class.
/// \param[in] dcomint DCOM interface pointer created by lvDCOMConfigure()
/// \param[in] portName @copydoc initArg0
NucInstDig::NucInstDig(const char *portName, const char *targetAddress, int dig_idx)
   : ADDriver(portName, 2, 100,
					0, // maxBuffers
					0, // maxMemory
                    asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver can block and is multi-device from the point of view of area detector live views */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0),	/* Default stack size*/
                     m_zmq_events_ctx{1}, m_zmq_events_socket(m_zmq_events_ctx, zmq::socket_type::pull),
                     m_zmq_cmd_ctx{1}, m_zmq_cmd_socket(m_zmq_cmd_ctx, zmq::socket_type::req),
                     m_zmq_stream_ctx{1}, m_zmq_stream_socket(m_zmq_stream_ctx, zmq::socket_type::pull),
                     m_dig_idx(dig_idx), m_pTraces(NULL), m_pDCSpectra(NULL), m_pRaw(NULL),
                     m_nDCSpec(0), m_nDCPts(0), m_nVoltage(0), m_NTRACE(8)
{					
    const char *functionName = "NucInstDig";
    int events_to_monitor =  ZMQ_EVENT_CONNECTED|ZMQ_EVENT_DISCONNECTED|ZMQ_EVENT_CLOSED|ZMQ_EVENT_BIND_FAILED|ZMQ_EVENT_CONNECT_DELAYED|ZMQ_EVENT_CONNECT_RETRIED;
    m_zmq_events_mon.init(m_zmq_events_socket, "inproc://NucInstDigConMon", events_to_monitor);
    m_zmq_cmd_mon.init(m_zmq_cmd_socket, "inproc://NucInstDigConMon", events_to_monitor);
    m_zmq_stream_mon.init(m_zmq_stream_socket, "inproc://NucInstDigConMon", events_to_monitor);
    if (epicsThreadCreate("zmqMonitorPoller",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)zmqMonitorPollerC, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    m_zmq_events_socket.connect(std::string("tcp://") + targetAddress + ":5555");
    m_zmq_cmd_socket.connect(std::string("tcp://") + targetAddress + ":5557");
    m_zmq_stream_socket.connect(std::string("tcp://") + targetAddress + ":5556");

    createParam(P_setupString, asynParamInt32, &P_setup);  // must be first as FIRST_NUCINSTDIG_PARAM
    createParam(P_startAcquisitionString, asynParamInt32, &P_startAcquisition);
    createParam(P_stopAcquisitionString, asynParamInt32, &P_stopAcquisition);
    createParam(P_configDGTZString, asynParamInt32, &P_configDGTZ);    
    createParam(P_configBASEString, asynParamInt32, &P_configBASE);    
    createParam(P_configHVString, asynParamInt32, &P_configHV);    
    createParam(P_configSTAVESString, asynParamInt32, &P_configSTAVES);    
    createNParams(P_DCSpecXString, asynParamFloat64Array, P_DCSpecX, 4);
    createNParams(P_DCSpecYString, asynParamFloat64Array, P_DCSpecY, 4);
    createNParams(P_DCSpecIdxString, asynParamInt32, P_DCSpecIdx, 4);
    createNParams(P_traceXString, asynParamFloat64Array, P_traceX, 4);
    createNParams(P_traceYString, asynParamFloat64Array, P_traceY, 4);
    createNParams(P_traceIdxString, asynParamInt32, P_traceIdx, 4);
    createParam(P_readDCSpectraString, asynParamInt32, &P_readDCSpectra);
    createParam(P_readEventsString, asynParamInt32, &P_readEvents);
    createParam(P_resetDCSpectraString, asynParamInt32, &P_resetDCSpectra);
    
	int maxSizes[2][2] = { {16, 20000}, { 16, 4096 } };
    NDDataType_t dataType = NDFloat64; // data type for each frame
    int status = 0;
    for(int i=0; i<2; ++i)
    {
        int maxSizeX = maxSizes[i][0];
        int maxSizeY = maxSizes[i][1];
		status =  setStringParam (i, ADManufacturer, "NucInstDig");
		status |= setStringParam (i, ADModel, "NucInstDig");
		status |= setIntegerParam(i, ADMaxSizeX, 1);
		status |= setIntegerParam(i, ADMaxSizeY, 1);
		status |= setIntegerParam(i, ADMinX, 0);
		status |= setIntegerParam(i, ADMinY, 0);
		status |= setIntegerParam(i, ADBinX, 1);
		status |= setIntegerParam(i, ADBinY, 1);
		status |= setIntegerParam(i, ADReverseX, 0);
		status |= setIntegerParam(i, ADReverseY, 0);
		status |= setIntegerParam(i, ADSizeX, 1);
		status |= setIntegerParam(i, ADSizeY, 1);
		status |= setIntegerParam(i, NDArraySizeX, 1);
		status |= setIntegerParam(i, NDArraySizeY, 1);
		status |= setIntegerParam(i, NDArraySize, 1);
		status |= setIntegerParam(i, NDDataType, dataType);
		status |= setIntegerParam(i, ADImageMode, ADImageContinuous);
		status |= setIntegerParam(i, ADStatus, ADStatusIdle);
		status |= setIntegerParam(i, ADAcquire, 0);
		status |= setDoubleParam (i, ADAcquireTime, .001);
		status |= setDoubleParam (i, ADAcquirePeriod, .005);
		status |= setIntegerParam(i, ADNumImages, 100);
    }

    if (status) {
        printf("%s: unable to set DAE parameters\n", functionName);
        return;
    }    

    // Create the thread for background tasks (not used at present, could be used for I/O intr scanning) 
    if (epicsThreadCreate("isisdaePoller1",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC1, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("isisdaePoller2",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC2, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("isisdaePoller3",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC3, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("isisdaePoller4",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC4, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("isisdaePoller5",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC5, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
}

void NucInstDig::pollerThreadC1(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread1();
	}
}

void NucInstDig::pollerThreadC2(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread2();
	}
}

void NucInstDig::pollerThreadC3(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread3();
	}
}

void NucInstDig::pollerThreadC4(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread4();
	}
}

void NucInstDig::pollerThreadC5(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread5();
	}
}

void NucInstDig::zmqMonitorPollerC(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->zmqMonitorPoller();
	}
}

void NucInstDig::pollerThread1()
{
    static const char* functionName = "isisdaePoller1";
    unsigned long counter = 0;
    while(true)
    {
        try
        {
            epicsGuard<NucInstDig> _lock(*this);
            for(const auto& kv : m_param_data)
            {
                const ParamData* p = kv.second;
                try
                {
                    rapidjson::Value value;
                    getParameter(p->name, value, p->chan);
                    if (p->type == asynParamInt32)
                    {
                        setIntegerParam(kv.first, (value.IsInt() ? value.GetInt() : atoi(value.GetString())));
                    }
                    else if (p->type == asynParamFloat64)
                    {
                        setDoubleParam(kv.first, (value.IsNumber() ? value.GetDouble() : atof(value.GetString())));
                    }
                    else if (p->type == asynParamOctet)
                    {
                        setStringParam(kv.first, value.GetString());
                    }
                    else
                    {
                        std::cerr << "pollerThread1: invalid type " << p->type << " for " << p->name << std::endl;
                    }
                }
                catch(const std::exception& ex)
                {
                    std::cerr << "pollerThread1: exception " << ex.what() << " for parameter " << p->name << " channel " << p->chan << std::endl;
                    epicsThreadSleep(1.0);
                }
                catch(...)
                {
                    std::cerr << "pollerThread1: exception for parameter " << p->name << " channel " << p->chan << std::endl;
                    epicsThreadSleep(1.0);
                }
            }
            callParamCallbacks();
        }
        catch(const std::exception& ex)
        {
            std::cerr << "pollerThread1: exception " << ex.what() << std::endl;
            epicsThreadSleep(3.0);
        }
        catch(...)
        {
            std::cerr << "pollerThread1: exception " << std::endl;
            epicsThreadSleep(3.0);
        }
        epicsThreadSleep(3.0);
    }
}

void NucInstDig::pollerThread2()
{
    static const char* functionName = "isisdaePoller1";
    updateTraces();
}

void NucInstDig::pollerThread3()
{
    static const char* functionName = "isisdaePoller1";
    updateEvents();
}

void NucInstDig::pollerThread4()
{
    static const char* functionName = "isisdaePoller1";
    updateDCSpectra();
}

void NucInstDig::pollerThread5()
{
    static const char* functionName = "isisdaePoller1";
    updateAD();
}

void NucInstDig::zmqMonitorPoller()
{
    static const char* functionName = "zmqMonitorPoller";
    while(true)
    {
        try
        {
            m_zmq_cmd_mon.check_event(100);
            m_zmq_stream_mon.check_event(100);
            m_zmq_events_mon.check_event(100);
        }
        catch(const std::exception& ex)
        {
            std::cerr << "zmqMonitorPoller: exception " << ex.what() << std::endl;
            epicsThreadSleep(3.0);
        }
        catch(...)
        {
            std::cerr << "zmqMonitorPoller: exception " << std::endl;
            epicsThreadSleep(3.0);
        }
        epicsThreadSleep(0.5);
    }
}

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void NucInstDig::report(FILE *fp, int details)
{
    fprintf(fp, "ISIS DAE driver %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

asynStatus NucInstDig::drvUserCreate(asynUser *pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize)
{
   // PARAM,name,type,chan,log_freq
   if (strncmp(drvInfo, "PARAM,", 6) == 0)
     {
         std::string param_name;
         std::vector<std::string> split_vec;
         int param_index;
         asynParamType param_type;
         boost::split(split_vec, drvInfo, boost::is_any_of(","), boost::token_compress_on);
         if (split_vec.size() == 5)
         {
             param_name = split_vec[0] + "_" + split_vec[1] + "_" + split_vec[3];
             if (findParam(param_name.c_str(), &param_index) == asynSuccess)
             {
                 pasynUser->reason = param_index;
                 return asynSuccess;
             }
             if (split_vec[2] == "I")
             {
                 param_type = asynParamInt32;
             }
             else if (split_vec[2] == "D")
             {
                 param_type = asynParamFloat64;
             }
             else if (split_vec[2] == "S")
             {
                 param_type = asynParamOctet;
             }
             else
             {
                 std::cerr << "ERROR: Param " << param_name << " invalid type " << split_vec[2] << std::endl;
                 return asynError;
             }
             if (createParam(param_name.c_str(), param_type, &param_index) == asynSuccess)
             {
                 m_param_data[param_index] = new ParamData(split_vec[1], param_type, atoi(split_vec[3].c_str()), atoi(split_vec[4].c_str()));
                 pasynUser->reason = param_index;
                 return asynSuccess;
             }
         }
         std::cerr << "ERROR: Incorrect field count in drvInfo " << drvInfo << std::endl;
         return asynError;
     }
  else
     {
     return ADDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
     }
}

// used to free pasynUser->userData is needed
asynStatus NucInstDig::drvUserDestroy(asynUser *pasynUser)
{
   return ADDriver::drvUserDestroy(pasynUser);
}

int nucInstDigConfigure(const char *portName, const char *targetAddress, int dig_idx)
{
	try
	{
		NucInstDig* iface = new NucInstDig(portName, targetAddress, dig_idx);
		return(asynSuccess);
	}
	catch(const std::exception& ex)
	{
		errlogSevPrintf(errlogMajor, "nucInstDigConfigure failed: %s\n", ex.what());
		return(asynError);
	}
}

// EPICS iocsh shell commands 

// isisdaeConfigure
extern "C" {
    
static const iocshArg initArg0 = { "portName", iocshArgString};			///< The name of the asyn driver port we will
static const iocshArg initArg1 = { "targetAddress", iocshArgString};			///< The name of the asyn driver port we will create
static const iocshArg initArg2 = { "digitiserIndex", iocshArgInt};			///< The name of the asyn driver port we will create

static const iocshArg * const initArgs[] = { &initArg0, &initArg1, &initArg2 };

static const iocshFuncDef initFuncDef = {"nucInstDigConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    nucInstDigConfigure(args[0].sval, args[1].sval, args[2].ival);
}

static void nucInstDigRegister(void)
{
	iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(nucInstDigRegister);

}

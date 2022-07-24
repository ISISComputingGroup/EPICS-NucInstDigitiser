#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
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

asynStatus NucInstDig::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;
	if (function < FIRST_NUCINSTDIG_PARAM)
	{
        return ADDriver::writeFloat64(pasynUser, value);
	}
    asynStatus stat = asynSuccess;
	if (stat == asynSuccess)
	{
		asynPortDriver::writeFloat64(pasynUser, value); // to update parameter and do callbacks
	}
	else
	{
		callParamCallbacks(); // this flushes P_ErrMsgs
	}
	return stat;
}

asynStatus NucInstDig::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	if (function < FIRST_NUCINSTDIG_PARAM)
	{
        return ADDriver::writeInt32(pasynUser, value);
	}
	asynStatus stat = asynSuccess;
    if (function == P_startAcquisition) {
        execute("execute_cmd", "start_acquisition", "", "");
    }
	if (stat == asynSuccess)
	{
		asynPortDriver::writeInt32(pasynUser, value); // to update parameter and do callbacks
	}
	else
	{
		callParamCallbacks(); // this flushes P_ErrMsgs
	}
	return stat;
}

asynStatus NucInstDig::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
	int function = pasynUser->reason;
	if (function < FIRST_NUCINSTDIG_PARAM)
	{
		return ADDriver::readFloat64Array(pasynUser, value, nElements, nIn);
	}
    asynStatus stat = asynSuccess;
	callParamCallbacks(); // this flushes P_ErrMsgs
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
	callParamCallbacks(); // this flushes P_ErrMsgs
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
		callParamCallbacks(); // this flushes P_ErrMsgs
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
	    if (function < FIRST_NUCINSTDIG_PARAM)
	    {
	        return ADDriver::readInt32(pasynUser, value);
	    }
	    return asynPortDriver::readInt32(pasynUser, value);
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName, functionName, function, paramName, *value);
		return asynSuccess;
	}
	catch(const std::exception& ex)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: function=%d, name=%s, error=%s", 
                  driverName, functionName, function, paramName, ex.what());
		callParamCallbacks(); // this flushes P_ErrMsgs
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
		callParamCallbacks(); // this flushes P_ErrMsgs
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
		    status = asynPortDriver::writeOctet(pasynUser, value_s.c_str(), value_s.size(), nActual); // update parameters and do callbacks
		}
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
		callParamCallbacks(); // this flushes P_ErrMsgs
		*nActual = 0;
		return asynError;
	}
	catch(...)
	{
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%s, error=unknow exception", 
                  driverName, functionName, status, function, paramName, value_s.c_str());
		callParamCallbacks(); // this flushes P_ErrMsgs
		*nActual = 0;
		return asynError;
	}
}


void NucInstDig::updateTraces()
{
    while(true)
    {
        zmq::message_t reply{};
        m_zmq_stream_socket.recv(reply, zmq::recv_flags::none);
        auto msg = GetDigitizerAnalogTraceMessage(reply.data());
        auto channels = msg->channels();    
        for(int i=0; i<channels->size(); ++i) {
            int chan = channels->Get(i)->channel();
            if (chan == 0) {
            int nvoltage = channels->Get(i)->voltage()->size();
            m_traceX.resize(nvoltage);
            m_traceY.resize(nvoltage);
            for(int j=0; j<nvoltage; ++j) {
                m_traceX[j] = j;
                m_traceY[j] = channels->Get(i)->voltage()->Get(j);
            }
            }
        }
        doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(&(m_traceX[0])), m_traceX.size(), P_traceX, 0);
        doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(&(m_traceY[0])), m_traceY.size(), P_traceY, 0);
//    std::cerr << "timestamp " << msg->status()->timestamp() << std::endl;
    }
}
    
void NucInstDig::execute(const std::string& type, const std::string& name, const std::string& arg1, const std::string& arg2)
{    
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
        arg1v.SetInt(atol(arg1.c_str()));
        arg2v.SetString(arg2.c_str(), doc_send.GetAllocator());
        doc_send.AddMember("idx", arg1v, doc_send.GetAllocator());
        doc_send.AddMember("value", arg2v, doc_send.GetAllocator());
    }
    else if (type == "read_data")
    {
        arg1v.SetString(arg1.c_str(), doc_send.GetAllocator());
        doc_send.AddMember("args", arg1v, doc_send.GetAllocator());
    }
    else
    {
        std::cerr << "unknown command" << std::endl;
    }

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    doc_send.Accept(writer);
    std::string sendstr = sb.GetString();
    std::cout << "Sending " << sendstr << std::endl;
    m_zmq_cmd_socket.send(zmq::buffer(sendstr), zmq::send_flags::none);
    zmq::message_t reply{};
    m_zmq_cmd_socket.recv(reply, zmq::recv_flags::none);
    std::cout << "Received " << reply.to_string() << std::endl;
    rapidjson::Document doc_recv;
    doc_recv.Parse(reply.to_string().c_str());
}



/// Constructor for the isisdaeDriver class.
/// Calls constructor for the asynPortDriver base class.
/// \param[in] dcomint DCOM interface pointer created by lvDCOMConfigure()
/// \param[in] portName @copydoc initArg0
NucInstDig::NucInstDig(const char *portName, const char * targetAddress)
   : ADDriver(portName, 1, 100,
					0, // maxBuffers
					0, // maxMemory
                    asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver can block and is multi-device from the point of view of area detector live views */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0),	/* Default stack size*/
					 m_pRaw(NULL),
                     m_zmq_cmd_ctx{1}, m_zmq_cmd_socket(m_zmq_cmd_ctx, zmq::socket_type::req),
                     m_zmq_stream_ctx{1}, m_zmq_stream_socket(m_zmq_stream_ctx, zmq::socket_type::pull)
{					
    const char *functionName = "NucInstDig";
    
    m_zmq_cmd_socket.connect(std::string("tcp://") + targetAddress + ":5557");
    m_zmq_stream_socket.connect(std::string("tcp://") + targetAddress + ":5556");
    createParam(P_startAcquisitionString, asynParamInt32, &P_startAcquisition);
    createParam(P_specNumString, asynParamInt32, &P_specNum);
    createParam(P_traceNumString, asynParamInt32, &P_traceNum);
    createParam(P_specXString, asynParamFloat64Array, &P_specX);
    createParam(P_specYString, asynParamFloat64Array, &P_specY);
    createParam(P_traceXString, asynParamFloat64Array, &P_traceX);
    createParam(P_traceYString, asynParamFloat64Array, &P_traceY);

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

void NucInstDig::pollerThread1()
{
    static const char* functionName = "isisdaePoller1";
    unsigned long counter = 0;
}

void NucInstDig::pollerThread2()
{
    static const char* functionName = "isisdaePoller1";
    updateTraces();
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

int nucInstDigConfigure(const char *portName, const char *targetAddress)
{
	try
	{
		NucInstDig* iface = new NucInstDig(portName, targetAddress);
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

static const iocshArg * const initArgs[] = { &initArg0, &initArg1 };

static const iocshFuncDef initFuncDef = {"nucInstDigConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    nucInstDigConfigure(args[0].sval, args[1].sval);
}

static void nucInstDigRegister(void)
{
	iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(nucInstDigRegister);

}

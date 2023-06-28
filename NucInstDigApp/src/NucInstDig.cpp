#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <strstream>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <iomanip>
#include <sys/timeb.h>
#include <numeric>
#include <atomic>
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
    char setupFile[512];
    rapidjson::Value value;  
    setIntegerParam(P_setupDone, 0);
    try {
        executeCmd("stop_acquisition");
    }
    catch(...) {
          ;
    }
      getStringParam(P_setupFile, sizeof(setupFile), setupFile);
      std::cerr << "SETUP: using file \"" << setupFile << "\"" << std::endl;
      std::fstream fs(setupFile, std::ios::in);
      if (!fs.good()) {
          throw std::runtime_error(std::string("File \"") + setupFile + "\" does not exist");
      }
      std::string line;
      std::vector<std::string> split_line;
      while(std::getline(fs, line)) {
          // remove comments
          size_t pos = line.find("#");
          if (pos != std::string::npos) {
              line.erase(line.begin() + pos, line.end());
          }
          boost::trim(line);
          if (line.size() == 0) {
              continue;
          }
          // split line into tokens
          boost::split(split_line, line, boost::is_any_of("\t "));
          if (split_line.size() < 1) {
              std::cerr << "SETUP: <ignored> \"" << line << "\"" << std::endl;
              continue;
          }
          // line is either a single command or   name,value,...   for setParameter
          std::cerr << "SETUP: line \"" << boost::algorithm::join(split_line, " ") << "\"" << std::endl;
          if (split_line.size() == 1) { // command
              std::cerr << "SETUP: executeCmd(\"" << split_line[0] << "\")" << std::endl;
              executeCmd(split_line[0]);
              continue;
          }
          int idx_start = 0;
          int idx_end = 1;
          if (split_line.size() == 3) { // name, value, index
              idx_start = atoi(split_line[2].c_str());
              idx_end = idx_start + 1;
          }
          else if (split_line.size() == 4) { // name, value, start, end
              idx_start = atoi(split_line[2].c_str());
              idx_end = atoi(split_line[3].c_str());
          }
          for(int i=idx_start; i<idx_end; ++i) {
              std::cerr << "SETUP: setParameter(\"" << split_line[0] << "\", \"" << split_line[1] << "\", index=" << i << ")" << std::endl;
              setParameter(split_line[0], split_line[1], i);
          }
      }
      setIntegerParam(P_setupDone, 1);
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
        setStringParam(P_error, "");
        if (stat == asynSuccess)
        {
            stat = asynPortDriver::writeFloat64(pasynUser, value); // to update parameter and do callbacks
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%f\n", 
              driverName, functionName, function, paramName, value);
        return stat;
    }
    catch(const std::exception& ex)
    {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%f, error=%s", 
                  driverName, functionName, stat, function, paramName, value, ex.what());
        setStringParam(P_error, ex.what());
        callParamCallbacks();
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
        else if (function == P_resetTOFSpectra) {
            executeCmd("reset_tof_spectra", "");
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
        else if (function >= P_TOFSpecIdx[0] && function <= P_TOFSpecIdx[3]) {
            int idx = function - P_TOFSpecIdx[0];
            m_TOFSpecIdx[idx] = value;
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
        setStringParam(P_error, "");
        if (stat == asynSuccess)
        {
            stat = asynPortDriver::writeInt32(pasynUser, value); // to update parameter and do callbacks
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName, functionName, function, paramName, value);
        return stat;
    }
    catch(const std::exception& ex)
    {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%d, error=%s", 
                  driverName, functionName, stat, function, paramName, value, ex.what());
        setStringParam(P_error, ex.what());
        callParamCallbacks();
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
        else if (function == P_setupFile)
        {
            ; // fall through to just update asyn parameter
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
        setStringParam(P_error, "");
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
        setStringParam(P_error, ex.what());
        callParamCallbacks();
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
        zmq::recv_result_t nbytes = m_zmq_stream.recv(reply, zmq::recv_flags::none);
        if (!nbytes || *nbytes == 0)
        {
            epicsThreadSleep(1.0);
            continue;
        }
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
            epicsGuard<NucInstDig> _lock(*this);
            for(size_t j=0; j<4; ++j) {
                doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_traceX[j].data()), m_traceX[j].size(), P_traceX[j], 0);
                doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_traceY[j].data()), m_traceY[j].size(), P_traceY[j], 0);
            }
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
    static const char* functionName = "NucInstDigPoller4";
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
                } else if (i == 1) {
                    enable = 1; // traces always enabled
                } else if (i == 2) {
                    getIntegerParam(P_readTOFSpectra, &enable);
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
                else if (i == 2) {
                    epicsGuard<epicsMutex> _lock(m_TOFSpectraLock);
				    status = computeImage(i, m_TOFSpectra, m_nTOFPts, m_nTOFSpec);
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
                    epicsGuard<NucInstDig> _lock2(*this);
                    doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_DCSpecX[j].data()), m_DCSpecX[j].size(), P_DCSpecX[j], 0);
                    doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_DCSpecY[j].data()), m_DCSpecY[j].size(), P_DCSpecY[j], 0);
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
    
void NucInstDig::updateTOFSpectra()
{
    while(true)
    {
        int read_spectra = 0;
        epicsThreadSleep(1.0);
        lock();
        getIntegerParam(P_readTOFSpectra, &read_spectra);
        unlock();
        if (read_spectra == 0) {
            continue;
        }
        try {
            {
                epicsGuard<epicsMutex> _lock(m_TOFSpectraLock);
                readData2d("get_tof_spectra", "", m_TOFSpectra, m_nTOFSpec, m_nTOFPts);
            }
            for(size_t j=0; j<4; ++j) {
                int idx = m_TOFSpecIdx[j];
                if (idx >= 0 && idx < m_nTOFSpec) {
                    m_TOFSpecX[j].resize(m_nTOFPts);
                    m_TOFSpecY[j].resize(m_nTOFPts);
                    for(int k=0; k<m_nTOFPts; ++k) {
                        m_TOFSpecX[j][k] = k;
                        m_TOFSpecY[j][k] = m_TOFSpectra[idx * m_nTOFPts + k];
                    }
                    epicsGuard<NucInstDig> _lock2(*this);
                    doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_TOFSpecX[j].data()), m_TOFSpecX[j].size(), P_TOFSpecX[j], 0);
                    doCallbacksFloat64Array(reinterpret_cast<epicsFloat64*>(m_TOFSpecY[j].data()), m_TOFSpecY[j].size(), P_TOFSpecY[j], 0);
                }
            }
        }
        catch(const std::exception& ex)
        {
            std::cerr << "update TOF spectra " << ex.what() << std::endl;
            epicsThreadSleep(3.0);            
        }
        catch(...)
        {
            std::cerr << "update TOF spectra exception"  << std::endl;
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
        zmq::recv_result_t nbytes = m_zmq_events.recv(reply, zmq::recv_flags::none);
        if (!nbytes || *nbytes == 0)
        {
            epicsThreadSleep(1.0);
            continue;
        }
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
        throw std::runtime_error(std::string("unknown command type: ") + type);
    }

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    doc_send.Accept(writer);
    std::string sendstr = sb.GetString();
//    std::cout << "Sending " << sendstr << std::endl;
    zmq::send_result_t nbytes_send = m_zmq_cmd.send(zmq::buffer(sendstr), zmq::send_flags::none);
    if (!nbytes_send || *nbytes_send == 0)
    {
        m_zmq_cmd.init();
        throw std::runtime_error(std::string("unable to send: ") + type + " " + arg1 + " " + arg2);
    }
    zmq::message_t reply{};
    zmq::recv_result_t nbytes_recv = m_zmq_cmd.recv(reply, zmq::recv_flags::none);
    if (!nbytes_recv || *nbytes_recv == 0)
    {
        m_zmq_cmd.init();
        throw std::runtime_error(std::string("unable to receive: ") + type + " " + arg1 + " " + arg2);
    }
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

/// Constructor for the NucInstDigDriver class.
/// Calls constructor for the asynPortDriver base class.
/// \param[in] dcomint DCOM interface pointer created by lvDCOMConfigure()
/// \param[in] portName @copydoc initArg0
NucInstDig::NucInstDig(const char *portName, const char *targetAddress, int dig_idx)
   : ADDriver(portName, 3, 100,
					0, // maxBuffers
					0, // maxMemory
                    asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver can block and is multi-device from the point of view of area detector live views */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0),	/* Default stack size*/
                     m_zmq_events(zmq::socket_type::pull, std::string("tcp://") + targetAddress + ":5555"),
                     m_zmq_cmd(zmq::socket_type::req, std::string("tcp://") + targetAddress + ":5557"),
                     m_zmq_stream(zmq::socket_type::pull, std::string("tcp://") + targetAddress + ":5556"),
                     m_dig_idx(dig_idx), m_pTraces(NULL), m_pDCSpectra(NULL), m_pTOFSpectra(NULL), m_pRaw(NULL),
                     m_nDCSpec(0), m_nDCPts(0), m_nVoltage(0), m_NTRACE(8), m_nTOFSpec(0), m_nTOFPts(0), m_connected(false)
{					
    const char *functionName = "NucInstDig";

    createParam(P_setupString, asynParamInt32, &P_setup);  // must be first as FIRST_NUCINSTDIG_PARAM
    createParam(P_setupFileString, asynParamOctet, &P_setupFile);
    createParam(P_errorString, asynParamOctet, &P_error);
    createParam(P_setupDoneString, asynParamInt32, &P_setupDone);
    createParam(P_ZMQConnectedString, asynParamInt32, &P_ZMQConnected); 
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
    createNParams(P_TOFSpecXString, asynParamFloat64Array, P_TOFSpecX, 4);
    createNParams(P_TOFSpecYString, asynParamFloat64Array, P_TOFSpecY, 4);
    createNParams(P_TOFSpecIdxString, asynParamInt32, P_TOFSpecIdx, 4);
    createParam(P_readDCSpectraString, asynParamInt32, &P_readDCSpectra);
    createParam(P_readEventsString, asynParamInt32, &P_readEvents);
    createParam(P_readTOFSpectraString, asynParamInt32, &P_readTOFSpectra);
    createParam(P_resetTOFSpectraString, asynParamInt32, &P_resetTOFSpectra);
    createParam(P_resetDCSpectraString, asynParamInt32, &P_resetDCSpectra);
    
    setStringParam(P_setupFile, "");
    setStringParam(P_error, "");
    setIntegerParam(P_setupDone, 0);
    setIntegerParam(P_ZMQConnected, 0);
    
	//int maxSizes[2][2] = { {16, 20000}, { 16, 4096 } };
    NDDataType_t dataType = NDFloat64; // data type for each frame
    int status = 0;
    for(int i=0; i<maxAddr; ++i)
    {
        //int maxSizeX = maxSizes[i][0];
        //int maxSizeY = maxSizes[i][1];
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

    if (epicsThreadCreate("zmqMonitorPoller",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)zmqMonitorPollerC, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    
    // Create the thread for background tasks (not used at present, could be used for I/O intr scanning) 
    if (epicsThreadCreate("NucInstDigPoller1",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC1, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("NucInstDigPoller2",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC2, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("NucInstDigPoller3",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC3, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("NucInstDigPoller4",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC4, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("NucInstDigPoller5",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC5, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
    if (epicsThreadCreate("NucInstDigPoller6",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC6, this) == 0)
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

void NucInstDig::pollerThreadC6(void* arg)
{
    NucInstDig* driver = (NucInstDig*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread6();
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
    static const char* functionName = "NucInstDigPoller1";
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
    static const char* functionName = "NucInstDigPoller2";
    updateTraces();
}

void NucInstDig::pollerThread3()
{
    static const char* functionName = "NucInstDigPoller3";
    updateEvents();
}

void NucInstDig::pollerThread4()
{
    static const char* functionName = "NucInstDigPoller4";
    updateDCSpectra();
}

void NucInstDig::pollerThread5()
{
    static const char* functionName = "NucInstDigPoller5";
    updateAD();
}

void NucInstDig::pollerThread6()
{
    static const char* functionName = "NucInstDigPoller6";
    updateTOFSpectra();
}

void NucInstDig::zmqMonitorPoller()
{
    static const char* functionName = "zmqMonitorPoller";
    while(true)
    {
        epicsThreadSleep(0.5);
        try
        {
            m_zmq_cmd.pollMonitor();
            m_zmq_stream.pollMonitor();
            m_zmq_events.pollMonitor();
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
        {
            epicsGuard<NucInstDig> _lock(*this);
            if (m_zmq_cmd.connected() && m_zmq_stream.connected() && m_zmq_events.connected()) {
                setIntegerParam(P_ZMQConnected, 1);
                m_connected = true;
            } else {
                setIntegerParam(P_ZMQConnected, 0);
                m_connected = false;
            }
            callParamCallbacks();
        }
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
    fprintf(fp, "NucInstDig driver %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    int connected;
    getIntegerParam(P_ZMQConnected, &connected);
    fprintf(fp, "connected: %s\n", (connected != 0 ? "YES" : "NO"));
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

// NucInstDigConfigure
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

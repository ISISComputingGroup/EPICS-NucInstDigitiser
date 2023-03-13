#ifndef NUCINSTDIG_H
#define NUCINSTDIG_H
 
#include "ADDriver.h"

struct ParamData
{
    std::string name;
    asynParamType type;
    int chan;
    int log_freq;
    ParamData(const std::string& name_, asynParamType type_, int chan_, int log_freq_) : name(name_), type(type_), chan(chan_), log_freq(log_freq_) { }
};

class zmq_monitor_t : public zmq::monitor_t {
    bool m_connected;
public:
    zmq_monitor_t() : zmq::monitor_t(), m_connected(false) { }
    bool connected() const { return m_connected; }
    
    void on_event_connected(const zmq_event_t& event, const char* addr) override
    {
        std::cerr << "ZMQ: Connection from " << addr << std::endl;
        m_connected = true;
    }
    void on_event_disconnected(const zmq_event_t & event, const char* addr) override
    {
        std::cerr << "ZMQ: Disconnect from " << addr << std::endl;
        m_connected = false;
    }        
    void on_event_closed(const zmq_event_t& event, const char* addr) override
    {
        std::cerr << "ZMQ: Closed from " << addr << std::endl;
        m_connected = false;
    }        
    void on_event_bind_failed(const zmq_event_t& event, const char* addr) override
    {
        std::cerr << "ZMQ: Bind failed from " << addr << std::endl;
        m_connected = false;
    }        
    void on_event_connect_retried(const zmq_event_t& event, const char* addr) override
    {
        std::cerr << "ZMQ: Connect retried from " << addr << std::endl;
        m_connected = false;
    }        
    void on_event_connect_delayed(const zmq_event_t& event, const char* addr) override
    {
        std::cerr << "ZMQ: Connect delayed from " << addr << std::endl;
        m_connected = false;
    }        
		    
};

class NucInstDig : public ADDriver
{
public:
    NucInstDig(const char *portName, const char *targetAddress, int dig_idx);
 	static void pollerThreadC1(void* arg);
 	static void pollerThreadC2(void* arg);
 	static void pollerThreadC3(void* arg);
 	static void pollerThreadC4(void* arg);
 	static void pollerThreadC5(void* arg);
    static void zmqMonitorPollerC(void* arg);

    // These are the methods that we override from asynPortDriver
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
	virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
	virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn);
	
    virtual void report(FILE *fp, int details);
    virtual void setShutter(int addr, int open);

private:

    std::atomic<bool> m_connected;

    zmq::context_t m_zmq_cmd_ctx;
    zmq::socket_t m_zmq_cmd_socket;
    zmq_monitor_t m_zmq_cmd_mon;
    zmq::context_t m_zmq_stream_ctx;
    zmq::socket_t m_zmq_stream_socket;
    zmq_monitor_t m_zmq_stream_mon;
    zmq::context_t m_zmq_events_ctx;
    zmq::socket_t m_zmq_events_socket;
    zmq_monitor_t m_zmq_events_mon;
    
    std::string m_targetAddress;

    int P_setup; // int, must be first createParam and in FIRST_NUCINSTDIG_PARAM
    int P_setupFile; // string
    int P_setupDone; // int
    int P_error; // string
    int P_ZMQConnected; // int
    int P_startAcquisition; // int
    int P_stopAcquisition; // int
    int P_DCSpecX[4]; // realarray
    int P_DCSpecY[4]; // realarray
    int P_DCSpecIdx[4]; // int
    int P_traceX[4]; // realarray
    int P_traceY[4]; // realarray
    int P_traceIdx[4]; // int
    int P_readDCSpectra; // int
    int P_readEvents; // int
    int P_configDGTZ; // int
    int P_configBASE; // int
    int P_configHV; // int
    int P_configSTAVES; // int
    int P_resetDCSpectra; // int
    
    std::map<int, ParamData*> m_param_data;
    
	#define FIRST_NUCINSTDIG_PARAM P_setup
	#define LAST_NUCINSTDIG_PARAM P_resetDCSpectra

    NDArray* m_pTraces;
    NDArray* m_pDCSpectra;
    NDArray* m_pRaw;
    
    epicsMutex m_dcLock;
    epicsMutex m_tracesLock;
    epicsMutex m_executeLock;
    
    std::vector<double> m_traces;
    std::vector<double> m_dcSpectra;
    size_t m_NTRACE;
    size_t m_nDCSpec;
    size_t m_nDCPts;
    size_t m_nVoltage;
    
    void updateTraces();
    void updateEvents();
    void updateDCSpectra();
    void updateAD();
    void zmqMonitorPoller();
    void execute(const std::string& type, const std::string& name, const std::string& arg1, const std::string& arg2, rapidjson::Document& doc_recv);
	void executeCmd(const std::string& name, const std::string& args = "");
    void getParameter(const std::string& name, rapidjson::Value& value, int idx = 0);
    void setParameter(const std::string& name, const std::string& value, int idx = 0);
    void setParameter(const std::string& name, double value, int idx = 0);
    void setParameter(const std::string& name, int value, int idx = 0);

    //This is for dynamically creating asyn parameters
    asynStatus drvUserCreate(asynUser* pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize); 
    asynStatus drvUserDestroy(asynUser *pasynUser);
    
    void setup();
    
    void createNParams(const char* name, asynParamType type, int* param, int n);
    
	void pollerThread1();
	void pollerThread2();
	void pollerThread3();
	void pollerThread4();
	void pollerThread5();
    void readData2d(const std::string& name, const std::string& args, std::vector<double>& dataOut, size_t& nspec, size_t& npts);
    void setADAcquire(int addr, int acquire);
    int computeImage(int addr, const std::vector<double>& data, int nx, int ny);
    template <typename epicsType> 
         int computeArray(int addr, const std::vector<double>& data, int maxSizeX, int maxSizeY);

    
    std::vector<double> m_traceX[4];
    std::vector<double> m_traceY[4];
    int m_traceIdx[4];
    std::vector<double> m_DCSpecX[4];
    std::vector<double> m_DCSpecY[4];
    int m_DCSpecIdx[4];
    
    int m_dig_idx;
};

#define P_setupString	            "SETUP"
#define P_setupFileString	        "SETUP_FILE"
#define P_setupDoneString	        "SETUP_DONE"
#define P_ZMQConnectedString	    "ZMQ_CONNECTED"
#define P_errorString	            "ERROR"
#define P_startAcquisitionString	"START"
#define P_stopAcquisitionString	    "STOP"
#define P_configDGTZString          "CONFIG_DGTZ" 
#define P_configBASEString          "CONFIG_BASE" 
#define P_configHVString            "CONFIG_HV" 
#define P_configSTAVESString        "CONFIG_STAVES" 
#define P_resetDCSpectraString	    "RESET_DC_SPECTRA"
#define P_readDCSpectraString	    "READ_DC_SPECTRA"
#define P_readEventsString          "READ_EVENTS"
#define P_DCSpecXString             "DCSPEC%dX"
#define P_DCSpecYString             "DCSPEC%dY"
#define P_DCSpecIdxString           "DCSPEC%dIDX"
#define P_traceXString              "TRACE%dX"
#define P_traceYString              "TRACE%dY"
#define P_traceIdxString            "TRACE%dIDX"

#endif /* NUCINSTDIG_H */

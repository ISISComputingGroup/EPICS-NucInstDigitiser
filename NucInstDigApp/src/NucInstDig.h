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

class ZMQConnectionHandler
{
    zmq::context_t m_zmq_ctx;
    zmq::socket_t* m_zmq_socket;
    zmq_monitor_t* m_zmq_mon;
    zmq::socket_type m_sock_type;
    std::string m_address;
    epicsMutex m_lock;
    bool m_conflate;
    public:
    ZMQConnectionHandler(zmq::socket_type sock_type, const std::string& address, bool conflate = false) : m_sock_type(sock_type), m_address(address), m_zmq_ctx{1}, m_zmq_socket(nullptr), m_zmq_mon(nullptr), m_conflate(conflate)
    {
        init();
    }
    
    void init()
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        int events_to_monitor =  ZMQ_EVENT_CONNECTED|ZMQ_EVENT_DISCONNECTED|ZMQ_EVENT_CLOSED|ZMQ_EVENT_BIND_FAILED|ZMQ_EVENT_CONNECT_DELAYED|ZMQ_EVENT_CONNECT_RETRIED;
        std::cerr << "ZMQ: initialising new socket for " << m_address << std::endl;
        if (m_zmq_socket != nullptr)
        {
            m_zmq_socket->set(zmq::sockopt::linger, 0);
        }
        delete m_zmq_mon;
        delete m_zmq_socket;
        m_zmq_socket = new zmq::socket_t(m_zmq_ctx, m_sock_type);
        m_zmq_socket->set(zmq::sockopt::linger, 5000);
        m_zmq_socket->set(zmq::sockopt::rcvtimeo, 5000);
        m_zmq_socket->set(zmq::sockopt::sndtimeo, 5000);
        if (m_conflate) {
            m_zmq_socket->set(zmq::sockopt::conflate, 1);
        }
        m_zmq_mon = new zmq_monitor_t();
        m_zmq_mon->init(*m_zmq_socket, "inproc://NucInstDigConMon", events_to_monitor);
        m_zmq_socket->connect(m_address);
    }

    bool connected()
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        return m_zmq_mon->connected();
    }
    
    void pollMonitor(int timeout = 10)
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        m_zmq_mon->check_event(timeout);        
    }
    
    zmq::recv_result_t recv(zmq::message_t& message, zmq::recv_flags flags)
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        return m_zmq_socket->recv(message, flags);
    }

    zmq::send_result_t send(zmq::const_buffer buffer, zmq::send_flags flags)
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        return m_zmq_socket->send(buffer, flags);
    }

    zmq::send_result_t send(zmq::message_t &msg, zmq::send_flags flags)
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        return m_zmq_socket->send(msg, flags);
    }

    zmq::send_result_t send(zmq::message_t &&msg, zmq::send_flags flags)
    {
        epicsGuard<epicsMutex> _lock(m_lock);
        return m_zmq_socket->send(msg, flags);
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
 	static void pollerThreadC6(void* arg);
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

    ZMQConnectionHandler m_zmq_cmd;
#ifdef PULL_TRACES
    ZMQConnectionHandler m_zmq_stream;
#endif
    ZMQConnectionHandler m_zmq_events;
    
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
    int P_TOFSpecIdx[4]; // int
    int P_TOFSpecX[4]; // realarray
    int P_TOFSpecY[4]; // realarray
    int P_readDCSpectra; // int
    int P_readEvents; // int
    int P_readTOFSpectra; // int
    int P_readTraces; // int
    int P_configDGTZ; // int
    int P_configBASE; // int
    int P_configHV; // int
    int P_configSTAVES; // int
    int P_resetTOFSpectra; // int
    int P_resetDCSpectra; // int
    
    std::map<int, ParamData*> m_param_data;
    
	#define FIRST_NUCINSTDIG_PARAM P_setup
	#define LAST_NUCINSTDIG_PARAM P_resetDCSpectra

    //NDArray* m_pTraces;
    //NDArray* m_pDCSpectra;
    //NDArray* m_pTOFSpectra;
    NDArray* m_pRaw; // temporary for traces, tof etc. real info in this->pArrays[addr]
    
    epicsMutex m_dcLock;
    epicsMutex m_tracesLock;
    epicsMutex m_TOFSpectraLock;
    epicsMutex m_executeLock;
    
    std::vector<double> m_traces;
    std::vector<double> m_dcSpectra;
    std::vector<double> m_TOFSpectra;
    size_t m_NTRACE;
    size_t m_nDCSpec;
    size_t m_nDCPts;
    size_t m_nVoltage;
    size_t m_nTOFPts;
    size_t m_nTOFSpec;
    
    void updateTraces();
    void updateTracesOnRequest();
    void updateEvents();
    void updateDCSpectra();
    void updateTOFSpectra();
    void updateAD();
    void zmqMonitorPoller();
    void execute(const std::string& type, const std::string& name, const std::string& arg1, const std::string& arg2, rapidjson::Document& doc_recv);
	void executeCmd(const std::string& name, const std::string& args = "");
    void getParameter(const std::string& name, rapidjson::Document& doc_recv, int idx = 0);
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
	void pollerThread6();
    void readData2d(const std::string& name, const std::string& args, std::vector<double>& dataOut, size_t& nspec, size_t& npts);
    void setADAcquire(int addr, int acquire);
    int computeImage(int addr, const std::vector<double>& data_in, int nx, int ny);
    template <typename epicsType> 
         int computeArray(int addr, const std::vector<double>& data, int maxSizeX, int maxSizeY);
         
    int callComputeArray(NDDataType_t dataType, int addr,
      const std::vector<double>& data, int sizeX, int sizeY);
    int rebin(const double* data_in, double xmin_in, double xmax_in, int nin,
               double* data_out, double xmin_out, double xmax_out, int nout);

    std::vector<double> m_traceX[4];
    std::vector<double> m_traceY[4];
    int m_traceIdx[4];
    std::vector<double> m_DCSpecX[4];
    std::vector<double> m_DCSpecY[4];
    int m_DCSpecIdx[4];
    std::vector<double> m_TOFSpecX[4];
    std::vector<double> m_TOFSpecY[4];
    int m_TOFSpecIdx[4];
    
    int m_dig_idx;
    int m_dig_id; // this is our position in g_dig_list
    
    static NDArray* g_rawCombined[3]; // across all digitisers
    static std::vector<NucInstDig*> g_dig_list;
    static epicsMutex g_digCombinedLock;
    public:
    void setDigId(int id) { m_dig_id = id; }
    static void addDigitiser(NucInstDig* dig, int dig_idx) {
        epicsGuard<epicsMutex> _lock(g_digCombinedLock);
        g_dig_list.push_back(dig);
        dig->setDigId(g_dig_list.size() - 1);
    }
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
#define P_resetTOFSpectraString	    "RESET_TOF_SPECTRA"
#define P_readTOFSpectraString	    "READ_TOF_SPECTRA"
#define P_DCSpecXString             "DCSPEC%dX"
#define P_DCSpecYString             "DCSPEC%dY"
#define P_DCSpecIdxString           "DCSPEC%dIDX"
#define P_traceXString              "TRACE%dX"
#define P_traceYString              "TRACE%dY"
#define P_traceIdxString            "TRACE%dIDX"
#define P_TOFSpecXString            "TOFSPEC%dX"
#define P_TOFSpecYString            "TOFSPEC%dY"
#define P_TOFSpecIdxString          "TOFSPEC%dIDX"
#define P_readTracesString          "READ_TRACES"

#endif /* NUCINSTDIG_H */

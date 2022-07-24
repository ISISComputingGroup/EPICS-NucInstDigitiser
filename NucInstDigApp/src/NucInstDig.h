#ifndef NUCINSTDIG_H
#define NUCINSTDIG_H
 
#include "ADDriver.h"

typedef std::vector< std::vector<double> > Vector2D;

class NucInstDig : public ADDriver
{
public:
    NucInstDig(const char *portName, const char *targetAddress);
 	static void pollerThreadC1(void* arg);
 	static void pollerThreadC2(void* arg);
 	static void pollerThreadC3(void* arg);
 	static void pollerThreadC4(void* arg);

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

private:

    zmq::context_t m_zmq_cmd_ctx;
    zmq::socket_t m_zmq_cmd_socket;
    zmq::context_t m_zmq_stream_ctx;
    zmq::socket_t m_zmq_stream_socket;
    zmq::context_t m_zmq_events_ctx;
    zmq::socket_t m_zmq_events_socket;
    std::string m_targetAddress;

    int P_startAcquisition; // int
    int P_stopAcquisition; // int
    int P_DCSpecX[4]; // realarray
    int P_DCSpecY[4]; // realarray
    int P_DCSpecIdx[4]; // int
    int P_traceX[4]; // realarray
    int P_traceY[4]; // realarray
    int P_traceIdx[4]; // int
    int P_readDCSpectra; // int
    int P_resetDCSpectra; // int
    
	#define FIRST_NUCINSTDIG_PARAM P_startAcquisition
	#define LAST_NUCINSTDIG_PARAM P_resetDCSpectra

    NDArray* m_pRaw;
    void updateTraces();
    void updateEvents();
    void updateDCSpectra();
    void execute(const std::string& type, const std::string& name, const std::string& arg1, const std::string& arg2, rapidjson::Document& doc_recv);
	void executeCmd(const std::string& name, const std::string& args);
    void getParameter(const std::string& name, int idx);
    void setParameter(const std::string& name, const std::string& value, int idx);
    void setParameter(const std::string& name, double value, int idx);
    void setParameter(const std::string& name, int value, int idx);
    
    void createNParams(const char* name, asynParamType type, int* param, int n);
    
	void pollerThread1();
	void pollerThread2();
	void pollerThread3();
	void pollerThread4();
    void readData2d(const std::string& name, const std::string& args, Vector2D& dataOut);
    
    std::vector<double> m_traceX[4];
    std::vector<double> m_traceY[4];
    int m_traceIdx[4];
    std::vector<double> m_DCSpecX[4];
    std::vector<double> m_DCSpecY[4];
    int m_DCSpecIdx[4];	  
};

#define P_startAcquisitionString	"START"
#define P_stopAcquisitionString	    "STOP"
#define P_resetDCSpectraString	    "RESET_DC_SPECTRA"
#define P_readDCSpectraString	    "READ_DC_SPECTRA"
#define P_DCSpecXString             "DCSPEC%dX"
#define P_DCSpecYString             "DCSPEC%dY"
#define P_DCSpecIdxString           "DCSPEC%dIDX"
#define P_traceXString              "TRACE%dX"
#define P_traceYString              "TRACE%dY"
#define P_traceIdxString            "TRACE%dIDX"
#endif /* NUCINSTDIG_H */

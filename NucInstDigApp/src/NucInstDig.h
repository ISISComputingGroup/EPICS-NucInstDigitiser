#ifndef NUCINSTDIG_H
#define NUCINSTDIG_H
 
#include "ADDriver.h"

class NucInstDig : public ADDriver
{
public:
    NucInstDig(const char *portName, const char *targetAddress);
 	static void pollerThreadC1(void* arg);
 	static void pollerThreadC2(void* arg);

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
    std::string m_targetAddress;

    int P_startAcquisition; // int
    int P_specY; // realarray
    int P_specX; // realarray
    int P_traceY; // realarray
    int P_traceX; // realarray
	int P_specNum; // int
	int P_traceNum; // int
    
	#define FIRST_NUCINSTDIG_PARAM P_startAcquisition
	#define LAST_NUCINSTDIG_PARAM P_traceNum

    NDArray* m_pRaw;
    void updateTraces();
    void execute(const std::string& type, const std::string& name, const std::string& arg1, const std::string& arg2);
	
	void pollerThread1();
	void pollerThread2();
    
    std::vector<double> m_traceX;
    std::vector<double> m_traceY;
    std::vector<double> m_specX;
    std::vector<double> m_specY;
	  
};

#define P_startAcquisitionString	"START"
#define P_specNumString             "SPECNUM"
#define P_traceNumString            "TRACENUM"
#define P_specXString             "SPECX"
#define P_specYString             "SPECY"
#define P_traceXString            "TRACEX"
#define P_traceYString            "TRACEY"

#endif /* NUCINSTDIG_H */

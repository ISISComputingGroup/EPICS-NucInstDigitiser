#ifndef NUCINSTDIG_H
#define NUCINSTDIG_H
 
#include "ADDriver.h"

class NucInstDig : public ADDriver
{
public:
    NucInstDig(const char *portName);
 	static void pollerThreadC1(void* arg);

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

	int P_GoodUAH; // double
	#define FIRST_NUCINSTDIG_PARAM P_GoodUAH
    int P_ErrMsgs; // char
	#define LAST_NUCINSTDIG_PARAM P_ErrMsgs

    NDArray* m_pRaw;
	
	void pollerThread1();
	  
};

#define P_GoodUAHString	"GOODUAH"

#endif /* NUCINSTDIG_H */

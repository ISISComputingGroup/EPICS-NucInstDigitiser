TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

USR_INCLUDES += -I"$(AREA_DETECTOR)" -I$(CPPZMQ) 
USR_CXXFLAGS += -DNOMINMAX

ifneq ($(findstring static,$(EPICS_HOST_ARCH)),)
USR_CXXFLAGS += -DZMQ_STATIC
endif

LIBRARY_IOC += NucInstDig

PROD_IOC += nidg_send nidg_stream

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install NucInstDig.dbd into <top>/dbd
DBD += NucInstDig.dbd

# specify all source files to be compiled and added to the library
NucInstDig_SRCS += NucInstDig.cpp
NucInstDig_LIBS += asyn zmq
NucInstDig_LIBS += $(EPICS_BASE_IOC_LIBS)
NucInstDig_LIBS_WIN32 += oncrpc
NucInstDig_SYS_LIBS_WIN32 += Iphlpapi

nidg_send_SRCS += nidg_send.cpp
nidg_send_LIBS += zmq
nidg_send_LIBS += $(EPICS_BASE_IOC_LIBS)

nidg_stream_SRCS += nidg_stream.cpp
nidg_stream_LIBS += zmq
nidg_stream_LIBS += $(EPICS_BASE_IOC_LIBS)

nidg_send_SYS_LIBS_WIN32 += Iphlpapi
nidg_stream_SYS_LIBS_WIN32 += Iphlpapi

#===========================
include $(ADCORE)/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_Linux += ADApogee

LIB_SRCS += Apogee.cpp
LIB_SRCS += ApogeeWrapper.cpp

USR_CFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0
USR_CXXFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0

ADApogee_LDFLAGS += -L/opt/apogee/lib/
ADApogee_SYS_LIBS += apogee boost_regex-mt
USR_INCLUDES += -I/opt/apogee/include/libapogee-3.0/apogee/

DBD += ApogeeSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


#include <stdint.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <iocsh.h>
#include <epicsExport.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsTime.h>

#include <Alta.h>
#include <FindDeviceUsb.h>
#include "ApogeeWrapper.h"

#include "ADDriver.h"


class ApogeeDriver : public ADDriver
{
	public:
	ApogeeDriver(const char* port_name);

	void connect_thread();
	void acquire_thread();
	void temperature_thread();

	virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
	virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);

	protected:
	int APOGEE_Camera_Connected;
	#define APOGEE_FIRST_PARAM APOGEE_Camera_Connected

	#define APOGEE_LAST_PARAM APOGEE_Camera_Connected

	bool connected;


	private:
	double CONNECT_POLL_DELAY;
	double ACQUIRE_POLL_DELAY;

	Alta camera;
	NDArray* pImage;

	void connect();
	void init();
	void disconnect();

	void startAcquire();
	void startMonitor();

	void processImage();

	Apg::Status finishExposure();

	epicsEventId startAcquireEvent;
	epicsEventId stopAcquireEvent;
};

#define NUM_APOGEE_PARAMS ((int) (&APOGEE_LAST_PARAM - &APOGEE_FIRST_PARAM + 1))

void connect_thread_callback(void* arg)
{
	ApogeeDriver* driver = (ApogeeDriver*) arg;

	driver->connect_thread();
}

void acquire_thread_callback(void* arg)
{
	ApogeeDriver* driver = (ApogeeDriver*) arg;

	driver->acquire_thread();
}

void temperature_thread_callback(void* arg)
{
	ApogeeDriver* driver = (ApogeeDriver*) arg;

	driver->temperature_thread();
}


ApogeeDriver::ApogeeDriver(const char* port_name):
    ADDriver(port_name,
             1,                 //Max # of Addresses
             NUM_APOGEE_PARAMS, //# of Params
             0,                 //Max Buffers
             0,                 //Max Memory
             asynEnumMask,                      //Interface Mask
             asynEnumMask,                      //Interrupt Mask
             ASYN_CANBLOCK | ASYN_MULTIDEVICE,  //Interface Type
             1,                 //Autoconnect
             0,                 //Thread Priority
             0)                 //Initial Stack Size
{
	this->connected = false;

	this->CONNECT_POLL_DELAY = 1.0;
	this->ACQUIRE_POLL_DELAY = 0.001;

	createParam("APOGEE_Camera_Connected", asynParamInt32,   &this->APOGEE_Camera_Connected);

	setDoubleParam(this->ADAcquireTime, 0.1);

	this->startAcquireEvent = epicsEventCreate(epicsEventEmpty);
	this->stopAcquireEvent  = epicsEventCreate(epicsEventEmpty);

	this->connect();
}

void ApogeeDriver::connect()
{
	std::stringstream name_stream;
	std::string thread_name;

	name_stream << "ApogeeDriver::connect()";
	name_stream >> thread_name;

	epicsThreadCreate(thread_name.c_str(),
	                  epicsThreadPriorityLow,
	                  epicsThreadGetStackSize(epicsThreadStackMedium),
	                  (EPICSTHREADFUNC)::connect_thread_callback,
	                  this);
}

void ApogeeDriver::disconnect()
{
	if (this->connected)    { this->camera.CloseConnection(); }

	this->connected = false;
}

void ApogeeDriver::connect_thread()
{
	this->lock();

	FindDeviceUsb searcher;

	this->disconnect();

	while(not this->connected)
	{
		std::string list = searcher.Find();

		std::string addr     = Apogee::GetStr(list, "address=");
		uint16_t    id       = Apogee::GetInt(list, "id=");
		uint16_t    firmware = Apogee::GetInt(list, "firmwareRev=");

		try
		{
			this->camera.OpenConnection("usb", addr, firmware, id);
		}
		catch (std::runtime_error e)
		{
			epicsThreadSleep(this->CONNECT_POLL_DELAY);
			continue;
		}

		try
		{
			this->camera.Init();
		}
		catch (std::runtime_error e)
		{
			this->camera.CloseConnection();
			epicsThreadSleep(this->CONNECT_POLL_DELAY);
			continue;
		}

		Apg::Status status = this->camera.GetImagingStatus();

		if (status == Apg::Status_ConnectionError)
		{
			this->disconnect();
			this->connect();
			this->unlock();

			return;
		}
		else if (status == Apg::Status_DataError or status == Apg::Status_PatternError)
		{
			this->camera.Reset();
		}

		this->connected = true;
	}

	this->unlock();

	this->init();
	this->startAcquire();
	this->startMonitor();
}

void ApogeeDriver::startAcquire()
{
	std::stringstream name_stream;
	std::string thread_name;

	name_stream << "ApogeeDriver::acquire()";
	name_stream >> thread_name;

	epicsThreadCreate(thread_name.c_str(),
	                  epicsThreadPriorityLow,
	                  epicsThreadGetStackSize(epicsThreadStackMedium),
	                  (EPICSTHREADFUNC)::acquire_thread_callback,
	                  this);
}

void ApogeeDriver::startMonitor()
{
	std::stringstream name_stream;
	std::string thread_name;

	name_stream << "ApogeeDriver::monitor()";
	name_stream >> thread_name;

	epicsThreadCreate(thread_name.c_str(),
	                  epicsThreadPriorityLow,
	                  epicsThreadGetStackSize(epicsThreadStackMedium),
	                  (EPICSTHREADFUNC)::temperature_thread_callback,
	                  this);
}

void ApogeeDriver::init()
{
	int row_start = this->camera.GetRoiStartRow();
	int col_start = this->camera.GetRoiStartCol();
	int num_rows  = this->camera.GetRoiNumRows();
	int num_cols  = this->camera.GetRoiNumCols();

	int max_rows = this->camera.GetMaxImgRows();
	int max_cols = this->camera.GetMaxImgCols();

	this->setIntegerParam(this->ADMinY,  row_start);
	this->setIntegerParam(this->ADMinX,  col_start);
	this->setIntegerParam(this->ADSizeY, num_rows);
	this->setIntegerParam(this->ADSizeX, num_cols);
	this->setIntegerParam(this->ADMaxSizeX, max_cols);
	this->setIntegerParam(this->ADMaxSizeY, max_rows);

	std::string model = this->camera.GetModel();
	std::string serial = this->camera.GetSerialNumber();

	this->setStringParam(this->ADManufacturer, "Apogee");
	this->setStringParam(this->ADModel, model.c_str());
	this->setStringParam(this->ADSerialNumber, serial.c_str());

	this->camera.SetCooler(true);

	this->callParamCallbacks();
}

Apg::Status ApogeeDriver::finishExposure()
{
	Apg::Status output;

	//this->setIntegerParam(this->ADStatus, ADStatusWaiting);
	//this->callParamCallbacks();

	while (true)
	{
		epicsEventWaitStatus status;

		this->unlock();
		status = epicsEventWaitWithTimeout(this->stopAcquireEvent, this->ACQUIRE_POLL_DELAY);
		this->lock();

		if (status == epicsEventWaitOK)    { this->camera.StopExposure(false); }

		output = this->camera.GetImagingStatus();

		if (output == Apg::Status_ImageReady or
		    output == Apg::Status_ConnectionError or
		    output == Apg::Status_DataError or
		    output == Apg::Status_PatternError or
		    status == epicsEventWaitOK)
		{
			return output;
		}
	}
}

void ApogeeDriver::acquire_thread()
{
	this->lock();

	while(this->connected)
	{
		this->unlock();
			epicsEventWait(this->startAcquireEvent);
		this->lock();

		this->setIntegerParam(this->ADAcquire, 1);
		this->callParamCallbacks();

		double exposure;
		double period;
		int num_rows, num_cols;

		this->getDoubleParam (this->ADAcquireTime,  &exposure);
		this->getDoubleParam (this->ADAcquirePeriod, &period);
		this->getIntegerParam(this->ADSizeX, &num_cols);
		this->getIntegerParam(this->ADSizeY, &num_rows);

		std::vector<uint16_t> frame(num_rows * num_cols);
		size_t image_dims[2];

		image_dims[0] = num_cols;
		image_dims[1] = num_rows;

		int num_images = 1;
		int image_mode, array_callbacks;

		this->getIntegerParam(this->ADImageMode,      &image_mode);
		this->getIntegerParam(this->NDArrayCallbacks, &array_callbacks);

		if (image_mode == ADImageMultiple) { this->getIntegerParam(this->ADNumImages, &num_images); }


		for (int acq_counter = 0; acq_counter < num_images;)
		{
			Apg::Status status = this->camera.GetImagingStatus();

			if (status == Apg::Status_DataError or status == Apg::Status_Exposing)
			{
				this->camera.Reset();
			}
			else if (status == Apg::Status_ConnectionError)
			{
				this->disconnect();
				this->connect();
				this->unlock();

				return;
			}

			this->camera.StartExposure(exposure, true);

			this->setIntegerParam(this->ADStatus, ADStatusAcquire);
			this->callParamCallbacks();

			status = this->finishExposure();

			this->setIntegerParam(this->ADStatus, ADStatusReadout);
			this->callParamCallbacks();

			if (status == Apg::Status_ImageReady)
			{
				this->camera.GetImage(frame);

				this->pArrays[0] = this->pNDArrayPool->alloc(2, image_dims, NDUInt16, 0, NULL);

				if (this->pArrays[0] != NULL)
				{
					pImage = this->pArrays[0];

					memcpy(pImage->pData, frame.data(), num_rows * num_cols * sizeof(uint16_t));

					this->processImage();
				}
			}
			else if (status == Apg::Status_ConnectionError)
			{
				this->disconnect();
				this->connect();
				this->unlock();

				return;
			}
			else
			{
				if (status == Apg::Status_DataError)
				{
					//Change status string
					printf("Data Error\n");
				}
				else if (status == Apg::Status_PatternError)
				{
					//Change status string
					printf("Status Error\n");
				}

				break;
			}

			this->setIntegerParam(this->ADStatus, ADStatusWaiting);
			this->callParamCallbacks();

			if (image_mode != ADImageContinuous)    { acq_counter += 1; }
			if (image_mode != ADImageSingle)        { epicsThreadSleep(period - exposure); }
		}

		this->setIntegerParam(this->ADAcquire, 0);
		this->setIntegerParam(this->ADStatus, ADStatusIdle);
		this->callParamCallbacks();
	}
}

void ApogeeDriver::processImage()
{
	int image_number;
	int total_images;

	this->getIntegerParam(this->NDArrayCounter,     &image_number);
	this->getIntegerParam(this->ADNumImagesCounter, &total_images);
	image_number += 1;
	total_images += 1;
	this->setIntegerParam(this->NDArrayCounter,     image_number);
	this->setIntegerParam(this->ADNumImagesCounter, total_images);

	pImage->uniqueId = image_number;

	updateTimeStamp(&pImage->epicsTS);
	pImage->timeStamp = pImage->epicsTS.secPastEpoch + pImage->epicsTS.nsec/1e9;

	getAttributes(pImage->pAttributeList);

	this->unlock();
		doCallbacksGenericPointer(pImage, NDArrayData, 0);
	this->lock();

	pImage->release();
	this->callParamCallbacks();
}

void ApogeeDriver::temperature_thread()
{
	while (true)
	{
		this->lock();
		if (this->connected)
		{
			double temperature = this->camera.GetTempCcd();

			this->setDoubleParam(this->ADTemperatureActual, temperature);
			this->callParamCallbacks();
		}
		else
		{
			this->unlock();
			return;
		}
		this->unlock();
		epicsThreadSleep(1.0);
	}
}

asynStatus ApogeeDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;

	this->setIntegerParam(function, value);

	if (function == ADAcquire)
	{
		if (value)    { epicsEventSignal(this->startAcquireEvent); }
		else          { epicsEventSignal(this->stopAcquireEvent); }
	}
	else if (function == ADSizeX)
	{
		this->camera.SetRoiNumCols((uint16_t) value);
		this->setIntegerParam(function, (epicsInt32) this->camera.GetRoiNumCols());
	}
	else if (function == ADSizeY)
	{
		this->camera.SetRoiNumRows((uint16_t) value);
		this->setIntegerParam(function, (epicsInt32) this->camera.GetRoiNumRows());
	}
	else if (function == ADMinX)
	{
		this->camera.SetRoiStartCol((uint16_t) value);
		this->setIntegerParam(function, (epicsInt32) this->camera.GetRoiStartCol());
	}
	else if (function == ADMinY)
	{
		this->camera.SetRoiStartRow((uint16_t) value);
		this->setIntegerParam(function, (epicsInt32) this->camera.GetRoiStartRow());
	}
	else if (function == ADBinX)
	{
		this->camera.SetRoiBinCol((uint16_t) value);
		this->setIntegerParam(function, (epicsInt32) this->camera.GetRoiBinCol());
	}
	else if (function == ADBinY)
	{
		this->camera.SetRoiBinRow((uint16_t) value);
		this->setIntegerParam(function, (epicsInt32) this->camera.GetRoiBinRow());
	}
	else if (function == ADTriggerMode)
	{
		if (value == ADTriggerInternal)    { this->camera.SetExternalTrigger(false, Apg::TriggerMode_Normal, Apg::TriggerType_Group); }
		else                               { this->camera.SetExternalTrigger(true, Apg::TriggerMode_Normal, Apg::TriggerType_Each); }
	}

	this->callParamCallbacks();

	return asynSuccess;
}


asynStatus ApogeeDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;

	this->setDoubleParam(function, value);

	if (function == ADTemperature)
	{
		this->camera.SetCoolerSetPoint((double) value);
		this->setDoubleParam(function, this->camera.GetCoolerSetPoint());
	}

	this->callParamCallbacks();

	return asynSuccess;
}

extern "C" void create_apogee(const char* portname)
{
	new ApogeeDriver(portname);
}

static const iocshArg apogeeArg0 = {"Port Name", iocshArgString};

static const iocshArg * const apogeeArgs[] = { &apogeeArg0 };

static const iocshFuncDef apogeeFuncDef = {"createApogeeDriver", 1, apogeeArgs};

static void apogeeCallFunc(const iocshArgBuf* args)
{
	create_apogee(args[0].sval);
}

void apogeeRegister(void)
{
	iocshRegister(&apogeeFuncDef, apogeeCallFunc);
}

extern "C"
{
	epicsExportRegistrar(apogeeRegister);
}

//
//  GPSmgr.cpp
//  GPStest
//
//  Created by Vincent Moscaritolo on 5/18/22.
//

#include "GPSmgr.hpp"
#include <fcntl.h>
#include <cassert>
#include <string.h>

#include <stdlib.h>
#include <errno.h> // Error integer and strerror() function
#include "ErrorMgr.hpp"
//#include "GPSmgr.hpp"
//#include "PiCarDB.hpp"
//#include "PropValKeys.hpp"
#include "utm.hpp"

#ifndef PI
#define PI           3.14159265358979323e0    /* PI                        */
#endif


enum UBLOX_Register
{
  UBLOX_BYTES_AVAIL  = 0xFD,
  UBLOX_DATA_STREAM = 0xFF,
};

 
typedef void * (*THREADFUNCPTR)(void *);

GPSmgr::GPSmgr() : _nmea( (void*)_nmeaBuffer, sizeof(_nmeaBuffer), this ){
	_isSetup = false;
	_shouldRead = false;
	_nmea.clear();
	
	_isRunning = true;

	pthread_create(&_TID, NULL,
										  (THREADFUNCPTR) &GPSmgr::GPSReaderThread, (void*)this);

	
}

GPSmgr::~GPSmgr(){
	stop();
	
	pthread_mutex_lock (&_mutex);
	_isRunning = false;
	_shouldRead = false;
	pthread_cond_signal(&_cond);
	pthread_mutex_unlock (&_mutex);
	pthread_join(_TID, NULL);
 }


bool GPSmgr::begin(uint8_t deviceAddress){
	int error = 0;

	return begin(deviceAddress, error);
}
 
bool GPSmgr::begin(uint8_t deviceAddress,   int &error){
	
	reset();
	_nmea.clear();
	_shouldRead = false;

	if( _i2cPort.begin(deviceAddress, error) ){
			_isSetup = true;
	}
	
	return _isSetup;
}
 
void GPSmgr::stop(){
	_isSetup = false;
	reset();
	_nmea.clear();

	_i2cPort.stop();
}

uint8_t	GPSmgr::getDevAddr(){
	return _i2cPort.getDevAddr();
};

 

bool  GPSmgr::isConnected() {
	return _isSetup;
};

 
bool GPSmgr::setShouldRead(bool shouldRead){
	if(_isSetup && _isRunning){
		_shouldRead = true;
		return true;
	}
	return false;
}


bool GPSmgr::reset(){

	pthread_mutex_lock (&_mutex);
	_lastLocation.isValid = false;
	_lastLocation.altitude = false;
	_lastLocation.HDOP = 255;
	pthread_mutex_unlock (&_mutex);

	return true;
}
 
bool	GPSmgr::GetLocation(GPSLocation_t & location){
 
	bool success = false;
	pthread_mutex_lock (&_mutex);
	if(_lastLocation.isValid ){
		location = _lastLocation;
		success = true;
	}
	
	pthread_mutex_unlock (&_mutex);
	return success;
}


bool GPSmgr::GetVelocity(GPSVelocity_t& velocity){
	bool success = false;
	pthread_mutex_lock (&_mutex);
 
	if(_lastVelocity.isValid ){
		velocity = _lastVelocity;
		success = true;
	}
 
	pthread_mutex_unlock (&_mutex);
	return success;
}


// MARK: -  Utilities
 

string GPSmgr::UTMString(GPSLocation_t location){
	string str = string();
	
	if(location.isValid){
		long  Zone;
		char 	latBand;
		char  Hemisphere;
		double Easting;
		double Northing;
	
		double latRad = (PI/180.) * location.latitude;
		double lonRad = (PI/180.) * location.longitude;
	
		if( Convert_Geodetic_To_UTM(latRad, lonRad,
											 &Zone,&latBand, &Hemisphere, &Easting, &Northing ) == UTM_NO_ERROR){
			
			char utmBuffer[32] = {0};
			sprintf(utmBuffer,  "%d%c %ld %ld", (int)Zone, latBand, (long) Easting, (long) Northing);
			str = string(utmBuffer);
		}
	}
	
	return str;
	
}

string GPSmgr::NavString(char navSystem ){
	string str = string();
	switch(navSystem){
		case 'N' : str = "GNSS"; break;
		case 'P' : str = "GPS"; break;
		case 'L' : str = "GLONASS"; break;
		case 'A' : str = "Galileo"; break;
		default: break;
	}
	
	return str;
}

// MARK: -  GPSReader thread


// call then when _nmea.process  is true
void GPSmgr::processNMEA(){
	
//	PiCarDB*			db 		= PiCarMgr::shared()->db();
	string msgID =  string(_nmea.getMessageID());
	
 
	//  GGA	Global Positioning System Fix Data
	if( msgID ==  "GGA") {
		//Global Positioning System Fix Data
		
		{
			long  tmp;
			time_t now = time(NULL);

			pthread_mutex_lock (&_mutex);
			memset((void*)&_lastLocation, 0, sizeof(_lastLocation));
			_lastLocation.isValid = _nmea.isValid();
			_lastLocation.latitude = _nmea.getLatitude() / 1e6;
			_lastLocation.longitude = _nmea.getLongitude() / 1e6;
			_lastLocation.altitudeIsValid = _nmea.getAltitude(tmp);
			_lastLocation.altitude 		= tmp * 0.001;
			_lastLocation.navSystem 	= _nmea.getNavSystem();
			_lastLocation.HDOP 			= _nmea.getHDOP();
			_lastLocation.numSat 		= _nmea.getNumSatellites();
			_lastLocation.geoidHeightValid  = _nmea.getGeoidHeight(tmp);
			_lastLocation.geoidHeight 		= tmp * 0.001;
			_lastLocation.timestamp = now;
			pthread_mutex_unlock (&_mutex);
			
		}
		
	}
	else 	if( msgID ==  "RMC") {
		//Recommended Minimum
		
		time_t now = time(NULL);

		pthread_mutex_lock (&_mutex);
		memset((void*)&_lastVelocity, 0, sizeof(_lastVelocity));
		_lastVelocity.isValid = _nmea.isValid();
		_lastVelocity.heading = _nmea.getCourse();
		_lastVelocity.speed = _nmea.getSpeed()/1000.;
		_lastVelocity.timestamp = now;
		pthread_mutex_unlock (&_mutex);

	}
}


static void  UnknownSentenceHandler(MicroNMEA & nmea, void *context){
//	GPSmgr* d = (GPSmgr*)context;
	
//	printf("UNKN |%s|\n", nmea.getSentence());
 
	/*
	 
	 */
};


void GPSmgr::GPSReader(){
	
	_nmea.setUnknownSentenceHandler(UnknownSentenceHandler);
 
	while(_isRunning){
		
		// if not setup // check back later
		if(!_shouldRead ){
			sleep(1);
			continue;
		}
  
#if 1
		uint8_t b;
		
		if(_i2cPort.readByte(b)){
			if(b == 0xff){
				// not ready.. wait a bit
				usleep(10000);
			}
			else {
				if(_nmea.process(b)){
					processNMEA();
				}
			}
		}
		else {
			printf("read from GPS failed\n");
			_shouldRead = false;
			
		}

#else
 
		
		uint16_t len = 0;
		if(_i2cPort.readWord(UBLOX_BYTES_AVAIL, len)
			&& (len > 0) && (len != 0xffff)){
			
			for(uint16_t i = 0; i < len; i++){
				uint8_t b;
				
				if(i == 0){
					if(! _i2cPort.readByte(UBLOX_DATA_STREAM, b)) break;
				}
				else {
					if(! _i2cPort.readByte(b)) break;
				}
				
				if(_nmea.process(b)){
					processNMEA();
				}
			}
			
		}
		else {
			usleep(1000);
		}
		
#endif
		
	}
}



void* GPSmgr::GPSReaderThread(void *context){
	GPSmgr* d = (GPSmgr*)context;

	//   the pthread_cleanup_push needs to be balanced with pthread_cleanup_pop
	pthread_cleanup_push(   &GPSmgr::GPSReaderThreadCleanup ,context);
 
	d->GPSReader();
	
	pthread_exit(NULL);
	
	pthread_cleanup_pop(0);
	return((void *)1);
}

 
void GPSmgr::GPSReaderThreadCleanup(void *context){
	//GPSmgr* d = (GPSmgr*)context;
 
	printf("cleanup GPSReader\n");
}
 

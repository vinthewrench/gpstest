//
//  main.cpp
//  Duppa
//
//  Created by Vincent Moscaritolo on 5/14/22.
//



#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "CommonDefs.hpp"
#include "GPSmgr.hpp"



int main(int argc, const char * argv[]) {
	
	GPSmgr _gps;
	
	try {
		int error = 0;
		
#if defined(__APPLE__)
		const char* path_gps  = "/dev/cu.usbmodem14101";
#else
		const char* path_gps  = "/dev/ttyAMA1";
#endif
		
		if(!_gps.begin(path_gps, B38400, error))
			throw Exception("failed to setup GPS.  error: %d", error);
		
		
		while(true){
			
			
			GPSLocation_t location;
			if(_gps.GetLocation(location)){
				string utm = GPSmgr::UTMString(location);
				
				printf("UTM: %s\n", utm.c_str());
				
			}
			sleep(1);
			
		}
		
		
//		_gps.stop();
	}
	
	catch ( const Exception& e)  {
		
		// display error on fail..
		
		printf("\tError %d %s\n\n", e.getErrorNumber(), e.what());
	}
	catch (std::invalid_argument& e)
	{
		// display error on fail..
		
		printf("EXCEPTION: %s ",e.what() );
	}
	
	
	return 0;
}

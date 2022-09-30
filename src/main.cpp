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
#include <math.h>
#include "CommonDefs.hpp"
#include "GPSmgr.hpp"



int main(int argc, const char * argv[]) {
	
	GPSmgr _gps;

	typedef struct  {
		string 			name;
		GPSLocation_t location;
	} waypoint_t;
	
	
	vector<waypoint_t>  waypoints = {
		{.name = "Dog Park", .location.longitude =-122.8161,  .location.latitude =  42.26521},
		{.name = "Station 7", .location.longitude =-122.87728,  .location.latitude =  42.42727 },
		{.name = "285 Beach", .location.longitude = -122.70219,  .location.latitude =  42.18968 },
		{.name = "MT Ashland", .location.longitude =-122.71926,  .location.latitude =  42.08173 },
		{.name = "SNAP South", .location.longitude =	-122.67433,  .location.latitude =  42.18602},
		{.name = "Food 4 Less", .location.longitude =-122.87178,  .location.latitude =  42.34901},
		{.name = "Crater Lake", .location.longitude =-122.14401,  .location.latitude =  42.93944},

 	};
 
	
		
	try {
		int error = 0;
		
#if defined(__APPLE__)
	const char* path_gps  = "/dev/cu.usbmodem14301";
 
#else
		const char* path_gps  = "/dev/ttyAMA1";
#endif
		
		if(!_gps.begin(path_gps, B38400, error))
			throw Exception("failed to setup GPS.  error: %d", error);

		
		printf("debug in UBX mode\n");
		
		GPSLocation_t here;
		
		while(true){
			if(_gps.GetLocation(here) && here.isValid){

				printf("\x1b[2J");

				constexpr double  M2FT = 	3.2808399;

				printf("%-10s: %-2.1f \n", "PDOP", here.HDOP/10.);
				printf("%-10s: %-3d \n", "Sats", here.numSat);
	
				printf("%-10s: (%3.5f, %3.5f)\n", "LAT/LONG", here.latitude,here.longitude);
			   printf("%-10s: %10s\n", "UTM",  GPSmgr::UTMString(here).c_str());
				printf("%-10s: %-5.1f ft\n", "Altitude",  here.altitude * M2FT);
		
				//				for(auto wp : waypoints){
				//
				//					auto r = GPSmgr::dist_bearing(here,wp.location);
				//
				//					printf("%-12s  %6.3f mi @ %9.5f° \n", wp.name.c_str(), r.first * 0.6213711922 , r.second);
				//				}
				//
				//				break;

				
				GPSVelocity_t velocity;
				if(_gps.GetVelocity(velocity) && velocity.isValid){
	 				printf("%-10s: %-5.1f mph\n", "Speed",  velocity.speed *  1.150779);
					printf("%-10s: %-3d° %2s\n", "Heading", int(round( velocity.heading)),
									GPSmgr::headingStringFromHeading(velocity.heading) .c_str());
	  				}
				printf("\n");
			}
 

			sleep(1);
		}
		_gps.stop();

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

/* © 2017 KAI OS TECHNOLOGIES (HONG KONG) LIMITED, all rights reserved. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include "hardware/gps.h"
#include <stdbool.h>

#define GPS_DELETE_ALL                           0xFFFFFFFF
#define GPS_DELETE_WARM                          0x00000001
#define ULP_CAPABILITY                  0x0000020

static bool nmea_display_flag = 0;
static const GpsInterface *sGpsInterface = NULL;
enum gps_start_type {
hot,
cold,
warm
};
static enum gps_start_type g_gps_start_type =hot;
static bool svInfo_display_flag = 0;
static  bool gpsotanb_display_flag = 0;
static struct timeval tvafter,tvpre,gps_tvafter,gps_tvpre;
static struct timezone tz,gps_tz;

static void start_count(){
	gettimeofday (&tvpre , &tz);
}

static bool judge(){
	const char* path = "/data/testbox_log";
	if(opendir(path) == NULL){
		if(mkdir(path, 0755) < 0){
			printf("can't creat directory %s\n",path);
			return 0;
		}
	}
	return 1;
}

static void start_count_gps_positioned(){
	gettimeofday (&gps_tvpre, &gps_tz);
}

static double stop_count(){//return ms different time interval
	gettimeofday (&tvafter , &tz);
	return (double)(tvafter.tv_sec-tvpre.tv_sec)*1000+(tvafter.tv_usec-tvpre.tv_usec)/1000;
}

static double stop_count_gps_positioned(){//return ms different time interval
	gettimeofday (&gps_tvafter, &gps_tz);
	return (double)(gps_tvafter.tv_sec-gps_tvpre.tv_sec)*1000+(gps_tvafter.tv_usec-gps_tvpre.tv_usec)/1000;
}


/*registed call back for gps*/
static void get_location_callback(GpsLocation* location){
	double gps_time_interval = 0;
        static bool firstentry = false;
	    printf("latitude:%lf\nlongitude:%lf\naltitude%lf\n%lf,%lf,%lf,%lld",location->latitude,
                                                        location->longitude,
                                                        location->altitude,
                                                        location->accuracy,
                                                        location->bearing,
                                                        location->speed,
                                                        location->timestamp);
        if (firstentry == false)
        {
            gps_time_interval = stop_count_gps_positioned();
            printf("gps use %fs to get the position\n" , gps_time_interval/1000);
            firstentry = true;
        }
       
}

static void get_status_callback(GpsStatus* status){
	printf("%s\n",__FUNCTION__);
	printf("get gps status\n");
}

static void get_sv_status_callback(GpsSvStatus* sv_info){
	printf("[goof]get gps sv status\n");
	/*todo: add gps sv info tracking*/

	if(svInfo_display_flag)
	{
		int i = 0;
		printf("number: \t%d\n", sv_info->num_svs);
		for(i=0;i<sv_info->num_svs;i++){
			printf("prn: \t%d\n", sv_info->sv_list[i].prn);
			printf("snr: \t%f\n", sv_info->sv_list[i].snr);
		}
	}

	if(svInfo_display_flag)
	{
		static FILE * fp = NULL;
		if(fp != NULL) {
			return;
		}

		if(sv_info->num_svs >= 0) {
			int i;
			int num = (int)(sv_info->num_svs);

			mode_t proc_mask = umask(0);
			if(judge() == false){
				return -1;
				}
			fp = fopen("/data/testbox_log/gps_info.txt", "wb");
			if (NULL == fp)
				{
					printf("fp == NULL\n");
				}
    
			fprintf(fp, "{");

			fprintf(fp, "\"num\": %d", num);
			printf("sv_info->num_svs = %d, num = %d\n",sv_info->num_svs,num);
			if(sv_info->num_svs != 0) {
				fprintf(fp, ",");
			}

			if(num > 0) {
				fprintf(fp, "\"gps\": [");

				for(i=0; i<num; i++) {
					fprintf(fp, "{\"prn\": %d, \"snr\": %f}", sv_info->sv_list[i].prn, sv_info->sv_list[i].snr);
					if(i<num-1) {
						fprintf(fp, ",");
					}
				}

				fprintf(fp, "]");
			}

			fprintf(fp, "}");
			fclose(fp);
			umask(proc_mask); 
			fp = NULL;
		}
	}
}

static void get_nmea_callback(GpsUtcTime timestamp, const char* nmea, int length){
	if(nmea_display_flag){
		printf("*** nmea info ***\n");
		printf("timestamp:\t%lld\n", timestamp);
		printf("length:   \t%d\n", length);
		printf("nmea:     \t%s\n", nmea);		
	}

	if(nmea_display_flag)
	{
		static FILE * fp = NULL;
		/*if(isWrite < 100) */{
			if(judge() == false){
				return -1;
			}
			fp = fopen("/data/testbox_log/gps_nmea.txt", "a+");
			if (NULL == fp) {
				return;
			}
			fprintf(fp, "%s", nmea); //clear output string.
			fclose(fp);
			fp = NULL;
		}
	}
	if (gpsotanb_display_flag)
	{
		static FILE * fp = NULL;
		fp = fopen("/storage/sdcard/GPSOTA/gpsotaNb.log", "a+");
		if (NULL == fp) {
			return;
		}
		if (strncmp(nmea, "$GPGSV", 6) == 0 || strncmp(nmea, "$GPGGA", 6) == 0)
		{
			fprintf(fp, "%s", nmea);
		}
		fclose(fp);
		fp = NULL;
	}
}

static void set_capabilities(uint32_t capabilities){
	printf("%s\n",__FUNCTION__);
	printf("Capabilities:%x\n",capabilities);
	if(capabilities & GPS_CAPABILITY_SCHEDULING){
		printf("GPS_CAPABILITY_SCHEDULING|");	
	}
	if(capabilities & GPS_CAPABILITY_MSB){
		printf("GPS_CAPABILITY_MSB|");	
	}
	if(capabilities & GPS_CAPABILITY_MSA){
		printf("GPS_CAPABILITY_MSA|");	
	}
	if(capabilities & GPS_CAPABILITY_SINGLE_SHOT){
		printf("GPS_CAPABILITY_SINGLE_SHOT|");	
	}
	if(capabilities & GPS_CAPABILITY_ON_DEMAND_TIME){
		printf("GPS_CAPABILITY_ON_DEMAND_TIME|");	
	}
	if(capabilities & ULP_CAPABILITY){
		printf("ULP_CAPABILITY|");	
	}
	printf("\n");
}

static void acquire_wakelock(){
	printf("%s\n",__FUNCTION__);
}

static void release_wakelock(){
	printf("%s\n",__FUNCTION__);
}

typedef void *(*pthread_func)(void *);
static pthread_t create_thread(const char* name, void (*start)(void *), void* arg){
	pthread_t thread;
	pthread_attr_t attr;

	pthread_attr_init(&attr);

	/* Unfortunately pthread_create and the callback disagreed on what
	* start function should return.
	*/
	pthread_create(&thread, &attr, (pthread_func)(start), arg);

	return thread;
}

static void request_utc_time(){
	printf("%s\n",__FUNCTION__);
}

static GpsCallbacks gpsCBs = {
	sizeof(GpsCallbacks),
	get_location_callback,
	get_status_callback,
	get_sv_status_callback,
	get_nmea_callback,
	set_capabilities,
	acquire_wakelock,
	release_wakelock,
	create_thread,
	request_utc_time,
};

static bool dead_loop = false;

static void test_stop(int signo) 
{
	printf("%s\n",__FUNCTION__);
	double time_interval = 0;
	if(sGpsInterface){
		sGpsInterface->stop();
		sGpsInterface->cleanup();
		sGpsInterface = NULL;
	}
	time_interval = stop_count();
	printf("test stop!!! using %fms\n" , time_interval);
	dead_loop = false;
	exit(0);
}

int main(int argc, char **argv) {

	struct gps_device_t *gps_device;
	hw_module_t const* module; 
	if (argc == 2){
          if(strcmp(argv[1],"cold")==0)
              g_gps_start_type = cold;
          else if(strcmp(argv[1],"warm")==0)
              g_gps_start_type = warm;
          else 
		      nmea_display_flag = (bool)atoi(argv[1]);
	}
	else if (argc == 3){
		nmea_display_flag = (bool)atoi(argv[1]);
		svInfo_display_flag = (bool)atoi(argv[2]);
	}
	else if (argc == 4)
	{
		nmea_display_flag = (bool)atoi(argv[1]);
		svInfo_display_flag = (bool)atoi(argv[2]);
		gpsotanb_display_flag = (bool)atoi(argv[3]);
		if (gpsotanb_display_flag)
		{
			if (access("/storage/sdcard/GPSOTA", F_OK) < 0)                       
			{
				system("mkdir /storage/sdcard/GPSOTA");
			}
		}
	}
	else{
		printf("Usage: gps_test <nmea display? 1:0> [svInfo display? 1:0]\n");
	}	
	signal(SIGINT, test_stop); //set stop function. ctrl+c

    	if (hw_get_module(GPS_HARDWARE_MODULE_ID, &module) == 0) { 
		module->methods->open(module,GPS_HARDWARE_MODULE_ID,(struct hw_device_t**)&gps_device);//open device via hal
		sGpsInterface = gps_device->get_gps_interface(gps_device);
		if(sGpsInterface){
			start_count();
			sGpsInterface->init(&gpsCBs);
            if(g_gps_start_type == cold)
            {
                sGpsInterface->delete_aiding_data(GPS_DELETE_ALL);
                printf("*** gps_test_cold***\n");
                usleep(1000000*10);
                goto end;
            }
            if(g_gps_start_type == warm)
            {
                sGpsInterface->delete_aiding_data(GPS_DELETE_WARM);
                usleep(1000000*10);
                goto end;
            }
            start_count_gps_positioned();
			sGpsInterface->start(); 
			printf("running!\n");
			dead_loop = true;
		}else{
			printf("no GPS hardware on this device\n");
			dead_loop = false;
		}
	}else{
		printf("can't get GPS HAL module\n");
		dead_loop = false;
	}
	while(dead_loop){
			//keep thread alive.
			/* give it ability to release cpu resource */
			sleep(1);
	}
	end:	   
        if(sGpsInterface){
            sGpsInterface->stop();
            sGpsInterface->cleanup();
            sGpsInterface = NULL;
        }
        exit(0);
}



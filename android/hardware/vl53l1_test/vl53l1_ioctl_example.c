/**
 * IOCTL test
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include "stmvl53l1_if.h"


#ifndef MAX
#define MAX(a,b ) ((a)> (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b ) ((a)< (b) ? (a) : (b))
#endif

int smtvl53l1_start(int fd){
	int rc;
	rc= ioctl(fd, VL53L1_IOCTL_START, NULL);
	if( rc ){
		if( errno == EBUSY){
			//the device is already started
			fprintf(stdout, "already started\n");
			return EBUSY;
		}
	}
	if( rc ){
		fprintf(stderr,"%d %s\n", rc, strerror(errno));
	}
	return rc;
}

int smtvl53l1_stop(int fd){
	int rc;
	rc= ioctl(fd, VL53L1_IOCTL_STOP, NULL);
	if( rc ){
		if( errno == EBUSY ){
			fprintf(stdout,"already stopped\n");
			return EBUSY;
		}
		fprintf(stderr,"%d %s\n", rc, strerror(errno));
	}
	return rc;
}

int stmvl53l1_set_integer_ioctl(int fd, uint32_t param, stmv53l1_parameter_name_e parameter_name)
{
	int rc;

	struct stmvl53l1_parameter params;

	params.is_read = 0;
	params.name = parameter_name;
	params.value = param;

	rc= ioctl(fd, VL53L1_IOCTL_PARAMETER, &params);
	if( rc ){
		if( errno == EBUSY ){
			fprintf(stdout, "ebusy can't set now\n");
			return errno;
		}
		fprintf(stderr, "%d %s\n", rc,strerror(errno));
	}
	return rc;
}

static int stmvl53l1_get_integer_ioctl(int fd, uint32_t *param, stmv53l1_parameter_name_e parameter_name)
{
	int rc;

	struct stmvl53l1_parameter params;

	params.is_read = 1;
	params.name = parameter_name;

	rc= ioctl(fd, VL53L1_IOCTL_PARAMETER, &params);
	if( rc ){
		fprintf(stderr, "%d %s\n", rc,strerror(errno));
	} else{
		rc = params.status;
		if( rc == 0 )
			*param = params.value;
	}

	return rc;
}

static float auto_16x16_to_float(FixPoint1616_t fix)
{
	return fix / 65536.0;
}

static char *auto_detectionmode_2_str(VL53L1_DetectionMode DetectionMode)
{
	switch(DetectionMode) {
		case VL53L1_DETECTION_NORMAL_RUN:
			return "normal";
		case VL53L1_DETECTION_DISTANCE_ONLY:
			return "distance only";
		case VL53L1_DETECTION_RATE_ONLY:
			return "rate only";
		case VL53L1_DETECTION_DISTANCE_AND_RATE:
			return "distance and rate";
		case VL53L1_DETECTION_DISTANCE_OR_RATE:
			return "distance or rate";
	}

	return "unknown";
}

static char *auto_crossmode_2_str(VL53L1_ThresholdMode CrossMode)
{
	switch(CrossMode) {
		case VL53L1_THRESHOLD_CROSSED_LOW:
			return "low";
		case VL53L1_THRESHOLD_CROSSED_HIGH:
			return "high";
		case VL53L1_THRESHOLD_OUT_OF_WINDOW:
			return "out of window";
		case VL53L1_THRESHOLD_IN_WINDOW:
			return "in window";
	}

	return "unknown";
}

static void display_auto_config_raw(struct stmvl53l1_autonomous_config_t *full)
{
	fprintf(stdout, "%d %d %d ", full->pollingTimeInMs, full->config.DetectionMode, full->config.IntrNoTarget);
	fprintf(stdout, "%d %d %d ", full->config.Distance.CrossMode, full->config.Distance.High, full->config.Distance.Low);
	fprintf(stdout, "%d %f %f ", full->config.Rate.CrossMode, auto_16x16_to_float(full->config.Rate.High), auto_16x16_to_float(full->config.Rate.Low));
	fprintf(stdout, "\n");
}

static void display_auto_config_cook(struct stmvl53l1_autonomous_config_t *full)
{
	fprintf(stdout, "- pollingTimeInMs = %d ms\n", full->pollingTimeInMs);
	fprintf(stdout, "- config\n");
	fprintf(stdout, " - DetectionMode = %s\n", auto_detectionmode_2_str(full->config.DetectionMode));
	fprintf(stdout, " - IntrNoTarget = %d\n", full->config.IntrNoTarget);
	fprintf(stdout, " - Distance\n");
	fprintf(stdout, "  - CrossMode = %s\n", auto_crossmode_2_str(full->config.Distance.CrossMode));
	fprintf(stdout, "  - High = %5d mm\n", full->config.Distance.High);
	fprintf(stdout, "  - Low  = %5d mm\n", full->config.Distance.Low);
	fprintf(stdout, " - Rate\n");
	fprintf(stdout, "  - CrossMode = %s\n", auto_crossmode_2_str(full->config.Rate.CrossMode));
	fprintf(stdout, "  - High = %f\n", auto_16x16_to_float(full->config.Rate.High));
	fprintf(stdout, "  - Low  = %f\n", auto_16x16_to_float(full->config.Rate.Low));
}

static int stmvl53l1_get_sd_data_blocking_ioctl(int fd, stmvl531_range_data_t *data)
{
	int rc;

	rc= ioctl(fd, VL53L1_IOCTL_GETDATAS_BLOCKING, data);
	if(rc != 0) {
		fprintf(stderr,"%d %s\n", rc,strerror(errno));
	}

	return rc;
}

static int stmvl53l1_get_ranging_data_blocking_ioctl(int fd, VL53L1_MultiRangingData_t *data)
{
	int rc;
	rc= ioctl(fd, VL53L1_IOCTL_MZ_DATA_BLOCKING, data);
	if( rc != 0){
		fprintf(stderr,"%d %s\n", rc,strerror(errno));
	}
	return rc;
}



void print_1xdata(FILE * fi, stmvl531_range_data_t *range_data){
	fprintf(fi, "cnt %4d st %d\t"
		"d=%4dmm\t"
		"qual=%3d\t"
		"spdcnt=%d\t"
		"sigma=%3d.%02d\t"
		"rate=%5d.%02d/%5d.%02d"
		"\n",
		(int)range_data->StreamCount, (int)range_data->RangeStatus,
		(int)range_data->RangeMilliMeter,
		(int)range_data->RangeQualityLevel,
		(int)range_data->EffectiveSpadRtnCount / 256,
		(int)range_data->SigmaMilliMeter >> 16,
		(int)((range_data->SigmaMilliMeter & 0xffff) * 100) >> 16,
		(int)range_data->SignalRateRtnMegaCps >> 16,
		(int)((range_data->SignalRateRtnMegaCps & 0xffff) * 100) >> 16,
		(int)range_data->AmbientRateRtnMegaCps >> 16,
		(int)((range_data->AmbientRateRtnMegaCps & 0xffff) * 100) >> 16
		);
}

void print_target1xdata(FILE * fi, VL53L1_TargetRangeData_t *range_data){
	if (range_data->RangeStatus == VL53L1_RANGESTATUS_NONE)
		fprintf(fi, "*NOTARGET* ");
	fprintf(fi, "st %4d\t"
		"d=%4dmm\t"
		"min/max=%4d/%4d\t"
		"qual=%3d ER=%1d\t"
		"sigma=%3d.%02d\t"
		"rate=%5d.%02d/%5d.%02d"
		"\n",
		(int)range_data->RangeStatus,
		(int)range_data->RangeMilliMeter,
		(int)range_data->RangeMinMilliMeter,
		(int)range_data->RangeMaxMilliMeter,
		(int)range_data->RangeQualityLevel,
		(int)range_data->ExtendedRange,
		(int)range_data->SigmaMilliMeter >> 16,
		(int)((range_data->SigmaMilliMeter & 0xffff) * 100) >> 16,
		(int)range_data->SignalRateRtnMegaCps >> 16,
		(int)((range_data->SignalRateRtnMegaCps & 0xffff) * 100) >> 16,
		(int)range_data->AmbientRateRtnMegaCps >> 16,
		(int)((range_data->AmbientRateRtnMegaCps & 0xffff) * 100) >> 16
		);
}


int main(int argc, char *argv[])
{
	int fd;
	int rc;
	uint32_t mode;
	uint32_t timingbudget;
	struct stmvl53l1_autonomous_config_t SDconfig;
	
	stmvl531_range_data_t sd_data;
	VL53L1_MultiRangingData_t ranging_data;
	int i;
	int n_loops = 10; /* nb of measurements to catch */
	int max_obj, obj;

	fd = open("/dev/" VL53L1_MISC_DEV_NAME ,O_RDWR );
	if (fd <= 0) {
		fprintf(stderr,"Error open %s device: %s\n", VL53L1_MISC_DEV_NAME, strerror(errno));
		return -1;
	}

	fprintf(stdout,"%s device opened\n", VL53L1_MISC_DEV_NAME);

	smtvl53l1_stop(fd);

	rc = stmvl53l1_get_integer_ioctl(fd, &mode, VL53L1_DEVICEMODE_PAR);
	if (rc)
		fprintf(stderr,"Error reading VL53L1_DEVICEMODE_PAR parameter rc %d\n", rc);
	else
		fprintf(stdout,"Current preset mode %d\n", mode);

	rc = stmvl53l1_get_integer_ioctl(fd, &timingbudget, VL53L1_TIMINGBUDGET_PAR);
	if (rc)
		fprintf(stderr,"Error reading VL53L1_TIMINGBUDGET_PAR parameter rc %d\n", rc);
	else
		fprintf(stdout,"Current timing budget %d\n", timingbudget);
	
	/**********************************************************/
	/* Set histogram ranging mode with 33 ms of timing budget */
	/**********************************************************/
	mode = (uint32_t) VL53L1_PRESETMODE_RANGING;
	rc = stmvl53l1_set_integer_ioctl(fd, mode, VL53L1_DEVICEMODE_PAR);
	if (rc)
		fprintf(stderr,"Error writing VL53L1_DEVICEMODE_PAR parameter rc %d\n", rc);
	else
		fprintf(stdout,"New preset mode %d\n", mode);

	timingbudget = 33000;
	rc = stmvl53l1_set_integer_ioctl(fd, timingbudget, VL53L1_TIMINGBUDGET_PAR);
	if (rc)
		fprintf(stderr,"Error writing VL53L1_TIMINGBUDGET_PAR parameter rc %d\n", rc);
	else
		fprintf(stdout,"New timing budget %d\n", timingbudget);

	/* read back settings to check ... */
	rc = stmvl53l1_get_integer_ioctl(fd, &mode, VL53L1_DEVICEMODE_PAR);
	rc |= stmvl53l1_get_integer_ioctl(fd, &timingbudget, VL53L1_TIMINGBUDGET_PAR);
	if (rc)
		fprintf(stderr,"Error reading parameters rc %d\n", rc);
	else
		fprintf(stdout,"read back New timing budget %d new mode %d\n", timingbudget, mode);
	
	/* start the device and get multi ranging data (as histogram ranging mode is set for now) */
	fprintf(stdout, "Start ranging mode %d timing budget %d\n", mode, timingbudget);
	smtvl53l1_start(fd);
	for(i=0; i< n_loops; i++) {
		rc = stmvl53l1_get_ranging_data_blocking_ioctl(fd, &ranging_data);
		if(rc != 0) {
			fprintf(stderr,"loop #%d fail to get data code=%d\n", i, rc);
			break;
		}
		fprintf(stdout,"StreamCount(%d), SpadCount(%.2f)\n",
			ranging_data.StreamCount,
			ranging_data.EffectiveSpadRtnCount/256.0);
		max_obj = MIN(VL53L1_MAX_RANGE_RESULTS, ranging_data.NumberOfObjectsFound);
		// allow to report ambient rate even when no target detected
		max_obj = MAX(max_obj, 1);
		for(obj=0; obj < max_obj; obj++) {
			fprintf(stdout, " - [%d]\t", obj);
			print_target1xdata(stdout, &ranging_data.RangeData[obj]);
		}
	}
	/* stop the device */
	smtvl53l1_stop(fd);

	
	/**********************************************************/
	/* Set SD mode autonomous low power ranging with          */
	/*     30 ms of timing budget                             */
	/*     100 ms of inter measurement period                 */
	/*     Generate interrupt when distance threshold is met  */
	/*     distance threshold when target is < 300 mm         */
	/**********************************************************/
	mode = (uint32_t) VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS;
	rc = stmvl53l1_set_integer_ioctl(fd, mode, VL53L1_DEVICEMODE_PAR);
	if (rc)
		fprintf(stderr,"Error writing VL53L1_DEVICEMODE_PAR parameter rc %d\n", rc);
	else
		fprintf(stdout,"New preset mode %d\n", mode);

	timingbudget = 30000;
	rc = stmvl53l1_set_integer_ioctl(fd, timingbudget, VL53L1_TIMINGBUDGET_PAR);
	if (rc)
		fprintf(stderr,"Error writing VL53L1_TIMINGBUDGET_PAR parameter rc %d\n", rc);
	else
		fprintf(stdout,"New timing budget %d\n", timingbudget);

	SDconfig.is_read = 1;
	rc = ioctl(fd, VL53L1_IOCTL_AUTONOMOUS_CONFIG, &SDconfig);
	if( rc ){
		fprintf(stderr,"Error reading VL53L1_IOCTL_AUTONOMOUS_CONFIG parameter rc %d\n", rc);
	}
	else{
		fprintf(stdout,"Current autonomous timing configuration :\n");
		display_auto_config_raw(&SDconfig);
		display_auto_config_cook(&SDconfig);
	}

	SDconfig.is_read = 0;
	SDconfig.pollingTimeInMs = 100;
	SDconfig.config.DetectionMode = 1; /* VL53L1_DETECTION_DISTANCE_ONLY */
	SDconfig.config.Distance.CrossMode = 0; /* VL53L1_THRESHOLD_CROSSED_LOW */
	SDconfig.config.Distance.Low = 300; 
	rc = ioctl(fd, VL53L1_IOCTL_AUTONOMOUS_CONFIG, &SDconfig);
	if( rc ){
		fprintf(stderr,"Error writing VL53L1_IOCTL_AUTONOMOUS_CONFIG parameter rc %d\n", rc);
	}
	else{
		fprintf(stdout,"New autonomous timing configuration :\n");
		display_auto_config_raw(&SDconfig);
		display_auto_config_cook(&SDconfig);
	}
	/* read back settings to check ... */
	rc = stmvl53l1_get_integer_ioctl(fd, &mode, VL53L1_DEVICEMODE_PAR);
	rc |= stmvl53l1_get_integer_ioctl(fd, &timingbudget, VL53L1_TIMINGBUDGET_PAR);
	if (rc)
		fprintf(stderr,"Error reading parameters rc %d\n", rc);
	else
		fprintf(stdout,"read back New timing budget %d new mode %d\n", timingbudget, mode);
	
	/* start the device and get SD ranging data (as VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS mode is set for now) */
	fprintf(stdout, "Start ranging mode %d timing budget %d\n", mode, timingbudget);
	smtvl53l1_start(fd);
	for(i=0; i< (n_loops); i++) {
		rc = stmvl53l1_get_sd_data_blocking_ioctl(fd, &sd_data);
		if(rc != 0) {
			fprintf(stderr,"loop #%d fail to get data code=%d\n", i, rc);
			break;
		}
		print_1xdata(stdout, &sd_data);
	}
	/* stop the device */
	smtvl53l1_stop(fd);
	close(fd);
	fprintf(stdout,"%s device closed\n", VL53L1_MISC_DEV_NAME);

	return 0;
}



/*mavlink_msg_global_position_int_send
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>
#ifdef HAL_IS_REGISTERED_FLIGHT_MODULE
// scaling factor from 1e-7 degrees to meters at equator
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FAC 0.011131884502145034f

#include <AP_Common/Location.h>
#include "AP_Security.h"
#include <AP_Math/crc.h>
#include <AP_RTC/AP_RTC.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <npnt_internal.h>

#if HAL_OS_POSIX_IO
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#endif

#if HAL_OS_FATFS_IO
#include <stdio.h>
#endif

#define PA_LOG_FILE_NAME "/APM/log_PA.json"

extern const AP_HAL::HAL& hal;

//Setup Security at the initialisation step
AP_Security::AP_Security():permission_granted(false), _logging(false), _init_status(false)
{
	if (_singleton != nullptr) {
		gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "Permission Artifact verification Successful\n");
		AP_HAL::panic("Too many security modules");
		return;
	}
	_singleton = this;
}

void AP_Security::init() {
	//Initialise NPNT Handler
	npnt_init_handle(&npnt_handle);
	if (_check_npnt_permission()) {
		_init_status = true;
		_pa_state = PA_VERTIFACTION_SUCCESS;
		gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "Permission Artifact verification Successful\n");
	}
	else{
		_pa_state = PA_VERFICATION_FAILED;
		gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "Permission Artifact verification failed\n");
	}
}

void AP_Security::pre_arm_check(){

	if(permission_granted){
		return;
	}
	else if(!_init_status){
		return;
	}

	if(_check_fence_and_time()){
		//create json log file
		_log_fd = ::open("/APM/log_PA.json", O_WRONLY|O_TRUNC|O_CREAT);
		if (_log_fd < 0) {
			AP_HAL::panic("Failed to create JSON log file");
			return;
		}
		gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "Permission Granted\n");
		permission_granted = true;
		::close(_log_fd);
	}
	else{
		gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "Permission Artifact verification failed\n");
		permission_granted = false;
	}
}

//Load permissionArtefacts
bool AP_Security::_check_npnt_permission()
{
	struct stat st;
	if (::stat(AP_NPNT_PERMART_FILE, &st) != 0) {
		printf("Unable to find Permission Artifact\n", strerror(errno));
		return false;
	}

	int fd = ::open(AP_NPNT_PERMART_FILE, O_RDONLY);

	uint8_t* permart = (uint8_t*)malloc(st.st_size + 1);

	if (!permart) {
		::close(fd);
		free(permart);
		return false;
	}

	uint32_t permart_len;
	permart_len = ::read(fd, permart, st.st_size);

	if ((off_t)permart_len != st.st_size) {
		::close(fd);
		free(permart);
		return false;
	}

	//append with NULL character
	permart[st.st_size] = '\0';
	permart_len++;
	int ret = npnt_set_permart(&npnt_handle, permart, permart_len, 0);
	if ( ret < 0) {
		goto fail;
	}
	else{
		//set geo fence
		// set parameter
		enum ap_var_type var_type;

		AP_Param *vp;
		char key[AP_MAX_NAME_SIZE+1];
		strncpy(key, "FENCE_TOTAL", AP_MAX_NAME_SIZE);
		key[AP_MAX_NAME_SIZE] = 0;

		// find existing param so we can get the old value
		vp = AP_Param::find(key, &var_type);
		if (vp == nullptr) {
			return false;
		}

		float old_value = vp->cast_to_float(var_type);

		// set the value
		vp->set_float(npnt_handle.fence.nverts+1, var_type);

		if((npnt_handle.fence.nverts+1) == old_value){
			vp->save(0);
		}
		else{
			vp->save(1);
		}

		AP_Logger *logger = AP_Logger::get_singleton();
		if (logger != nullptr) {
			logger->Write_Parameter(key, vp->cast_to_float(var_type));
		}

		AP_AHRS &ahrs = AP::ahrs();
		ahrs.get_position(_global_position_current_loc);

		AC_Fence *fence = AP::fence();
		if (fence == nullptr) {
			hal.console->printf("fence set failed\n");
			goto fail;
		}

		float return_lat = 0;
		float return_lon = 0;
		hal.console->printf("total_fence_points: %d\n", npnt_handle.fence.nverts);

		for(uint8_t i = 0; i < npnt_handle.fence.nverts; i++){
			if(!fence->set_geo_fence_from_PA(npnt_handle.fence.vertlat[i], npnt_handle.fence.vertlon[i], i+1)){
				hal.console->printf("Fence set failed\n");
				goto fail;
			}
			else{
				if(i != npnt_handle.fence.nverts-1){
					geo_fence[i].lat = npnt_handle.fence.vertlat[i]*1e+7;
					geo_fence[i].lon = npnt_handle.fence.vertlon[i]*1e+7;
					return_lat += npnt_handle.fence.vertlat[i];
					return_lon += npnt_handle.fence.vertlon[i];
					location_diff(geo_fence[0], geo_fence[i]);
					geo_fence_local[i] = diff_coord;
				}
			}
		}

		if(!fence->set_geo_fence_from_PA(return_lat/(npnt_handle.fence.nverts-1), return_lon/(npnt_handle.fence.nverts-1), 0)){ //set the zeroth point as current location
			hal.console->printf("Fence set failed\n");
			goto fail;
		}
		goto success;
	}

	fail:
		::close(fd);
		free(permart);
		hal.console->printf("PA set failed\n");
		return false;

	success:
		::close(fd);
		free(permart);
		hal.console->printf("PA set successful\n");
		return true;
}


//Check for geo-fence and time limits
bool AP_Security::_check_fence_and_time(){

	AP_AHRS &ahrs = AP::ahrs();
	ahrs.get_position(_global_position_current_loc);

	struct coordinate curr_loc;
	curr_loc.lat = _global_position_current_loc.lat;
	curr_loc.lon = _global_position_current_loc.lng;


	location_diff(geo_fence[0], curr_loc);
	struct local_coord curr_loc_local= diff_coord;

	bool out = _polygon_outside(geo_fence_local, curr_loc_local, npnt_handle.fence.nverts-1);
	if(out){
		hal.console->printf("polygon outside\n");
		_pa_state = INCORRECT_GEO_FENCE;
		return false;
	}

	uint64_t time_unix = 0;
	AP::rtc().get_utc_usec(time_unix); // may fail, leaving time_unix at 0

	time_t seconds = (time_t)((time_unix/1000000)+5*3600+30*60); //IST offset 5:30
	struct tm tm = *localtime(&seconds);
	tm.tm_year += 1900;
	tm.tm_mon += 1;

	hal.console->printf("%d %d %d %d %d %d\n", tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	if(!permission_granted){
		_log_time = tm;
	}
	time_t t_now = mktime(&tm);
	time_t t_start = mktime(&npnt_handle.params.flightStartTime);
	time_t t_end = mktime(&npnt_handle.params.flightEndTime);
	if(!(difftime(t_now, t_start)>0 && difftime(t_end, t_now)>0)){
		hal.console->printf("Time limit incorrect\n");
		_pa_state = PA_INCORRECT_TIME_LIMIT;
		return false;
	}

	_pa_state = PERMISSION_GRANTED;
	return true;
}

float AP_Security::longitude_scale(struct coordinate loc)
{
	float scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
	if(scale < 0.01f){
		return 0.01f;
	}
	else if(scale > 1.0f){
		return 1.0f;
	}
	else{
		return scale;
	}
}

void AP_Security::location_diff(struct coordinate loc1, struct coordinate loc2){
	diff_coord.x = (loc2.lat - loc1.lat) * LOCATION_SCALING_FAC;
	diff_coord.y = (loc2.lon - loc1.lon) * LOCATION_SCALING_FAC;
}


bool AP_Security::_polygon_outside(struct local_coord* V, struct local_coord P, int n){
	int i,j;
	bool outside = true;
	for (i = 0, j = n-1; i < n; j = i++) {
		if ((V[i].y > P.y) == (V[j].y > P.y)) {
			continue;
		}
		const int32_t dx1 = P.x - V[i].x;
		const int32_t dx2 = V[j].x - V[i].x;
		const int32_t dy1 = P.y - V[i].y;
		const int32_t dy2 = V[j].y - V[i].y;
		const int8_t dx1s = (dx1 < 0) ? -1 : 1;
		const int8_t dx2s = (dx2 < 0) ? -1 : 1;
		const int8_t dy1s = (dy1 < 0) ? -1 : 1;
		const int8_t dy2s = (dy2 < 0) ? -1 : 1;
		const int8_t m1 = dx1s * dy2s;
		const int8_t m2 = dx2s * dy1s;
		// we avoid the 64 bit multiplies if we can based on sign checks.
		if (dy2 < 0) {
			if (m1 > m2) {
				outside = !outside;
			} else if (m1 < m2) {
				continue;
			} else if ( dx1 * (int64_t)dy2 > dx2 * (int64_t)dy1 ) {
				outside = !outside;
			}
		} else {
			if (m1 < m2) {
				outside = !outside;
			} else if (m1 > m2) {
				continue;
			} else if ( dx1 * (int64_t)dy2 < dx2 * (int64_t)dy1 ) {
				outside = !outside;
			}
		}
	}
	return outside;
}

void AP_Security::_sign_json_log(){

	_pa_state = SIGNING_LOG;
	struct stat _st;
	if (stat(AP_NPNT_LOG_FILE, &_st) != 0) {
		printf("Unable to find Log File\n", strerror(errno));
		return;
	}
	int fd = open(AP_NPNT_LOG_FILE, O_RDONLY);
	uint8_t* log = (uint8_t*)malloc(_st.st_size + 1);
	if (!log || fd<=0) {
		::close(fd);
		free(log);
		return;
	}
	uint32_t log_len;
	log_len = read(fd, log, _st.st_size);
	log_size = _st.st_size;
	if ((off_t)log_len != _st.st_size) {
		::close(fd);
		free(log);
		return;
	}

	if(sign_log_data(&npnt_handle, log, _st.st_size, digest_value) != 0){
		return;
	}

	uint8_t signature[256];
	__sign_hash((uint8_t*)digest_value, 32, signature);
	::close(fd);
	free(log);
}

void AP_Security::send_mavlink_status(mavlink_channel_t chan){

	 mavlink_msg_radio_send(chan, _pa_state,0,0,0,0,0,0);
}

void AP_Security::_save_signed_log(uint8_t* sig, uint16_t out_len){

	struct stat _st;
	if (stat(AP_NPNT_LOG_FILE, &_st) != 0) {
		printf("Unable to find Log File\n", strerror(errno));
		return;
	}
	int fd = open(AP_NPNT_LOG_FILE, O_RDONLY);
	char* log_char = (char*)malloc(_st.st_size + 1);
	if (!log_char || fd<=0) {
		::close(fd);
		free(log_char);
		return;
	}
	uint32_t log_len;
	log_len = read(fd, log_char, _st.st_size);
	log_size = _st.st_size;
	if ((off_t)log_len != _st.st_size) {
		::close(fd);
		free(log_char);
		return;
	}
	::close(fd);

	char buf[100];
	snprintf(buf, sizeof buf, "/APM/%d_%d_%d_%d_%d_%d_signed.json",_log_time.tm_year, _log_time.tm_mon, _log_time.tm_mday, _log_time.tm_hour, _log_time.tm_min, _log_time.tm_sec);
	hal.console->printf("%s\n",buf);

	int json_fd = ::open(buf, O_WRONLY|O_TRUNC|O_CREAT);
	if (json_fd < 0) {
		return;
	}
	//TODO: get the pa ref from the PA xml file
	int success = ::write(json_fd, "{\"PermissionArtifact\": \"12345678\",\n\"FlightLog\": ", strlen("{\"PermissionArtifact\": \"12345678\",\n\"FlightLog\": "));
	if(success>0){
		success = ::write(json_fd, log_char, _st.st_size);
	}
	if(success>0){
		success = ::write(json_fd, ",\n\"Signature\": \"", strlen(",\n\"Signature\": \""));
	}
	if(success>0){
		char * sig_char = (char *)malloc(out_len);
		int _out_len = 0;
		for(int i = 0; i < out_len; i++){
			if(sig[i] == '\n'){ //remove new line char
				continue;
			}
			sig_char[_out_len] = (char)sig[i];
			_out_len++;
		}
		success = ::write(json_fd, sig_char, _out_len);
	}
	if(success>0){
		success = ::write(json_fd, "\"}", strlen("\"}"));
	}
	if(success>0){
		hal.key_mgr->log_signed = false;
	}
	_pa_state = LOG_SIGNED;
	free(log_char);
	::close(json_fd);
}

//update loop called every 1s for logging
void AP_Security::update(bool arm_flag){

	if(hal.key_mgr->log_signed){
		uint16_t out_len;
		uint8_t* sig = base64_encode(hal.key_mgr->signed_log, 256, &out_len);
		hal.console->printf("signature generated: %d\n", out_len);
		_save_signed_log(sig, out_len);
		return;
	}

	if(!permission_granted)
		return;

	if((!_logging && arm_flag)|| !_check_fence_and_time() || (!arm_flag && _logging)){

		_log_fd = ::open("/APM/log_PA.json", O_WRONLY|O_APPEND|O_CREAT);
		if (_log_fd < 0) {
			AP_HAL::panic("Failed to open JSON log file");
			return;
		}

		uint64_t time_unix = 0;
		AP::rtc().get_utc_usec(time_unix);

		AP_AHRS &ahrs = AP::ahrs();
		ahrs.get_position(_global_position_current_loc);

		char buf[200];
		int ret = -1;

		if(!_logging)
			ret = snprintf(buf, sizeof buf, "[{\"Latitude\": \"%f\", \"TimeStamp\": \"%llu\", \"Altitude\": \"%f\", \"Longitude\": \"%f\"}, ",  _global_position_current_loc.lat*1e-7, time_unix, _global_position_current_loc.alt, _global_position_current_loc.lng*1e-7);
		else if(arm_flag)
			ret = snprintf(buf, sizeof buf, "{\"Latitude\": \"%f\", \"TimeStamp\": \"%llu\", \"Altitude\": \"%f\", \"Longitude\": \"%f\"}, ",  _global_position_current_loc.lat*1e-7, time_unix, _global_position_current_loc.alt, _global_position_current_loc.lng*1e-7);
		else
			ret = snprintf(buf, sizeof buf, "{\"Latitude\": \"%f\", \"TimeStamp\": \"%llu\", \"Altitude\": \"%f\", \"Longitude\": \"%f\"}]",  _global_position_current_loc.lat*1e-7, time_unix, _global_position_current_loc.alt, _global_position_current_loc.lng*1e-7);

		if(ret < 0){
			return;
		}
		else if(ret >= (int)sizeof(buf)){
			return;
		}

		int success = ::write(_log_fd, buf, strlen(buf));
		::close(_log_fd);

		if(success<=0){
			return;
		}

		if(!arm_flag){
			_sign_json_log();
			permission_granted = false;
			_logging = false;
		}

		if(!_logging){
			_logging = true;
		}
	}

}

// singleton instance
AP_Security *AP_Security::_singleton;

namespace AP {

AP_Security &security()
{
	return *AP_Security::get_singleton();
}

}

#endif //HAL_IS_REGISTERED_FLIGHT_MODULE

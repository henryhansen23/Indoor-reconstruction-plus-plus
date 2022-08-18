/* ***********************************************************
 * This file was automatically generated on 2022-05-11.      *
 *                                                           *
 * C/C++ Bindings Version 2.1.33                             *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generators git repository on tinkerforge.com       *
 *************************************************************/


#define IPCON_EXPOSE_INTERNALS

#include "bricklet_gps_v2.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef void (*PulsePerSecond_CallbackFunction)(void *user_data);

typedef void (*Coordinates_CallbackFunction)(uint32_t latitude, char ns, uint32_t longitude, char ew, void *user_data);

typedef void (*Status_CallbackFunction)(bool has_fix, uint8_t satellites_view, void *user_data);

typedef void (*Altitude_CallbackFunction)(int32_t altitude, int32_t geoidal_separation, void *user_data);

typedef void (*Motion_CallbackFunction)(uint32_t course, uint32_t speed, void *user_data);

typedef void (*DateTime_CallbackFunction)(uint32_t date, uint32_t time, void *user_data);

#if defined _MSC_VER || defined __BORLANDC__
	#pragma pack(push)
	#pragma pack(1)
	#define ATTRIBUTE_PACKED
#elif defined __GNUC__
	#ifdef _WIN32
		// workaround struct packing bug in GCC 4.7 on Windows
		// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=52991
		#define ATTRIBUTE_PACKED __attribute__((gcc_struct, packed))
	#else
		#define ATTRIBUTE_PACKED __attribute__((packed))
	#endif
#else
	#error unknown compiler, do not know how to enable struct packing
#endif

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetCoordinates_Request;

typedef struct {
	PacketHeader header;
	uint32_t latitude;
	char ns;
	uint32_t longitude;
	char ew;
} ATTRIBUTE_PACKED GetCoordinates_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetStatus_Request;

typedef struct {
	PacketHeader header;
	uint8_t has_fix;
	uint8_t satellites_view;
} ATTRIBUTE_PACKED GetStatus_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAltitude_Request;

typedef struct {
	PacketHeader header;
	int32_t altitude;
	int32_t geoidal_separation;
} ATTRIBUTE_PACKED GetAltitude_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetMotion_Request;

typedef struct {
	PacketHeader header;
	uint32_t course;
	uint32_t speed;
} ATTRIBUTE_PACKED GetMotion_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetDateTime_Request;

typedef struct {
	PacketHeader header;
	uint32_t date;
	uint32_t time;
} ATTRIBUTE_PACKED GetDateTime_Response;

typedef struct {
	PacketHeader header;
	uint8_t restart_type;
} ATTRIBUTE_PACKED Restart_Request;

typedef struct {
	PacketHeader header;
	uint8_t satellite_system;
} ATTRIBUTE_PACKED GetSatelliteSystemStatusLowLevel_Request;

typedef struct {
	PacketHeader header;
	uint8_t satellite_numbers_length;
	uint8_t satellite_numbers_data[12];
	uint8_t fix;
	uint16_t pdop;
	uint16_t hdop;
	uint16_t vdop;
} ATTRIBUTE_PACKED GetSatelliteSystemStatusLowLevel_Response;

typedef struct {
	PacketHeader header;
	uint8_t satellite_system;
	uint8_t satellite_number;
} ATTRIBUTE_PACKED GetSatelliteStatus_Request;

typedef struct {
	PacketHeader header;
	int16_t elevation;
	int16_t azimuth;
	int16_t snr;
} ATTRIBUTE_PACKED GetSatelliteStatus_Response;

typedef struct {
	PacketHeader header;
	uint8_t config;
} ATTRIBUTE_PACKED SetFixLEDConfig_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetFixLEDConfig_Request;

typedef struct {
	PacketHeader header;
	uint8_t config;
} ATTRIBUTE_PACKED GetFixLEDConfig_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetCoordinatesCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetCoordinatesCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetCoordinatesCallbackPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetStatusCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetStatusCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetStatusCallbackPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetAltitudeCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAltitudeCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetAltitudeCallbackPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetMotionCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetMotionCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetMotionCallbackPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetDateTimeCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetDateTimeCallbackPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetDateTimeCallbackPeriod_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED PulsePerSecond_Callback;

typedef struct {
	PacketHeader header;
	uint32_t latitude;
	char ns;
	uint32_t longitude;
	char ew;
} ATTRIBUTE_PACKED Coordinates_Callback;

typedef struct {
	PacketHeader header;
	uint8_t has_fix;
	uint8_t satellites_view;
} ATTRIBUTE_PACKED Status_Callback;

typedef struct {
	PacketHeader header;
	int32_t altitude;
	int32_t geoidal_separation;
} ATTRIBUTE_PACKED Altitude_Callback;

typedef struct {
	PacketHeader header;
	uint32_t course;
	uint32_t speed;
} ATTRIBUTE_PACKED Motion_Callback;

typedef struct {
	PacketHeader header;
	uint32_t date;
	uint32_t time;
} ATTRIBUTE_PACKED DateTime_Callback;

typedef struct {
	PacketHeader header;
	uint8_t sbas_config;
} ATTRIBUTE_PACKED SetSBASConfig_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetSBASConfig_Request;

typedef struct {
	PacketHeader header;
	uint8_t sbas_config;
} ATTRIBUTE_PACKED GetSBASConfig_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetSPITFPErrorCount_Request;

typedef struct {
	PacketHeader header;
	uint32_t error_count_ack_checksum;
	uint32_t error_count_message_checksum;
	uint32_t error_count_frame;
	uint32_t error_count_overflow;
} ATTRIBUTE_PACKED GetSPITFPErrorCount_Response;

typedef struct {
	PacketHeader header;
	uint8_t mode;
} ATTRIBUTE_PACKED SetBootloaderMode_Request;

typedef struct {
	PacketHeader header;
	uint8_t status;
} ATTRIBUTE_PACKED SetBootloaderMode_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetBootloaderMode_Request;

typedef struct {
	PacketHeader header;
	uint8_t mode;
} ATTRIBUTE_PACKED GetBootloaderMode_Response;

typedef struct {
	PacketHeader header;
	uint32_t pointer;
} ATTRIBUTE_PACKED SetWriteFirmwarePointer_Request;

typedef struct {
	PacketHeader header;
	uint8_t data[64];
} ATTRIBUTE_PACKED WriteFirmware_Request;

typedef struct {
	PacketHeader header;
	uint8_t status;
} ATTRIBUTE_PACKED WriteFirmware_Response;

typedef struct {
	PacketHeader header;
	uint8_t config;
} ATTRIBUTE_PACKED SetStatusLEDConfig_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetStatusLEDConfig_Request;

typedef struct {
	PacketHeader header;
	uint8_t config;
} ATTRIBUTE_PACKED GetStatusLEDConfig_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetChipTemperature_Request;

typedef struct {
	PacketHeader header;
	int16_t temperature;
} ATTRIBUTE_PACKED GetChipTemperature_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED Reset_Request;

typedef struct {
	PacketHeader header;
	uint32_t uid;
} ATTRIBUTE_PACKED WriteUID_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED ReadUID_Request;

typedef struct {
	PacketHeader header;
	uint32_t uid;
} ATTRIBUTE_PACKED ReadUID_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetIdentity_Request;

typedef struct {
	PacketHeader header;
	char uid[8];
	char connected_uid[8];
	char position;
	uint8_t hardware_version[3];
	uint8_t firmware_version[3];
	uint16_t device_identifier;
} ATTRIBUTE_PACKED GetIdentity_Response;

#if defined _MSC_VER || defined __BORLANDC__
	#pragma pack(pop)
#endif
#undef ATTRIBUTE_PACKED

static void gps_v2_callback_wrapper_pulse_per_second(DevicePrivate *device_p, Packet *packet) {
	PulsePerSecond_CallbackFunction callback_function;
	void *user_data;
	PulsePerSecond_Callback *callback;

	if (packet->header.length != sizeof(PulsePerSecond_Callback)) {
		return; // silently ignoring callback with wrong length
	}

	callback_function = (PulsePerSecond_CallbackFunction)device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_PULSE_PER_SECOND];
	user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_PULSE_PER_SECOND];
	callback = (PulsePerSecond_Callback *)packet;
	(void)callback; // avoid unused variable warning

	if (callback_function == NULL) {
		return;
	}

	callback_function(user_data);
}

static void gps_v2_callback_wrapper_coordinates(DevicePrivate *device_p, Packet *packet) {
	Coordinates_CallbackFunction callback_function;
	void *user_data;
	Coordinates_Callback *callback;

	if (packet->header.length != sizeof(Coordinates_Callback)) {
		return; // silently ignoring callback with wrong length
	}

	callback_function = (Coordinates_CallbackFunction)device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_COORDINATES];
	user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_COORDINATES];
	callback = (Coordinates_Callback *)packet;
	(void)callback; // avoid unused variable warning

	if (callback_function == NULL) {
		return;
	}

	callback->latitude = leconvert_uint32_from(callback->latitude);
	callback->longitude = leconvert_uint32_from(callback->longitude);

	callback_function(callback->latitude, callback->ns, callback->longitude, callback->ew, user_data);
}

static void gps_v2_callback_wrapper_status(DevicePrivate *device_p, Packet *packet) {
	Status_CallbackFunction callback_function;
	void *user_data;
	Status_Callback *callback;
	bool unpacked_has_fix;

	if (packet->header.length != sizeof(Status_Callback)) {
		return; // silently ignoring callback with wrong length
	}

	callback_function = (Status_CallbackFunction)device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_STATUS];
	user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_STATUS];
	callback = (Status_Callback *)packet;
	(void)callback; // avoid unused variable warning

	if (callback_function == NULL) {
		return;
	}
	unpacked_has_fix = callback->has_fix != 0;

	callback_function(unpacked_has_fix, callback->satellites_view, user_data);
}

static void gps_v2_callback_wrapper_altitude(DevicePrivate *device_p, Packet *packet) {
	Altitude_CallbackFunction callback_function;
	void *user_data;
	Altitude_Callback *callback;

	if (packet->header.length != sizeof(Altitude_Callback)) {
		return; // silently ignoring callback with wrong length
	}

	callback_function = (Altitude_CallbackFunction)device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_ALTITUDE];
	user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_ALTITUDE];
	callback = (Altitude_Callback *)packet;
	(void)callback; // avoid unused variable warning

	if (callback_function == NULL) {
		return;
	}

	callback->altitude = leconvert_int32_from(callback->altitude);
	callback->geoidal_separation = leconvert_int32_from(callback->geoidal_separation);

	callback_function(callback->altitude, callback->geoidal_separation, user_data);
}

static void gps_v2_callback_wrapper_motion(DevicePrivate *device_p, Packet *packet) {
	Motion_CallbackFunction callback_function;
	void *user_data;
	Motion_Callback *callback;

	if (packet->header.length != sizeof(Motion_Callback)) {
		return; // silently ignoring callback with wrong length
	}

	callback_function = (Motion_CallbackFunction)device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_MOTION];
	user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_MOTION];
	callback = (Motion_Callback *)packet;
	(void)callback; // avoid unused variable warning

	if (callback_function == NULL) {
		return;
	}

	callback->course = leconvert_uint32_from(callback->course);
	callback->speed = leconvert_uint32_from(callback->speed);

	callback_function(callback->course, callback->speed, user_data);
}

static void gps_v2_callback_wrapper_date_time(DevicePrivate *device_p, Packet *packet) {
	DateTime_CallbackFunction callback_function;
	void *user_data;
	DateTime_Callback *callback;

	if (packet->header.length != sizeof(DateTime_Callback)) {
		return; // silently ignoring callback with wrong length
	}

	callback_function = (DateTime_CallbackFunction)device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_DATE_TIME];
	user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + GPS_V2_CALLBACK_DATE_TIME];
	callback = (DateTime_Callback *)packet;
	(void)callback; // avoid unused variable warning

	if (callback_function == NULL) {
		return;
	}

	callback->date = leconvert_uint32_from(callback->date);
	callback->time = leconvert_uint32_from(callback->time);

	callback_function(callback->date, callback->time, user_data);
}

void gps_v2_create(GPSV2 *gps_v2, const char *uid, IPConnection *ipcon) {
	IPConnectionPrivate *ipcon_p = ipcon->p;
	DevicePrivate *device_p;

	device_create(gps_v2, uid, ipcon_p, 2, 0, 1, GPS_V2_DEVICE_IDENTIFIER);

	device_p = gps_v2->p;

	device_p->response_expected[GPS_V2_FUNCTION_GET_COORDINATES] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_STATUS] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_ALTITUDE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_MOTION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_DATE_TIME] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_RESTART] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_SATELLITE_SYSTEM_STATUS_LOW_LEVEL] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_SATELLITE_STATUS] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_FIX_LED_CONFIG] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_FIX_LED_CONFIG] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_COORDINATES_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_COORDINATES_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_STATUS_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_STATUS_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_ALTITUDE_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_ALTITUDE_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_MOTION_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_MOTION_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_DATE_TIME_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_DATE_TIME_CALLBACK_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_SBAS_CONFIG] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_SBAS_CONFIG] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_SPITFP_ERROR_COUNT] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_BOOTLOADER_MODE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_BOOTLOADER_MODE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_WRITE_FIRMWARE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_SET_STATUS_LED_CONFIG] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_STATUS_LED_CONFIG] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_CHIP_TEMPERATURE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_RESET] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_WRITE_UID] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[GPS_V2_FUNCTION_READ_UID] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[GPS_V2_FUNCTION_GET_IDENTITY] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;

	device_p->callback_wrappers[GPS_V2_CALLBACK_PULSE_PER_SECOND] = gps_v2_callback_wrapper_pulse_per_second;
	device_p->callback_wrappers[GPS_V2_CALLBACK_COORDINATES] = gps_v2_callback_wrapper_coordinates;
	device_p->callback_wrappers[GPS_V2_CALLBACK_STATUS] = gps_v2_callback_wrapper_status;
	device_p->callback_wrappers[GPS_V2_CALLBACK_ALTITUDE] = gps_v2_callback_wrapper_altitude;
	device_p->callback_wrappers[GPS_V2_CALLBACK_MOTION] = gps_v2_callback_wrapper_motion;
	device_p->callback_wrappers[GPS_V2_CALLBACK_DATE_TIME] = gps_v2_callback_wrapper_date_time;

	ipcon_add_device(ipcon_p, device_p);
}

void gps_v2_destroy(GPSV2 *gps_v2) {
	device_release(gps_v2->p);
}

int gps_v2_get_response_expected(GPSV2 *gps_v2, uint8_t function_id, bool *ret_response_expected) {
	return device_get_response_expected(gps_v2->p, function_id, ret_response_expected);
}

int gps_v2_set_response_expected(GPSV2 *gps_v2, uint8_t function_id, bool response_expected) {
	return device_set_response_expected(gps_v2->p, function_id, response_expected);
}

int gps_v2_set_response_expected_all(GPSV2 *gps_v2, bool response_expected) {
	return device_set_response_expected_all(gps_v2->p, response_expected);
}

void gps_v2_register_callback(GPSV2 *gps_v2, int16_t callback_id, void (*function)(void), void *user_data) {
	device_register_callback(gps_v2->p, callback_id, function, user_data);
}

int gps_v2_get_api_version(GPSV2 *gps_v2, uint8_t ret_api_version[3]) {
	return device_get_api_version(gps_v2->p, ret_api_version);
}

int gps_v2_get_coordinates(GPSV2 *gps_v2, uint32_t *ret_latitude, char *ret_ns, uint32_t *ret_longitude, char *ret_ew) {
	DevicePrivate *device_p = gps_v2->p;
	GetCoordinates_Request request;
	GetCoordinates_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_COORDINATES, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_latitude = leconvert_uint32_from(response.latitude);
	*ret_ns = response.ns;
	*ret_longitude = leconvert_uint32_from(response.longitude);
	*ret_ew = response.ew;

	return ret;
}

int gps_v2_get_status(GPSV2 *gps_v2, bool *ret_has_fix, uint8_t *ret_satellites_view) {
	DevicePrivate *device_p = gps_v2->p;
	GetStatus_Request request;
	GetStatus_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_STATUS, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_has_fix = response.has_fix != 0;
	*ret_satellites_view = response.satellites_view;

	return ret;
}

int gps_v2_get_altitude(GPSV2 *gps_v2, int32_t *ret_altitude, int32_t *ret_geoidal_separation) {
	DevicePrivate *device_p = gps_v2->p;
	GetAltitude_Request request;
	GetAltitude_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_ALTITUDE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_altitude = leconvert_int32_from(response.altitude);
	*ret_geoidal_separation = leconvert_int32_from(response.geoidal_separation);

	return ret;
}

int gps_v2_get_motion(GPSV2 *gps_v2, uint32_t *ret_course, uint32_t *ret_speed) {
	DevicePrivate *device_p = gps_v2->p;
	GetMotion_Request request;
	GetMotion_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_MOTION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_course = leconvert_uint32_from(response.course);
	*ret_speed = leconvert_uint32_from(response.speed);

	return ret;
}

int gps_v2_get_date_time(GPSV2 *gps_v2, uint32_t *ret_date, uint32_t *ret_time) {
	DevicePrivate *device_p = gps_v2->p;
	GetDateTime_Request request;
	GetDateTime_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_DATE_TIME, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_date = leconvert_uint32_from(response.date);
	*ret_time = leconvert_uint32_from(response.time);

	return ret;
}

int gps_v2_restart(GPSV2 *gps_v2, uint8_t restart_type) {
	DevicePrivate *device_p = gps_v2->p;
	Restart_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_RESTART, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.restart_type = restart_type;

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_satellite_system_status_low_level(GPSV2 *gps_v2, uint8_t satellite_system, uint8_t *ret_satellite_numbers_length, uint8_t ret_satellite_numbers_data[12], uint8_t *ret_fix, uint16_t *ret_pdop, uint16_t *ret_hdop, uint16_t *ret_vdop) {
	DevicePrivate *device_p = gps_v2->p;
	GetSatelliteSystemStatusLowLevel_Request request;
	GetSatelliteSystemStatusLowLevel_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_SATELLITE_SYSTEM_STATUS_LOW_LEVEL, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.satellite_system = satellite_system;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_satellite_numbers_length = response.satellite_numbers_length;
	memcpy(ret_satellite_numbers_data, response.satellite_numbers_data, 12 * sizeof(uint8_t));
	*ret_fix = response.fix;
	*ret_pdop = leconvert_uint16_from(response.pdop);
	*ret_hdop = leconvert_uint16_from(response.hdop);
	*ret_vdop = leconvert_uint16_from(response.vdop);

	return ret;
}

int gps_v2_get_satellite_status(GPSV2 *gps_v2, uint8_t satellite_system, uint8_t satellite_number, int16_t *ret_elevation, int16_t *ret_azimuth, int16_t *ret_snr) {
	DevicePrivate *device_p = gps_v2->p;
	GetSatelliteStatus_Request request;
	GetSatelliteStatus_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_SATELLITE_STATUS, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.satellite_system = satellite_system;
	request.satellite_number = satellite_number;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_elevation = leconvert_int16_from(response.elevation);
	*ret_azimuth = leconvert_int16_from(response.azimuth);
	*ret_snr = leconvert_int16_from(response.snr);

	return ret;
}

int gps_v2_set_fix_led_config(GPSV2 *gps_v2, uint8_t config) {
	DevicePrivate *device_p = gps_v2->p;
	SetFixLEDConfig_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_FIX_LED_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.config = config;

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_fix_led_config(GPSV2 *gps_v2, uint8_t *ret_config) {
	DevicePrivate *device_p = gps_v2->p;
	GetFixLEDConfig_Request request;
	GetFixLEDConfig_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_FIX_LED_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_config = response.config;

	return ret;
}

int gps_v2_set_coordinates_callback_period(GPSV2 *gps_v2, uint32_t period) {
	DevicePrivate *device_p = gps_v2->p;
	SetCoordinatesCallbackPeriod_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_COORDINATES_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_coordinates_callback_period(GPSV2 *gps_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = gps_v2->p;
	GetCoordinatesCallbackPeriod_Request request;
	GetCoordinatesCallbackPeriod_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_COORDINATES_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int gps_v2_set_status_callback_period(GPSV2 *gps_v2, uint32_t period) {
	DevicePrivate *device_p = gps_v2->p;
	SetStatusCallbackPeriod_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_STATUS_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_status_callback_period(GPSV2 *gps_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = gps_v2->p;
	GetStatusCallbackPeriod_Request request;
	GetStatusCallbackPeriod_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_STATUS_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int gps_v2_set_altitude_callback_period(GPSV2 *gps_v2, uint32_t period) {
	DevicePrivate *device_p = gps_v2->p;
	SetAltitudeCallbackPeriod_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_ALTITUDE_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_altitude_callback_period(GPSV2 *gps_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = gps_v2->p;
	GetAltitudeCallbackPeriod_Request request;
	GetAltitudeCallbackPeriod_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_ALTITUDE_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int gps_v2_set_motion_callback_period(GPSV2 *gps_v2, uint32_t period) {
	DevicePrivate *device_p = gps_v2->p;
	SetMotionCallbackPeriod_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_MOTION_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_motion_callback_period(GPSV2 *gps_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = gps_v2->p;
	GetMotionCallbackPeriod_Request request;
	GetMotionCallbackPeriod_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_MOTION_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int gps_v2_set_date_time_callback_period(GPSV2 *gps_v2, uint32_t period) {
	DevicePrivate *device_p = gps_v2->p;
	SetDateTimeCallbackPeriod_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_DATE_TIME_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_date_time_callback_period(GPSV2 *gps_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = gps_v2->p;
	GetDateTimeCallbackPeriod_Request request;
	GetDateTimeCallbackPeriod_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_DATE_TIME_CALLBACK_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int gps_v2_set_sbas_config(GPSV2 *gps_v2, uint8_t sbas_config) {
	DevicePrivate *device_p = gps_v2->p;
	SetSBASConfig_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_SBAS_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.sbas_config = sbas_config;

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_sbas_config(GPSV2 *gps_v2, uint8_t *ret_sbas_config) {
	DevicePrivate *device_p = gps_v2->p;
	GetSBASConfig_Request request;
	GetSBASConfig_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_SBAS_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_sbas_config = response.sbas_config;

	return ret;
}

int gps_v2_get_spitfp_error_count(GPSV2 *gps_v2, uint32_t *ret_error_count_ack_checksum, uint32_t *ret_error_count_message_checksum, uint32_t *ret_error_count_frame, uint32_t *ret_error_count_overflow) {
	DevicePrivate *device_p = gps_v2->p;
	GetSPITFPErrorCount_Request request;
	GetSPITFPErrorCount_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_SPITFP_ERROR_COUNT, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_error_count_ack_checksum = leconvert_uint32_from(response.error_count_ack_checksum);
	*ret_error_count_message_checksum = leconvert_uint32_from(response.error_count_message_checksum);
	*ret_error_count_frame = leconvert_uint32_from(response.error_count_frame);
	*ret_error_count_overflow = leconvert_uint32_from(response.error_count_overflow);

	return ret;
}

int gps_v2_set_bootloader_mode(GPSV2 *gps_v2, uint8_t mode, uint8_t *ret_status) {
	DevicePrivate *device_p = gps_v2->p;
	SetBootloaderMode_Request request;
	SetBootloaderMode_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_BOOTLOADER_MODE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.mode = mode;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_status = response.status;

	return ret;
}

int gps_v2_get_bootloader_mode(GPSV2 *gps_v2, uint8_t *ret_mode) {
	DevicePrivate *device_p = gps_v2->p;
	GetBootloaderMode_Request request;
	GetBootloaderMode_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_BOOTLOADER_MODE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_mode = response.mode;

	return ret;
}

int gps_v2_set_write_firmware_pointer(GPSV2 *gps_v2, uint32_t pointer) {
	DevicePrivate *device_p = gps_v2->p;
	SetWriteFirmwarePointer_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.pointer = leconvert_uint32_to(pointer);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_write_firmware(GPSV2 *gps_v2, uint8_t data[64], uint8_t *ret_status) {
	DevicePrivate *device_p = gps_v2->p;
	WriteFirmware_Request request;
	WriteFirmware_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_WRITE_FIRMWARE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	memcpy(request.data, data, 64 * sizeof(uint8_t));

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_status = response.status;

	return ret;
}

int gps_v2_set_status_led_config(GPSV2 *gps_v2, uint8_t config) {
	DevicePrivate *device_p = gps_v2->p;
	SetStatusLEDConfig_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_SET_STATUS_LED_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.config = config;

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_get_status_led_config(GPSV2 *gps_v2, uint8_t *ret_config) {
	DevicePrivate *device_p = gps_v2->p;
	GetStatusLEDConfig_Request request;
	GetStatusLEDConfig_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_STATUS_LED_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_config = response.config;

	return ret;
}

int gps_v2_get_chip_temperature(GPSV2 *gps_v2, int16_t *ret_temperature) {
	DevicePrivate *device_p = gps_v2->p;
	GetChipTemperature_Request request;
	GetChipTemperature_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_CHIP_TEMPERATURE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_temperature = leconvert_int16_from(response.temperature);

	return ret;
}

int gps_v2_reset(GPSV2 *gps_v2) {
	DevicePrivate *device_p = gps_v2->p;
	Reset_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_RESET, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_write_uid(GPSV2 *gps_v2, uint32_t uid) {
	DevicePrivate *device_p = gps_v2->p;
	WriteUID_Request request;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_WRITE_UID, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.uid = leconvert_uint32_to(uid);

	ret = device_send_request(device_p, (Packet *)&request, NULL, 0);

	return ret;
}

int gps_v2_read_uid(GPSV2 *gps_v2, uint32_t *ret_uid) {
	DevicePrivate *device_p = gps_v2->p;
	ReadUID_Request request;
	ReadUID_Response response;
	int ret;

	ret = device_check_validity(device_p);

	if (ret < 0) {
		return ret;
	}

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_READ_UID, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	*ret_uid = leconvert_uint32_from(response.uid);

	return ret;
}

int gps_v2_get_identity(GPSV2 *gps_v2, char ret_uid[8], char ret_connected_uid[8], char *ret_position, uint8_t ret_hardware_version[3], uint8_t ret_firmware_version[3], uint16_t *ret_device_identifier) {
	DevicePrivate *device_p = gps_v2->p;
	GetIdentity_Request request;
	GetIdentity_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), GPS_V2_FUNCTION_GET_IDENTITY, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response, sizeof(response));

	if (ret < 0) {
		return ret;
	}

	memcpy(ret_uid, response.uid, 8);
	memcpy(ret_connected_uid, response.connected_uid, 8);
	*ret_position = response.position;
	memcpy(ret_hardware_version, response.hardware_version, 3 * sizeof(uint8_t));
	memcpy(ret_firmware_version, response.firmware_version, 3 * sizeof(uint8_t));
	*ret_device_identifier = leconvert_uint16_from(response.device_identifier);

	return ret;
}

int gps_v2_get_satellite_system_status(GPSV2 *gps_v2, uint8_t satellite_system, uint8_t *ret_satellite_numbers, uint8_t *ret_satellite_numbers_length, uint8_t *ret_fix, uint16_t *ret_pdop, uint16_t *ret_hdop, uint16_t *ret_vdop) {
	int ret = 0;
	uint8_t satellite_numbers_length;
	uint8_t satellite_numbers_data[12];

	*ret_satellite_numbers_length = 0;

	ret = gps_v2_get_satellite_system_status_low_level(gps_v2, satellite_system, &satellite_numbers_length, satellite_numbers_data, ret_fix, ret_pdop, ret_hdop, ret_vdop);

	if (ret < 0) {
		return ret;
	}

	memcpy(ret_satellite_numbers, satellite_numbers_data, sizeof(uint8_t) * satellite_numbers_length);
	memset(&ret_satellite_numbers[satellite_numbers_length], 0, sizeof(uint8_t) * (12 - satellite_numbers_length));

	*ret_satellite_numbers_length = satellite_numbers_length;

	return ret;
}

#ifdef __cplusplus
}
#endif

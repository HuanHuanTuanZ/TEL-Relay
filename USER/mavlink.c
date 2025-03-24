#include "Ctrl.h"
#include "open_tel_mavlink.h"
#include "common.h"
/*

heartbeat
attitude
sys_status
battery_status
gps_raw_int
global_position_int
radio_status
rc_channel
vfr_hud
*/

void send_gps_aat()
{
	GPS_raw.lat = crsf_latitude;
	GPS_raw.lon = crsf_longitude;
	GPS_raw.alt = crsf_altitude * 1000;
	GPS_raw.fix_type = 3;
	GPS_raw.satellites_visible = crsf_satellites;
	GPS_raw.eph = 10;
	GPS_raw.epv = 10;
	mavlink_send_message(MAVLINK_COMM_0, MSG_GPS_RAW, 0); // 发送心跳
}

void send_position_aat()
{
	position.lat = crsf_latitude;
	position.lon = crsf_longitude;
	position.alt = crsf_altitude * 1000;
	position.relative_alt = crsf_altitude * 1000;
	position.hdg = crsf_yaw;
	mavlink_send_message(MAVLINK_COMM_0, MSG_POSITION, 0);
}

void send_heartbeat_aat()
{
	mavlink_send_message(MAVLINK_COMM_0, MSG_HEARTBEAT, 0); // 发送心跳
}

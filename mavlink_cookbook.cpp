#if 0 
// mavlink-related code snippets removed from winglevlr.ino
// A lot of the uses of the mavlink library was the result of a lot of digging
// and experimentation, so saving these snippets 


void mav_gps_msg(float lat, float lon, float crs, float speed, float alt, float hdop, float vdop) {
	uint64_t time_usec = 0;
	uint8_t gps_id = 12;
	uint16_t ignore_flags = 0; 
	uint32_t time_week_ms = 0; 
	uint16_t time_week = 0;
	uint8_t fix_type = 4;
	uint8_t system_type = MAV_TYPE_GENERIC;
	uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
	uint8_t targetSysId = 1;

	float vn = cos(radians(crs)) * speed;
	float ve = sin(radians(crs)) * speed;
	float vd = 0;
	float speed_accuracy = 10.321; 
	float horiz_accuracy = 10.210; 
	float vert_accuracy = 10.321;
	uint8_t satellites_visible = 11;

	mavlink_message_t msg;
	mavlink_msg_gps_input_pack(1, 100, &msg, time_usec, 
	gps_id, ignore_flags, time_week_ms, time_week,  fix_type, lat * 10000000, lon * 10000000,  alt, hdop, vdop, vn, ve, vd,
	speed_accuracy,  horiz_accuracy, vert_accuracy, satellites_visible);
	
	uint8_t mavbuf[1028];
	int len = mavlink_msg_to_send_buffer(mavbuf, &msg);
	mavlink_send(mavbuf, len);
}	


// send mavlink message for use in debuggin mavlink connections 
int new_mode = armServo;
mavlink_msg_set_mode_pack(1, 200, &msg, 1, 1, new_mode); 
len = mavlink_msg_to_send_buffer(buf, &msg);
mavlink_send(buf, len); 

//TODO move this into mavlink cookbook
mavlink_msg_command_long_pack(1, 2, &msg, 0, 0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 4, 0, 0); // 4 == param5
len = mavlink_msg_to_send_buffer(buf, &msg);
mavlink_send(buf, len);


	if (mavTimer.tick()) {
		uint8_t system_type = MAV_TYPE_GENERIC;
		uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
		uint8_t targetSysId = 1;

		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
		
		mavlink_msg_param_request_read_pack(1, 200, &msg, 0, 0, "CRUISE_HEADING", -1);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		mavlink_msg_request_data_stream_pack(1, 200, &msg , 0, 0, MAVLINK_MSG_ID_VFR_HUD , 1 , 1 );
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		mavlink_msg_request_data_stream_pack(1, 200, &msg , 0, 0, MAVLINK_MSG_ID_ATTITUDE , 1 , 1 );
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
 
		mavlink_msg_command_long_pack(1, 2, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);

		mavlink_msg_set_mode_pack(1, 2, &msg, 0, MAV_MODE_FLAG_DECODE_POSITION_SAFETY, (int)(phSafetySwitch));
		len = mavlink_msg_to_send_buffer(buf, &msg);
		mavlink_send(buf, len);
	 
/*		if (apMode == 1) { 
			int new_mode = 0; // 0 == MANUAL  
			desiredTrk = -1;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

		} else if (apMode == 2) {
			int new_mode = 2;  //2 == STABILIZE
			desiredTrk = -1;
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

		} else if (apMode == 3 || apMode == 4 || apMode == 5) {
			int new_mode = 7; // 7 = CRUISE
			mavlink_msg_set_mode_pack(1, 200, &msg, targetSysId, 1, new_mode); 
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len); 

			if (apMode == 3) 
				desiredTrk = (int)re.value;
			if (apMode == 4) 
				desiredTrk = (int)(obs + 15.5  + 360) % 360;
			if (apMode == 5) 
				desiredTrk = navDTK;

			mavlink_msg_param_set_pack(1, 200, &msg, 0, 0, "CRUISE_HEADING", desiredTrk, MAV_VAR_FLOAT);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			mavlink_send(buf, len);
		}
		*/
	}

#endif

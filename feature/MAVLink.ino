#include "MAVLink.h"
#include <SoftwareSerial.h> 

uint8_t system_id = 20;
uint8_t component_id = MAV_COMP_ID_IMU;
uint8_t type = 6; //GCS
uint8_t autopilot = 0; //generic
const long R = 6371e3; // metres

uint8_t received_sysid = 1;
uint8_t received_compid = 1;
uint8_t GCS_UNITS = 0;
int a = 0;
uint32_t homeLatitude;
uint32_t homeLongitude;
unsigned int distance;

//HardwareSerial *mavSerial; // RX, TX
SoftwareSerial mavSerial(3, 4); // RX, TX

void mav_setup() {
  //mavSerial = &teleSerial;
  mavSerial.begin(19200); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  //Serial.begin(57600); //Main serial port to read the port
}

void mav_loop() {

  /*if (a == 0)
  {
    present_msg();
    a = -1;
  }
  else if (a == 1) {
    a = 2;
    start_feeds();
  }*/
  // Define the system type (see mavlink_types.h for list of possible types)
  receve_msg();
}
void present_msg() {
  mavlink_message_t msg;
  mavlink_heartbeat_t heartbeat;
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg,  type, autopilot, 0, 0, 4);
  //send_message(&msg);
  //delay(10);
}


void start_feeds() //request data
{
  //mavlink_msg_request_data_stream_pack(system_id, component_id, &msg,
  //                                     received_sysid, received_compid, MAV_DATA_STREAM_RAW_SENSORS, 20, 1);
  //send_message(&msg);
  //delay(10);
  //mavlink_msg_request_data_stream_pack(system_id, component_id, &msg,
  //                                     received_sysid, received_compid, MAVLINK_MSG_ID_SYS_STATUS, 20, 1);
  //send_message(&msg);

  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {//MAV_DATA_STREAM_RAW_SENSORS,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    //MAV_DATA_STREAM_RC_CHANNELS,
    //MAV_DATA_STREAM_POSITION,
    MAV_DATA_STREAM_EXTRA1,
    //MAV_DATA_STREAM_EXTRA2
  };
  const uint16_t MAVRates[maxStreams] = {0x02, 0x02, /*0x05, 0x02, 0x05, 0x02*/};
  for (int i = 0; i < maxStreams; i++) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg,
                                         received_sysid, received_compid, MAVStreams[i], MAVRates[i], 1);
    send_message(&msg);
    delay(10);
  }
}

void send_message(mavlink_message_t* msg) //send data bit by bit
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  //delay(1000);
  mavSerial.write(buf, len);// Send the message (.write sends as bytes)
}


//void stop_feeds() //stop request
//{
//  mavlink_message_t msg1;
//
//  send_message(&msg1);
//  delay(500);
//}

void calculateDistanceBearing(long lat1, long lon1, long lat2, long lon2, unsigned int *distance/*, int *brng*/){
  float phi1 = lat1 / 1E7 * Pi / 180;
  float phi2 = lat2 / 1E7 * Pi / 180;
  float dPhi = (lat2-lat1) / 1E7 * Pi / 180;
  float dLambda = (lon2 - lon1) / 1E7 * Pi / 180;

  double a = sin(dPhi/2) * sin(dPhi/2) +
    cos(phi1) * cos(phi2) *
    sin(dLambda/2) * sin(dLambda/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  *distance = R * c;

  /*double y = sin(dLambda) * cos(phi2);
  double x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dLambda);
  *brng = atan2(y, x) * 180 / Pi;*/
}

void receve_msg()
{ //receive data over serial
  mavlink_message_t msg;
  mavlink_status_t status;
  while (mavSerial.available() > 0)
  {
    uint8_t rec =  mavSerial.read();//Show bytes send from the pixhawk
    //Serial.println(rec);
    mavlink_parse_char(*&mavSerial, rec, &msg, &status);
    // Serial.println("sali");
  }
  gcs_handleMessage(&msg);
}

void gcs_handleMessage(mavlink_message_t* msg) //read varaible
{
  //Serial.println();
  //Serial.print("Message ID: ");
  //Serial.println(msg->msgid);
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      {
        mavlink_heartbeat_t packet;
        mavlink_msg_heartbeat_decode(msg, &packet);
        uint8_t beat = 1;
        if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
          uint8_t received_sysid = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
          uint8_t received_compid = (*msg).compid;
          uint8_t bmode = packet.base_mode;
          uint8_t cmode = packet.custom_mode;
          a = 1;
          //Serial.println("Im in heartbeat");

        }
        break;
      }
    case MAVLINK_MSG_ID_ATTITUDE:
      {
        if (telemetry_status & TELEMETRY_ATT)
          break;
        // decode
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(msg, &packet);
        uint16_t pitch = packet.pitch / 2 / Pi / 18 * 0xFFFF;
        uint16_t yaw = packet.yaw / 2 / Pi / 18 * 0xFFFF;
        uint16_t roll = packet.roll / 2 / Pi / 18 * 0xFFFF;
        tele_attitude[1] = yaw >> 8 & 0xFF;
        tele_attitude[2] = yaw & 0xFF;
        tele_attitude[1] = yaw >> 8 & 0xFF;
        tele_attitude[3] = roll >> 8 & 0xFF;
        tele_attitude[4] = roll & 0xFF;
        tele_attitude[5] = pitch >> 8 & 0xFF;
        tele_attitude[6] = pitch & 0xFF;
        telemetry_status |= TELEMETRY_ATT;
        //Serial.println("Im im  Attitude");
        break;
      }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        // decode
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(msg, &packet);
        uint8_t  gpsfix = packet.fix_type;
        uint8_t  mav_utime = packet.time_usec;
        uint8_t  numSats = packet.satellites_visible;
        tele_attitude[14] = numSats;
        uint8_t  cog = packet.cog;
        //Serial.println("Im im  Raw int");
        break;
      }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        int32_t  latitude = packet.lat;
        int32_t  longitude = packet.lon;
        int16_t altitude = packet.relative_alt / 100;
        //else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) ) altitude = (packet.alt / 100) * 3.28084;
        tele_position[1] = latitude >> 24;
        tele_position[2] = latitude >> 16;
        tele_position[3] = latitude >> 8;
        tele_position[4] = longitude;
        tele_position[5] = longitude >> 24;
        tele_position[6] = longitude >> 16;
        tele_position[7] = longitude >> 8;
        tele_position[8] = longitude;
        tele_attitude[11] = altitude >> 8;
        tele_attitude[12] = altitude;
        telemetry_status |= TELEMETRY_POS;
        if(!homeLatitude){
          homeLatitude = latitude;
          homeLongitude = longitude;
        }else{
          calculateDistanceBearing(homeLatitude, homeLongitude, latitude, longitude, &distance/*, &brng*/);
          tele_attitude[9] = distance >> 8;
          tele_attitude[10] = distance;
        }
        //Serial.println("Im im  Global pos int");
        break;
      }
    case MAVLINK_MSG_ID_VFR_HUD:
      {
        mavlink_vfr_hud_t packet;
        mavlink_msg_vfr_hud_decode(msg, &packet);
        int16_t  speed = packet.groundspeed * 10;
        //int16_t  altitude = packet.alt / 100;
        tele_attitude[7] = speed >> 8 & 0xFF;
        tele_attitude[8] = speed & 0xFF;
        //tele_attitude[11] = altitude >> 8;
        //tele_attitude[12] = altitude & 0xFF;
        break;
      }
    case MAVLINK_MSG_ID_SYS_STATUS:
      {
        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(msg, &packet);
        //uint8_t  vbat = packet.voltage_battery;
        tele_attitude[13] = packet.voltage_battery / 100;
        //telemetry_status |= TELEMETRY_STS;
        //Serial.println("Im im  Sys status");
        break;
      }
  }

}

/*
   float roll = ((18.0 * bytes2int(packet[3], packet[4]) / 0xFFFF) ) * 2 * Pi;
      float pitch = ((18.0 * bytes2int(packet[5], packet[6]) / 0xFFFF) ) * 2 * Pi;
      yaw = ((18.0 * bytes2int(packet[1], packet[2]) / 0xFFFF) ) * 2 * Pi;
      acSpeed = packet[8] / 10.0;
      acDistance = (packet[11] << 8 | packet[12]) / 10.0;
      gpsCount = packet[14];
      mavlink_msg_attitude_pack(sysid,MAV_COMP_ID_IMU, &msg, 0, roll, pitch, yaw, 0, 0, 0);
      mavlink_send(msg);
      //mavlink_msg_gps_status_pack(sysid,MAV_COMP_ID_GPS, &msg, gpsCount, 0, gpsCount, 0, 0, 0);
      //mavlink_send(msg);
      mavlink_msg_sys_status_pack(sysid,MAV_COMP_ID_ALL, &msg, 0, 0, 0, 0, packet[13]*100, 0, 0, 0, 0, 0, 0, 0, 0);
      mavlink_send(msg);

*/
//void heartbeat_msg()//MAVLINK_MSG_ID_HEARTBEAT 0
//{
//  uint8_t system_type = MAV_TYPE_QUADROTOR;
//  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
//  uint8_t base_mode =MAV_MODE_FLAG_ENUM_END;
//  uint8_t custom_mode = 0 ;
//  uint8_t system_status = MAV_STATE_STANDBY ;
//
//  mavlink_message_t msg;// Initialize the required buffers
//  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
//  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, system_type, autopilot_type, base_mode, custom_mode, system_status);// Pack the message
//  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);// Copy the message to send buffer
//  delay(1000);
//  mavSerial.write(buf, len);// Send the message (.write sends as bytes)
//}



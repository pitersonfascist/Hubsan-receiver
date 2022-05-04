/* #################################################################################################################
   LightTelemetry protocol (LTM)

   Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds).

   Protocol details: 3 different frames, little endian.
     G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
      0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0
       $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
     A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
       0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0
        $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
     S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
       0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0
        $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
   ################################################################################################################# */

#if defined(PROTOCOL_LIGHTTELEMETRY)
#include "LightTelemetry/LightTelemetry.h"
#include <SoftwareSerial.h> 


static uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH - 4];
static uint8_t LTMreceiverIndex;
static uint8_t LTMcmd;
static uint8_t LTMrcvChecksum;
static uint8_t LTMreadIndex;
static uint8_t LTMframelength;

#define TELEMETRY_ATT    1
#define TELEMETRY_POS    2
#define AVERAGE_ITERATIONS  10

u8 telemetry_status = 0;
u8 tele_attitude[16];
u8 tele_position[16];

const long R = 6371e3; // metres

SoftwareSerial ltmSerial(3, 4); // RX, TX



uint8_t ltmread_u8()  {
  return LTMserialBuffer[LTMreadIndex++];
}

uint16_t ltmread_u16() {
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8() << 8;
  return t;
}

uint32_t ltmread_u32() {
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16() << 16;
  return t;
}


void ltm_read() {
  
  uint8_t c;

  static enum _serial_state {
    IDLE,
    HEADER_START1,
    HEADER_START2,
    HEADER_MSGTYPE,
    HEADER_DATA
  }
  c_state = IDLE;

  while (ltmSerial.available()) {
    c = char(ltmSerial.read());
    //Serial.print(c);;

    if (c_state == IDLE) {
      c_state = (c == '$') ? HEADER_START1 : IDLE;
      
    }
    else if (c_state == HEADER_START1) {
      c_state = (c == 'T') ? HEADER_START2 : IDLE;
      
    }
    else if (c_state == HEADER_START2) {
      switch (c) {
        case 'G':
          LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          break;
        case 'A':
          LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'S':
          LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'O':
          LTMframelength = LIGHTTELEMETRY_OFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'N':
          LTMframelength = LIGHTTELEMETRY_NFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'X':
          LTMframelength = LIGHTTELEMETRY_XFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          ;
          break;
        default:
          c_state = IDLE;
      }
      LTMcmd = c;
      LTMreceiverIndex = 0;
    }
    else if (c_state == HEADER_MSGTYPE) {
      if (LTMreceiverIndex == 0) {
        LTMrcvChecksum = c;
      }
      else {
        LTMrcvChecksum ^= c;
      }
      if (LTMreceiverIndex == LTMframelength - 4) { // received checksum byte
        if (LTMrcvChecksum == 0) {
          //telemetry_ok = true;
          LTM_pkt_ok++;   //increase packet ok counter
          //lastpacketreceived = millis();
          ltm_check();
          c_state = IDLE;
        }
        else {                                                   // wrong checksum, drop packet
          LTM_pkt_ko++;       //increase packet dropped counter
          c_state = IDLE;
          

        }
      }
      else LTMserialBuffer[LTMreceiverIndex++] = c;
    }
  }
}

// --------------------------------------------------------------------------------------
// Decoded received commands
void ltm_check() {
  
  LTMreadIndex = 0;

  if (LTMcmd == LIGHTTELEMETRY_GFRAME)
  {

    uav_lat = (int32_t)ltmread_u32();
    uav_lon = (int32_t)ltmread_u32();
    uav_groundspeedms = ltmread_u8();
    uav_groundspeed = (uint16_t) round((float)(uav_groundspeedms * 3.6f)); // convert to kmh
    uav_alt = (int32_t)ltmread_u32()/10;//convert to dm
    uint8_t ltm_satsfix = ltmread_u8();
    uav_satellites_visible         = (ltm_satsfix >> 2) & 0xFF;
    uav_fix_type                   = ltm_satsfix & 0b00000011;
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_GFRAMELENGTH - 4);

  }

  if (LTMcmd == LIGHTTELEMETRY_AFRAME)
  {
    uav_pitch = (int16_t)ltmread_u16();
    uav_roll =  (int16_t)ltmread_u16();
    uav_heading = (int16_t)ltmread_u16();
    if (uav_heading < 0 ) uav_heading = uav_heading + 360; //convert from -180/180 to 0/360°
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_AFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_SFRAME)
  {
    uav_bat = ltmread_u16();
    uav_amp = ltmread_u16();
    uav_rssi = (ltmread_u8()*100)/255;
    uav_airspeed = ltmread_u8()*100;
    ltm_armfsmode = ltmread_u8();
    uav_arm = ltm_armfsmode & 0b00000001;
    uav_failsafe = (ltm_armfsmode >> 1) & 0b00000001;
    uav_flightmode = (ltm_armfsmode >> 2) & 0b00111111;
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_SFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_OFRAME) // origin frame
  {
    uav_homelat = (int32_t)ltmread_u32();
    uav_homelon = (int32_t)ltmread_u32();
    uav_homealt = (int32_t)ltmread_u32();
    uav_osd_on = (int8_t)ltmread_u8(); // always 1
    uav_homefixstatus = (int8_t)ltmread_u8(); // home fix status
    
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_OFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_NFRAME)
  {
    uav_gpsmode = ltmread_u8();
    uav_navmode = ltmread_u8();
    uav_navaction = ltmread_u8();
    uav_WPnumber = ltmread_u8();
    ltm_naverror = ltmread_u8();
    ltm_flags = ltmread_u8();
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_NFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_XFRAME)
  {
    uav_HDOP = ltmread_u16();
    uav_HWstatus = ltmread_u8();
    uav_spare1 = ltmread_u8();
    uav_spare2 = ltmread_u8();
    ltm_spare3 = ltmread_u8();
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_XFRAMELENGTH - 4);
  }
}

void ltm_setup() {
  //mavSerial = &teleSerial;
  ltmSerial.begin(9600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  //Serial.begin(57600); //Main serial port to read the port

  tele_attitude[0] = 0xE7;
  tele_position[0] = 0xE8;
  tele_position[9] = 0x11;
  // 0x05 - HOME, 0x10 - MANUAL, 0x20 - STAB, 0x30 - RTH, 
  tele_position[10] = 0x00;
  
}

///////////////////////  distance and direction between two points giving coordinates

void GPS_dist_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, uint32_t* dist, int32_t* bearing) {

  if (uav_homefixstatus == 1) {        //gps home fix is OK
  
    /*float rads = (abs((float) * lat2) / 10000000.0) * 0.0174532925; // latitude to radians
    double scaleLongDown = cos (rads); // calculate longitude scaling **taking lat2 ? why not lat1 ?
  
    float distLat = *lat2 - *lat1;                                          // difference of latitude in 1/10 000 000 degrees
    float distLon = (float)(*lon2 - *lon1) * scaleLongDown;
    //distance calculation
    *dist = sqrt(sq(distLat) + sq(distLon)) * 1.113195 / 100;          // distance between two GPS points in m
    //direction calculation
    *bearing = (int) round ( 90 + atan2(-distLat, distLon) * 57.2957795);      //bearing, convert the output radians to deg
    if (*bearing < 0) *bearing += 360; */

    float phi1 = (float) *lat1 / 1E7 * Pi / 180;
    float phi2 = (float) *lat2 / 1E7 * Pi / 180;
    float dPhi = (*lat2-*lat1) / 1E7 * Pi / 180;
    float dLambda = (*lon2 - *lon1) / 1E7 * Pi / 180;
  
    double a = sin(dPhi/2) * sin(dPhi/2) +
      cos(phi1) * cos(phi2) *
      sin(dLambda/2) * sin(dLambda/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
    *dist = R * c;
  
    double y = sin(dLambda) * cos(phi2);
    double x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dLambda);
    *bearing = atan2(y, x) * 180 / Pi;
  }
  else                         // we dont have a home fix
  {
    *dist = 0;
    *bearing = 0;
  }

}

u8 map_flight_mode(u8 uav_flightmode) {
  // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
  // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
  // 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
  
  // 0x05 - HOME, 0x10 - MANUAL, 0x20 - STAB, 0x30 - RTH, 
  switch (uav_flightmode) {
    case 0:
    case 1:
    case 4:
      return 0x10;
    case 3:
    case 5:
    case 6:
    case 7:
      return 0x20;
    case 13:
      return 0x30;
  }
  return 0;
}

void ltm_loop() {
  int32_t last_uav_lat = uav_lat; // last values of GPS for calculation of course over ground
  int32_t last_uav_lon = uav_lon; //this only works if uav is moving FIXME
  
  uint16_t ground_courseRaw = 0;
  static uint8_t sig = 0;
  
  ltm_read();           //read LTM telemetry

  if (LTMcmd == LIGHTTELEMETRY_GFRAME) {
    telemetry_status = TELEMETRY_POS;
    /*// distance between two GPS points (last and actual) and course of aircraft
    GPS_dist_bearing(&last_uav_lat, &last_uav_lon, &uav_lat, &uav_lon, &ground_distance, &ground_course);
    
    //aveaging ground_course value
    static uint16_t ground_courseRawArray[AVERAGE_ITERATIONS];
    ground_courseRawArray[(sig++) % AVERAGE_ITERATIONS] = ground_course;
    for (uint8_t i = 0; i < AVERAGE_ITERATIONS; i++) ground_courseRaw += ground_courseRawArray[i];
    ground_course = ground_courseRaw / AVERAGE_ITERATIONS;*/
    
    GPS_dist_bearing(&uav_lat, &uav_lon, &uav_homelat, &uav_homelon, &home_distance, &home_heading);        // calculate some variables from LTM data
    home_heading -= uav_heading;

    tele_position[1] = uav_lat >> 24;
    tele_position[2] = uav_lat >> 16;
    tele_position[3] = uav_lat >> 8;
    tele_position[4] = uav_lat;
    tele_position[5] = uav_lon >> 24;
    tele_position[6] = uav_lon >> 16;
    tele_position[7] = uav_lon >> 8;
    tele_position[8] = uav_lon;
  
    tele_position[10] = map_flight_mode(uav_flightmode);
    tele_position[10] += uav_homefixstatus;
    
    tele_position[11] = uav_alt >> 8;
    tele_position[12] = uav_alt;
    tele_position[13] = uav_bat / 100;
  } else if (LTMcmd == LIGHTTELEMETRY_AFRAME) {
    telemetry_status = TELEMETRY_ATT;

    uint16_t pitch = uav_pitch / 18.0 / 360 * 0xFFFF;
    uint16_t roll = uav_roll / 18.0 / 360 * 0xFFFF;
    uint16_t yaw = home_heading / 18.0 / 360 * 0xFFFF;
  
    /*Serial.print("FM: ");
    Serial.print(uav_flightmode);
    Serial.print(" R: ");
    Serial.print(uav_roll);
    Serial.print(" Y: ");
    Serial.print(uav_heading);
    Serial.println("");*/
    
    tele_attitude[1] = yaw >> 8 & 0xFF;
    tele_attitude[2] = yaw & 0xFF;
    tele_attitude[1] = yaw >> 8 & 0xFF;
    tele_attitude[3] = roll >> 8 & 0xFF;
    tele_attitude[4] = roll & 0xFF;
    tele_attitude[5] = pitch >> 8 & 0xFF;
    tele_attitude[6] = pitch & 0xFF;
    tele_attitude[7] = (10 * uav_groundspeedms) >> 8;
    tele_attitude[8] = (10 * uav_groundspeedms);
    tele_attitude[9] = (10 * home_distance) >> 8;
    tele_attitude[10] = (10 * home_distance);
    tele_attitude[11] = uav_alt >> 8;
    tele_attitude[12] = uav_alt;
    tele_attitude[13] = uav_bat / 100;
    tele_attitude[14] = uav_satellites_visible;
  } else {
    // continue sending previous data
    telemetry_status ^= TELEMETRY_ATT | TELEMETRY_POS;
  }
}

#endif

#include <EEPROM.h>
//#include <Servo.h>

#define CONTROL_PACKET 3
#define SETTING_PACKET 4
#define LEDS_ON 0x05
#define LEDS_OFF 0x06
#define FLIPS_ON 0x07
#define FLIPS_OFF 0x08

uint8_t startval, command;
uint8_t prev_drone_settings = 0x0e;
uint8_t drone_settings_cnt = 0;
uint8_t retry = 0;
//Servo throservo;  // create servo object to control a servo
//Servo ruddservo;  // create servo object to control a servo
//Servo elevservo;  // create servo object to control a servo
//Servo aileservo;  // create servo object to control a servo

void setup() {
  //delay(2000);
  verbose = true;
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  RED_ON();
  BLUE_ON();
  Serial.begin(57600);
  Serial.flush();
  Serial.println("Initialising...");

  // SPI initialisation and mode configuration
  A7105_Setup();

  // calibrate the chip and set the RF frequency, timing, transmission modes, session ID and channel
  initialize();

#ifdef PPM
  ppm_setup();
#endif
#ifdef SBUS
  sbus_setup();
#endif
#ifdef IBUS
  ibus_setup();
#endif

  //init_rssi();
  ltm_setup();

  /*elevservo.attach(9);  // attaches the servo on pin 9 to the servo object
  ruddservo.attach(6);  // attaches the servo on pin 9 to the servo object
  throservo.attach(5);  // attaches the servo on pin 9 to the servo object
  aileservo.attach(3);  // attaches the servo on pin 9 to the servo object*/

  rudder = aileron = elevator = 0x7F;
  throttle = 0x00;

  Serial.println("Initialisation Complete");

  state = FIND_C;
  waitTime = 0;
  hubsanWait = 0;
}

void loop() {

  /*
      // don't attempt to read from serial unless there is data being sent
      if (Serial.available() > 4) {

      // work through data on serial until we find the start of a packet
      do {
          startval = Serial.read();
      }
      while (startval != CONTROL_PACKET && startval != SETTING_PACKET);

      if (startval == CONTROL_PACKET) {
          throttle=Serial.read();
          rudder=Serial.read();
          aileron=Serial.read();
          elevator=Serial.read();
      }

      else if (startval == SETTING_PACKET) {
          command = Serial.read();
          if (command == LEDS_ON) {
              drone_settings |= 0x04;
          }
          else if (command == LEDS_OFF) {
              drone_settings &= ~0x04;
          }
          else if (command == FLIPS_ON) {
              drone_settings |= 0x08;
          }
          else if (command == FLIPS_OFF) {
              drone_settings &= ~0x08;
          }

          // clear unused bytes
          for ( int i = 0 ; i < 3 ; i++)
              Serial.read();
      }
      }*/
  // start the timer for the first packet transmission
  /*startTime = micros();
    // print information about which state the RF dialogue os currently in
    //Serial.print("State: ");
    //Serial.println(state);

    // perform the correct RF transaction
    hubsanWait = hubsan_cb();

    // stop timer for this packet
    finishTime = micros();

    // calculate how long to wait before transmitting the next packet
    waitTime = hubsanWait - (micros() - startTime);

    // wait that long
    delayMicroseconds(waitTime);

    // start the timer again
    startTime = micros();
  */
  switch (state) {
    case FIND_C:
      channel = A7105_findchannel();
      if (channel > 0)
      {
        Serial.print(channel);
        Serial.print(" ");
        Serial.println(channel, HEX);
        A7105_WriteReg(A7105_0F_CHANNEL, channel);
        state = BIND_1;
      } else {
        Serial.println("Work mode");
        EEPROM.get(0, channel);
        EEPROM.get(1, sessionid);
        //EEPROM.get(2, txid);
        A7105_WriteReg(A7105_0F_CHANNEL, channel);
        A7105_WriteID(sessionid);
        state = DATA_1;
      }
      // setRSSIChannel(channel);
      break;
    case BIND_1:
    case BIND_2:
    case BIND_3:
    case BIND_4:
      //if (state == BIND_4) state = BIND_1;
      if (state == BIND_1 || state == BIND_2) {
        command = 0x01;
        Serial.println("HANDSHAKE 1 ");
        if (state == BIND_2) command = 0x03;
      } else if (state == BIND_3) {
        Serial.println("HANDSHAKE 2 ");
        channel = receivedpacket[1];
        startval = receivedpacket[2];
        A7105_WriteReg(A7105_0F_CHANNEL, channel);
        // use the data from the packet to switch the chip over to the transactions session ID
        A7105_SetTxRxMode(TXRX_OFF);
        sessionid = ((unsigned long)receivedpacket[2] << 24) | ((unsigned long)receivedpacket[3] << 16) | ((unsigned long)receivedpacket[4] << 8) | (unsigned long)receivedpacket[5];
        txid = ((unsigned long)receivedpacket[11] << 24) | ((unsigned long)receivedpacket[12] << 16) | ((unsigned long)receivedpacket[13] << 8) | (unsigned long)receivedpacket[14];
        Serial.println(txid, HEX);
        A7105_WriteID(sessionid);
        A7105_ReadID();
        EEPROM.put(0, channel);
        EEPROM.put(1, sessionid);
        //EEPROM.put(2, txid);
        //channel = A7105_findchannel();
        command = 0x01;
      } else if (state == BIND_4) {
        Serial.println("HANDSHAKE 3 ");
        command = 0x09;
      }
      //Serial.println(state);
      while (A7105_sniffchannel() != 1);
      printpacket(receivedpacket);
      if (state == BIND_4 && receivedpacket[0] == 0x20) {
        state = DATA_1;
        return;
      }
      if (receivedpacket[0] != command) {
        retry ++;
        if (retry > 5 ) {
          state = FIND_C;
          retry = 0;
         }
        return;
      }

      if (state <= BIND_2) {
        //receivedpacket[2] = (sessionid >> 24) & 0xff;
        //receivedpacket[3] = (sessionid >> 16) & 0xff;
        //receivedpacket[4] = (sessionid >>  8) & 0xff;
        //receivedpacket[5] = (sessionid >>  0) & 0xff;
      }
      if (state >= BIND_3) {
        receivedpacket[6] = 0x03;
        receivedpacket[7] = 0x02;
        receivedpacket[8] = 0x0;
      } else {
        //receivedpacket[6] = 0x03;
        //receivedpacket[7] = 0x02;
        //receivedpacket[8] = 0x0;
      }
      //receivedpacket[13] = 0x7E;
      //receivedpacket[14] = 0x14;
      if (state == BIND_4) {
        receivedpacket[0] = 0x0A;
        receivedpacket[1] = receivedpacket[2] + 0x01;
        if (receivedpacket[1] == 0x09) {
          state = DATA_1;
          //break;
        }
        receivedpacket[2] = startval;
      } else {
        receivedpacket[0] += 0x01;
        state += 1;
      }

      packet_crc(receivedpacket);
      //delayMicroseconds(3000);

      // write packet into fifo
      A7105_SetTxRxMode(TX_EN);
      A7105_Strobe(A7105_STANDBY);
      A7105_WriteData(receivedpacket, 16, channel > 0 ? channel : receivedpacket[1]);
      //Serial.print("->");
      //printpacket(receivedpacket);
      //delayMicroseconds(500);
      // allow 20 loops for the transmitting flag to clear
     
      A7105_SetTxRxMode(TXRX_OFF);
      A7105_Strobe(A7105_STANDBY);
      //Serial.flush();
      break;
    case DATA_1:
      //A7105_SetPower(TXPOWER_150mW);
      A7105_WriteReg(A7105_1F_CODE_I, 0x0F); // enable CRC
    case DATA_2:
    case DATA_3:
    case DATA_4:
    case DATA_5:
      //Serial.println("NEXT STEP ");
      if (A7105_sniffchannel() == 1)
      {
        if(!((receivedpacket[11]==(txid >> 24) & 0xff)&&(receivedpacket[12]==(txid >> 16) & 0xff)&&(receivedpacket[13]==(txid >> 8) & 0xff)&&(receivedpacket[14]==(txid >> 0) & 0xff))&&0)
          return; // not our TX !
        if(!hubsan_check_integrity())
          return; // bad checksum
        // Read TX RSSI
        //byte trssi = readRSSI();    // value from A7105 is between 8 for maximum signal strength to 160 or less
        //rssi = /*0.95 * rssi + 0.05 */trssi;
        //percentrssi = map(rssi, 8, 160, 100, 0);
        checkRSSI();
        if (receivedpacket[0] == 0x20) {
          //printpacket(receivedpacket);
          throttle = receivedpacket[2];
          rudder = receivedpacket[4]; // Rudder is not reversed
          elevator = 0xff - receivedpacket[6]; // Elevator is not reversed
          aileron = 0xff - receivedpacket[8]; // Ailerons is reversed
          if(prev_drone_settings != receivedpacket[9]){ // prevent fluctuations
            prev_drone_settings = receivedpacket[9];
            drone_settings_cnt = 0;
          }else if(drone_settings_cnt > 10) {
            drone_settings = receivedpacket[9];
          }else {
            drone_settings_cnt ++;
          }
          kmob1 = receivedpacket[14];
          kmob2 = receivedpacket[12];
          /*Serial.print("throttle = ");
            Serial.print(throttle);
            Serial.print(" rudder = ");
            Serial.print(rudder);
            Serial.print(" elevator = ");
            Serial.print(elevator);
            Serial.print(" aileron = ");
            Serial.print(aileron);
            Serial.print(" flags = ");
            Serial.print(drone_settings, BIN);
            //Serial.print(prev_drone_settings, BIN);
            Serial.print(" RSSI = ");
            Serial.print(percentrssi);
            Serial.println("");
            */
#ifdef PPM
          update_ppm();
#endif
#ifdef SBUS
          update_sbus_channels();
#endif
#ifdef IBUS
          update_ibus_channels();
#endif
          /*elevservo.write(map(elevator, 0, 255, 1000, 2000));                  // sets the servo position according to the scaled value
          ruddservo.write(map(0xff - rudder, 0, 255, 1000, 2000));                  // sets the servo position according to the scaled value
          throservo.write(map(throttle, 0, 255, 1000, 2000));                  // sets the servo position according to the scaled value
          aileservo.write(map(0xff - aileron, 0, 255, 1000, 2000));                  // sets the servo position according to the scaled value
          */
          //40 17 20 14 77 0 80 0 86 6 14 0 0 0 0 DE
          //40 - controll   1,2 Video Mhz
        } else {
          printpacket(receivedpacket);
        }
        
        if (telemetry_status)
        {
          /*if (state % 2 == 0){
            tele_attitude[0] = 0xe7;
            int att[] = {0x00, 0x10, 0x00, 0xFF, 0x00, 0xF0, 0x03, 0xE7, 0x7F, 0xFF, 0xFF, 0xFF, 0x07};
            for(int i = 0; i< 12; ++i) {
              tele_attitude[1 + i] = att[i];
            }
            tele_attitude[14] = 5;
          }else{
            tele_attitude[0] = 0xe8;
            int lat[] = {0x1E, 0x09, 0xF1, 0x00, 0x12, 0x30, 0xAD, 0xC4, 0x11, 0x20, 0x00, 0xAF};
            for(int i = 0; i< 12; ++i) {
              tele_attitude[1 + i] = lat[i];
            }
          }
          tele_attitude[13] = percentrssi;*/
          packet_crc(telemetry_status & TELEMETRY_POS ? tele_position : tele_attitude);
          
          //printpacket(*telemetrypacket);
          //delayMicroseconds(9000);

          // write packet into fifo
          A7105_SetTxRxMode(TX_EN);
          A7105_Strobe(A7105_STANDBY);
          A7105_WriteData(telemetry_status & TELEMETRY_POS ? tele_position : tele_attitude, 16, channel);
          
          telemetry_status = 0;
          //delayMicroseconds(3000);          
          //A7105_WriteReg(0x0F, channel);
        } 
        
        A7105_SetTxRxMode(TXRX_OFF);
        A7105_Strobe(A7105_STANDBY);
        //A7105_SetTxRxMode(RX_EN);
      }
      break;
  }

//  update_rssi();
#ifdef SBUS
  update_sbus();
#endif
#ifdef IBUS
  update_ibus();
#endif
  ltm_loop();
  //Serial.println(A7105_ReadReg(0x00));
  //A7105_shoutchannel();
  //A7105_sniffchannel();
  //A7105_findchannel();
  //digitalWrite(8, HIGH);
  //A7105_sniffchannel(0x32);
  //A7105_scanchannels(allowed_ch);
  //eavesdrop();
}

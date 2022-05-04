#ifdef IBUS

#define IBUS_MAXCHANNELS 14
#define IBUS_BUFFSIZE 2*IBUS_MAXCHANNELS + 4    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define RC_CHANNEL_MIN 0
#define RC_CHANNEL_MAX 254

#define IBUS_MIN_OFFSET 1000
#define CHANNEL_DEFAULT_VALUE 1500
#define IBUS_MAX_OFFSET 2000

#define IBUS_FRAME_HEADER 0x2040
#define IBUS_UPDATE_RATE 15 //ms
                                                                                                                                     
static int rcChannels[IBUS_MAXCHANNELS];
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
uint32_t ibusTime = 0;
long lastChannelUpdate;

void ibus_setup()
{
  Serial.begin(115200);
  //initiallize default ppm values
  for (int i = 0; i < IBUS_MAXCHANNELS; i++) {
    rcChannels[i] = CHANNEL_DEFAULT_VALUE;
  }
}

void ibusPreparePacket(uint8_t packet[], int channels[], bool isFailsafe){

    static int output;

    packet[0] = IBUS_FRAME_HEADER >> 8; //Header
    packet[1] = IBUS_FRAME_HEADER & 0xFF; //Header
    
    /*
     * Map 0-255 with middle at 127 chanel values to
     * 1000-2000 with middle at 1500 IBUS protocol requires
     */
    for (uint8_t i = 0; i < IBUS_MAXCHANNELS; i++) {
        output = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, IBUS_MIN_OFFSET, IBUS_MAX_OFFSET);
        if (isFailsafe && i == 2)
          output = 930;
        packet[2 * i + 3] = output >> 8;
        packet[2 * i + 2] = output & 0xFF;
    }
    uint16_t chksum = 0xFFFF;
    for (uint8_t i = 0; i < IBUS_BUFFSIZE - 2; ++i)
        chksum -= packet[i];

    packet[IBUS_BUFFSIZE - 1] = chksum >> 8;
    packet[IBUS_BUFFSIZE - 2] = chksum & 0xFF;
}

void update_ibus_channels() {
    rcChannels[0] = aileron;
    rcChannels[1] = elevator;
    rcChannels[2] = throttle;
    rcChannels[3] = rudder;

    rcChannels[4] = drone_settings & SETTINGS_LEDS  ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;
    rcChannels[5] = drone_settings & SETTINGS_HOME  ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;
    rcChannels[6] = drone_settings & SETTINGS_RTH   ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;
    rcChannels[7] = drone_settings & SETTINGS_GPS   ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;

    rcChannels[8] = drone_settings & SETTINGS_STAB  ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;
    rcChannels[9] = drone_settings & SETTINGS_REC   ? RC_CHANNEL_MAX : RC_CHANNEL_MIN;
    rcChannels[10] = kmob1;
    rcChannels[11] = kmob2;
    
    lastChannelUpdate = millis();
}

void update_ibus() {
    
    uint32_t currentMillis = millis();
    rcChannels[12] = map(percentrssi, 0, 100, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    /*
     * Here you can modify values of rcChannels while keeping it in 1000:2000 range
     */
    if (currentMillis > ibusTime) {
        ibusPreparePacket(ibus, rcChannels, currentMillis - lastChannelUpdate > 400);
        Serial.write(ibus, IBUS_BUFFSIZE);
        ibusTime = currentMillis + IBUS_UPDATE_RATE;
    }
} 

#endif

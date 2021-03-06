#include <stdint.h>

#define NUM_OUT_CHANNELS 12
#define CHAN_MULTIPLIER 100
#define CHAN_MAX_VALUE (100 * CHAN_MULTIPLIER)
#define PROTOCOL_LIGHTTELEMETRY
//#define PPM
//#define SBUS
//#define IBUS

#define s16 int16_t
#define u16 uint16_t
#define s32 int32_t
#define u32 uint32_t
#define u8 uint8_t
extern volatile s16 Channels[NUM_OUT_CHANNELS];

const float Pi = 3.14159;


uint16_t rssi;
uint16_t percentrssi;

enum TxPower {
    TXPOWER_100uW,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
};

enum ProtoCmds {
    PROTOCMD_INIT,
    PROTOCMD_DEINIT,
    PROTOCMD_BIND,
    PROTOCMD_CHECK_AUTOBIND,
    PROTOCMD_NUMCHAN,
    PROTOCMD_DEFAULT_NUMCHAN,
    PROTOCMD_CURRENT_ID,
    PROTOCMD_SET_TXPOWER,
    PROTOCMD_GETOPTIONS,
    PROTOCMD_SETOPTIONS,
    PROTOCMD_TELEMETRYSTATE,
};


void PROTOCOL_SetBindState(u32 msec);

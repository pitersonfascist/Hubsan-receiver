#include "mavlink/include/mavlink.h"
#include "mavlink/include/checksum.h"
#include "mavlink/include/mavlink_types.h"
#include "mavlink/include/protocol.h"
//#include <math.h>
#define TELEMETRY_ATT    1
#define TELEMETRY_POS    2
uint8_t telemetry_status = 0;

void send_message(mavlink_message_t* msg);
void gcs_handleMessage(mavlink_message_t* msg);
uint8_t _mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);

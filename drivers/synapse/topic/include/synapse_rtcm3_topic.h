#ifndef SYNAPSE_RTCM3_TOPIC_H
#define SYNAPSE_RTCM3_TOPIC_H

#include <stdint.h>

/*
 * Raw RTCM3 correction frame carried as an in-process topic. A frame is a
 * 3-byte header plus a 10-bit length payload (up to 1023 bytes) plus a 3-byte
 * CRC, so 1030 bytes covers the largest legal frame.
 */
#define SYNAPSE_TOPIC_RTCM3_MAX_BYTES 1030U

struct synapse_topic_Rtcm3 {
	uint32_t len;
	uint8_t data[SYNAPSE_TOPIC_RTCM3_MAX_BYTES];
};
typedef struct synapse_topic_Rtcm3 synapse_topic_Rtcm3_t;

#endif /* SYNAPSE_RTCM3_TOPIC_H */

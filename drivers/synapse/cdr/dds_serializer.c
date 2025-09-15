/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dds_serializer.h"


void *dds_malloc(size_t size)
{
	return NULL;
}
void *dds_realloc(void *ptr, size_t new_size)
{
	printf("Error CDR buffer is too small\n");
	return NULL;
}
void dds_free(void *pt)
{

}
const struct dds_cdrstream_allocator dds_allocator = { dds_malloc, dds_realloc, dds_free };

// CDR Xtypes header {0x00, 0x01} indicates it's Little Endian (CDR_LE representation)
const uint8_t ros2_header[4] = {0x00, 0x01, 0x00, 0x00};

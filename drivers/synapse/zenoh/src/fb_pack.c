/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hand-written FlatBuffer packing for VehicleOpticalFlow tables.
 * Each table wraps a single inline struct field.
 */

#include "fb_pack.h"

#include <string.h>

#include <zephyr/sys/util.h>

BUILD_ASSERT(sizeof(synapse_topic_VehicleOpticalFlowData_t) == 56U);
BUILD_ASSERT(sizeof(synapse_topic_VehicleOpticalFlowVelData_t) == 56U);
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

/*
 * FlatBuffer binary layout for a table with one inline struct field:
 *
 * Offset  Content
 * ------  -------
 * 0       root_offset (uint32 LE) -> points to table start
 * 4       vtable_size (uint16 LE) = 8
 * 6       object_size (uint16 LE) = 4 + sizeof(struct) = 60
 * 8       field_0_offset (uint16 LE) = 4  (offset within object to struct data)
 * 10      padding (uint16) = 0  (vtable is aligned to 4 bytes -> 8 bytes total)
 * 12      vtable_soffset (int32 LE) = 8  (signed offset back to vtable start)
 * 16      struct data (56 bytes)
 * 72      total
 */

enum {
	VTABLE_OFFSET = 4,
	VTABLE_SIZE = 8,
	TABLE_OFFSET = VTABLE_OFFSET + VTABLE_SIZE,
	OBJECT_SIZE = 4 + 56, /* soffset + struct data */
	FIELD_DATA = 4,
};

static void put_le16(uint16_t value, uint8_t *buf)
{
	buf[0] = (uint8_t)(value & 0xffU);
	buf[1] = (uint8_t)((value >> 8) & 0xffU);
}

static void put_le32(uint32_t value, uint8_t *buf)
{
	buf[0] = (uint8_t)(value & 0xffU);
	buf[1] = (uint8_t)((value >> 8) & 0xffU);
	buf[2] = (uint8_t)((value >> 16) & 0xffU);
	buf[3] = (uint8_t)((value >> 24) & 0xffU);
}

static size_t pack_single_struct_table(uint8_t *buf, size_t buf_size,
				       const void *struct_data, size_t struct_size,
				       size_t total_size)
{
	if (buf == NULL || struct_data == NULL || buf_size < total_size) {
		return 0U;
	}

	memset(buf, 0, total_size);

	/* root offset -> TABLE_OFFSET */
	put_le32(TABLE_OFFSET, buf);

	/* vtable */
	uint8_t *vtable = buf + VTABLE_OFFSET;

	put_le16(VTABLE_SIZE, vtable + 0);
	put_le16(OBJECT_SIZE, vtable + 2);
	put_le16(FIELD_DATA, vtable + 4);
	/* vtable[6..7] = 0 (padding, already zeroed) */

	/* table: signed offset back to vtable */
	uint8_t *table = buf + TABLE_OFFSET;

	put_le32(VTABLE_SIZE, table);

	/* struct data */
	memcpy(table + FIELD_DATA, struct_data, struct_size);

	return total_size;
}

size_t fb_pack_vehicle_optical_flow(uint8_t *buf, size_t buf_size,
				    const synapse_topic_VehicleOpticalFlowData_t *data)
{
	return pack_single_struct_table(buf, buf_size, data,
					sizeof(*data),
					FB_VEHICLE_OPTICAL_FLOW_SIZE);
}

size_t fb_pack_vehicle_optical_flow_vel(uint8_t *buf, size_t buf_size,
					const synapse_topic_VehicleOpticalFlowVelData_t *data)
{
	return pack_single_struct_table(buf, buf_size, data,
					sizeof(*data),
					FB_VEHICLE_OPTICAL_FLOW_VEL_SIZE);
}

/* vi: ts=4 sw=4 et */

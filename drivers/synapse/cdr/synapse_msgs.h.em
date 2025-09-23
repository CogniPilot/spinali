/** Auto-generated H file */

#ifndef SYNAPSE_MSGS_H
#define SYNAPSE_MSGS_H

#include <stdint.h>
@{
for entry in json_data:
  print("#include <" + entry['type_description_msg']['type_description']['type_name'] + ".h>")
}@

typedef struct synapse_msg
{
  struct dds_cdrstream_desc* desc;
  const char* msg_name;
  const uint8_t rihs_hash[32];
} synapse_msg;

@{
for entry in json_data:
  type_name = entry['type_description_msg']['type_description']['type_name']
  print("extern synapse_msg " + type_name.replace("/", "_").lower() + ";")
}@

#endif /* SYNAPSE_MSGS_H */

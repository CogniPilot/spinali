/** Auto-generated C file */

@{
def get_type_name(data):
    return data['type_description_msg']['type_description']['type_name']

def get_rihs01_hash(data, type_name):
    for item in data['type_hashes']:
        if item['type_name'] == type_name:
            return item['hash_string'][7:]
    return None

def convert_type_name(type_str):
    # Split the string by '/'
    parts = type_str.split('/')
    # Insert 'dds_' before the last part (the message type)
    return f"{parts[0]}::msg::dds_::{parts[-1]}"

}@

#include "synapse_msgs.h"

@{
for entry in json_data:
  type_name = get_type_name(entry)
  type_hash = get_rihs01_hash(entry, type_name)
  # Type decleration
  print("synapse_msg " + type_name.replace("/", "_").lower() + " = {")
  # CDR descriptor
  print("    &" + type_name.replace("/", "_") + "_cdrstream_desc,")
  # Type Name string
  print("    \"" + convert_type_name(type_name) + "\",")
  # RIHS01 Hash
  print("    // RIHS01_" + type_hash)
  byte_array = [f"0x{type_hash[i:i+2]}" for i in range(0, len(type_hash), 2)]
  print(f"    {{{', '.join(byte_array)} }}")
  print("};")
  print()
}@

# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SPINALI_SYNAPSE_TOPIC topic)
if(CONFIG_SPINALI_SYNAPSE_TOPIC AND CONFIG_NET_GPTP)
  add_subdirectory(time_reference)
endif()
add_subdirectory_ifdef(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW vehicle_optical_flow)
add_subdirectory_ifdef(CONFIG_SPINALI_SYNAPSE_WIRE wire)
add_subdirectory_ifdef(CONFIG_SPINALI_SYNAPSE_ZENOH zenoh)
if(CONFIG_SPINALI_SYNAPSE_MCUMGR_HOOKS OR CONFIG_MCUMGR_GRP_HCDF)
  add_subdirectory(mcumgr_hooks)
endif()

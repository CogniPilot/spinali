# spinali
Connected Sensors and Actuators with Zephyr RTOS

[![Build](https://github.com/CogniPilot/spinali/actions/workflows/build.yml/badge.svg)](https://github.com/CogniPilot/spinali/actions/workflows/build.yml)

See [documentation](https://cognipilot.org/).

Please make sure that your target already has mcuboot (with the default rsa-2048 key). If it does not (or is the first time flashing a new device) please build and flash mcuboot for your target platform (example for the mr-mcxn-t1):
```bash
cd ~/cognipilot/ws/spinali
west build -b mr_mcxn_t1/mcxn947/cpu0 ../bootloader/mcuboot/boot/zephyr/ -d build_mcuboot -p -- \
  -C ../modules/lib/zephyr_boards/boards/nxp/mr_mcxn_t1/mcuboot/mcuboot.cmake
west flash -d build_mcuboot
```

After mcuboot is on the system you can build and flash the target with your spinali application (example of optical flow on the mr-mcxn-t1):
```bash
cd ~/cognipilot/ws/spinali
west build -b mr_mcxn_t1/mcxn947/cpu0 app/optical_flow/ -p
west flash
```

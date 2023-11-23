# ----------------------------------------------------------------------------------------- Imports
import network
import time

# ----------------------------------------------------------------------------------------- Network Config
WIFI_NETWORK='Smokeys-Hotspot'
WIFI_PASSWORD='<password>'


# ----------------------------------------------------------------------------------------- Script
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_NETWORK, WIFI_PASSWORD)

print()
print("Connected to ",WIFI_NETWORK)

import mip
print("Imported mip")

mip.install("github:jposada202020/MicroPython_LSM6DSOX")

from micropython_lsm6dsox import lsm6dsox
print("getting lsm6dsox done")
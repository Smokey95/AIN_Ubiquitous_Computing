# ----------------------------------------------------------------------------------------- Imports
import network
import time
from machine import Pin, I2C
from umqtt.simple import MQTTClient
from micropython_lsm6dsox import lsm6dsox

# ----------------------------------------------------------------------------------------- Network Config
WIFI_NETWORK='Smokeys-Hotspot'
WIFI_PASSWORD='<password>'

# ----------------------------------------------------------------------------------------- Function Definitions

# --- Connect Wifi
def connect_wifi(led):
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        print('Try to connected to network [' + WIFI_NETWORK + '] ...')
        wlan.active(True)
        wlan.connect(WIFI_NETWORK, WIFI_PASSWORD)
        while not wlan.isconnected():
            pass
    i = 4
    while i != 0:
        led.value(1)
        time.sleep(0.05)
        led.value(0)
        time.sleep(0.05)
        i -= 1
    print('Conntected to network [config:', wlan.ifconfig()[0], ']')
    return wlan

# --- Disconnect WiFi
def disconnect_wifi(wlan):
    if wlan.isconnected():
        wlan.disconnect()
        print('Disconnected from network [' + WIFI_NETWORK + ']')
    else:
        print('No network to disconnect from')

# --- Connect to MQTT Broker
def mqtt_connect():
    print('Try to connect to MQTT Broker')
    client = MQTTClient(client_id=b'arduino_smokey',
                        server=b'<server>',
                        port = 0,
                        user=b'<user>',
                        password=b'<password>',
                        keepalive=7200,
                        ssl=True,
                        ssl_params={'server_hostname' : '<server>'}
                        )
    client.connect()
    print('Connected to MQTT Broker')
    return client

def cumulocity_connect():
    print('Try to connect to Cumulocity Broker')
    client = MQTTClient(client_id='button',
                        server='<server>',
                        port=0,
                        user=b'<tenat-id>/<user>',
                        password=b'<password>',
                        keepalive=7200,
                        ssl=True,
                        ssl_params={'server_hostname' : '<server>'}
                        )
    client.connect()
    print('Connected to Cumulocity Broker')
    return client

# --- Reonnect to MQTT Broker
def reconnect():
    print('Failed to connect to the MQTT Broker. Reconnecting...')
    time.sleep(5)
    
def publish(client, topic, msg):
    print('Publish msg [topic:%s] to broker' %topic)
    client.publish(b's/us', b'100,Smokeys_arduino' + msg)
    print('Publish done')

# ----------------------------------------------------------------------------------------- Script
print("------------------------------------------")
print("Hold tigth, setting up device for you ... ")
myLED = Pin(6, Pin.OUT)
myButton = Pin(25, Pin.IN)
myLED.value(0)
wlan = connect_wifi(myLED)

# Connect Cumulocity Broker
try:
    #client = mqtt_connect()
    client = cumulocity_connect()
except OSError as e:
    reconnect()

#
print("Try to get access to the onboard sensors...")
i2c = I2C(0, sda=Pin(12), scl=Pin(13)) # https://micropython-lsm6dsox.readthedocs.io/en/latest/examples.html
lsm = lsm6dsox.LSM6DSOX(i2c)
print("Access to onboard sensors done")


print("Setting up the device done, enter mainloop")
print("------------------------------------------")
      
while True:
    
    if myButton.value() == 1:
        publish(client, b'button', b'true')
        time.sleep(2)
        
    accx, accy, accz = lsm.acceleration
    val = "accx: %2f accy: %2f accz: %2f" %(accx, accy, accz)
    print(val)
    publish(client, b'acc', val)
    time.sleep(2)
        
disconnect_wifi(wlan)
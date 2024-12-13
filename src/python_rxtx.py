#!/bin/python3

import time
import struct

import rodosmwinterface as rodos
rodos.printTopicInit(enable=True)

# Callback
def topicHandler(data):
  try:
    unpacked = struct.unpack("l", data)
    print("stm sends telemetry: {}".format(unpacked[0]))
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

python2rodos = rodos.Topic(1002)
rodos2python = rodos.Topic(1003)

luart = rodos.LinkinterfaceUART(path="/dev/rfcomm0")
gwUart = rodos.Gateway(luart)
gwUart.run()

rodos2python.addSubscriber(topicHandler)
gwUart.forwardTopic(python2rodos)

sensor_index = 0

while True:
  # Dummy sensor data
  sensor_index += 1
  x = 3.1415

  # Pack sensor data to a struct that RODOS recognizes
  sensor_struct = struct.pack("Hd",sensor_index, x)
  print("sending telecom: {}".format(sensor_index))
  python2rodos.publish(sensor_struct)

  time.sleep(1)

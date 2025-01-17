#!/bin/python3

import time
import struct

import rodosmwinterface as rodos
rodos.printTopicInit(enable=True)

ready_for_picture = False

# Callback
def topicHandler(data):
  try:
    unpacked = struct.unpack("B", data)
    print("stm sends data: {}".format(unpacked[0]))
    if(unpacked[0]):
      ready_for_picture = True
  except Exception as e:
    print(e)
    print(data)
    print(len(data))

python2rodos = rodos.Topic(1021)
rodos2python = rodos.Topic(1020)

luart = rodos.LinkinterfaceUART(path="/dev/rfcomm0")
gwUart = rodos.Gateway(luart)
gwUart.run()

rodos2python.addSubscriber(topicHandler)
gwUart.forwardTopic(python2rodos)

sensor_index = 0

while True:
  # Dummy sensor data

  
  # Pack sensor data to a struct that RODOS recognizes
  if(ready_for_picture):
    #take picture then:
    sensor_struct = struct.pack("B",1)
    print("sending telecom: {}".format(sensor_index))
    python2rodos.publish(sensor_struct)
    ready_for_picture = False

  time.sleep(1)

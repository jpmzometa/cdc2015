#!/usr/bin/python3

import struct 

import serial, csv

class System(object):
    def __init__(self, states, hor_inputs, samples):
        self.states = states
        self.hor_inputs = hor_inputs
        self.samples = samples

aircraft = System(5, 5, 60)
#ystem = aircraft
#ystem = legoarm
system = aircraft
#path.append('../mpcctl/systems/' + SYSTEM)

infoWriter = csv.writer(open('simdata.csv', 'w', newline=''), quoting=csv.QUOTE_MINIMAL)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 10)
ser.flushInput()
portName = ser.portstr

head = ['k', 'Tc']
for i in range(system.states):
    head.append('x' + str(i))
for i in range(system.hor_inputs):
    head.append('uo' + str(i))
fmt = '<II' + str(system.states + system.hor_inputs) + 'f'
data = struct.Struct(fmt)
print(portName)
infoWriter.writerow(head)

for i in range(system.samples):
        unp = data.unpack(ser.read(data.size))
        print(unp)
        infoWriter.writerow(unp)
ser.close()

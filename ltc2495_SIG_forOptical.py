#!/usr/bin/python

######################################################################
#
# IF YOU GET IO-READ ERROR - YOU ARE MISSING THE SUCCESSIVE READ
# CAPABILITY ENABLED ON THE PI.
# For I2C and SMBus Documentation see
# http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc
#
# See repeated start discussion or successive read information at
#    https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=15840&start=25
#
# sudo su -
# echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined
# exit
# sudo reboot
# File /sys/module/i2cbcm2708/parameters/combined changed from N to Y
#
# Add line
# echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined
# in /etc/rc.local before the last line. After adding the line, reboot
# the pi.
# See Pi-16ADC User Guide.
#
######################################################################
# (C) ALCHEMY POWER INC 2016,2017 etc. - ALL RIGHT RESERVED.
# CODE SUBJECT TO CHANGE AT ANY TIME.
# YOU CAN COPY/DISTRIBUTE THE CODE AS NEEDED AS LONG AS YOU MAINTAIN
# THE HEADERS (THIS PORTION) OF THE TEXT IN THE FILE.
######################################################################

import time
import smbus
import sys
import os
import subprocess
import numpy as np

from smbus import SMBus
from sys import exit

# Module variables
i2c_ch = 1
bus = None

# ltc2495 address on the I2C bus
i2c_address = 0x14

def init():
	global bus
	
	# for older PI's (version 1) use bus = SMBus(0) in statement below.
	bus = SMBus(i2c_ch)

# Read func config
gain = 1
config = {1:0x80, 4:0x81, 8:0x82, 16:0x83, \
32:0x84, 64:0x85, 128:0x86, 256:0x87}

# Channel Address - Single channel use
channel0        =     0xB0
channel1        =     0xB8
channel2        =     0xB1
channel3        =     0xB9
channel4        =     0xB2

'''
# Channel Address - Differential

# Positive voltage inputs

channel0       =       0xA0  # Differential pair Channel 0 + and Channel 1 -
channel2       =       0xA1  # Differential pair Channel 2 + and Channel 3 -
channel4       =       0xA2  # Differential pair Channel 4 + and Channel 5 -
channel6       =       0xA3  # Differential pair Channel 6 + and Channel 7 -
channel8       =       0xA4  # Differential pair Channel 8 + and Channel 9 -

channelA       =       0xA5  # Differential pair Channel A + and Channel B -
channelC       =       0xA6  # Differential pair Channel C + and Channel D -
channelE       =       0xA7  # Differential pair Channel E + and Channel F -

# Negatives voltage inputs

channel0       =       0xA8  # Differential pair Channel 0 - and Channel 1 +
channel2       =       0xA9  # Differential pair Channel 2 - and Channel 3 +
channel4       =       0xAA  # Differential pair Channel 4 - and Channel 5 +
channel6       =       0xAB  # Differential pair Channel 6 - and Channel 7 +
channel8       =       0xAC  # Differential pair Channel 8 - and Channel 9 +
'''
chs = [channel0, channel1, channel2, channel3, channel4]
#####################################################################################
#
# Determine the reference voltage
#
#####################################################################################
vref = 5.1
#####################################################################################

# To calculate the voltage, the number read in is 3 bytes. The first bit is ignored. 
# Second bit is sign bit. Last 6 bits are always zeros, effective data length is 16 bit (2^16 or 65536) 
# Max reading is 2^23 = 8388608
#

max_reading = 8388608.0

# Now we determine the operating parameters.
# lange = number of bytes to read. A minimum of 3 bytes are read in. In this sample we read in 6 bytes,
# ignoring the last 3 bytes.
# zeit (German for time) - tells how frequently you want the readings to be read from the ADC. Define the
# time to sleep between the readings.
# tiempo (Spanish - noun - for time) shows how frequently each channel is read in over the I2C bus. Best to use
# timepo between each successive readings.
#
#

lange = 0x03 # number of bytes to read in the block
#zeit = 2     # number of seconds to sleep between each measurement
tiempo = 0.15 # number of seconds to sleep between each channel reading

# tiempo - has to be more than 0.2 (seconds).


#====================================================================================
# This is a subroutine which is called from the main routine. 
# The variables passed are:
# adc_address - I2C address for the device. 
#         This is set using the jumpers A0, A1 and A2. Default is 0x45
# adc_channel - the analog channel you want to read in.
#
#====================================================================================
def read_ch_data(adc_address,adc_channel):
# Debug
#	print adc_address, adc_channel
	bus.write_byte(adc_address, adc_channel)
#	bus.write_byte_data(adc_address, adc_channel, config[gain])

# Debug	
#	print("cmd sent: 0x%x" % cmd)
	time.sleep(tiempo)
	reading  = bus.read_i2c_block_data(adc_address, adc_channel, lange)

#----------- Start conversion for the Channel Data ----------
	valor = ((reading[0]&0x3F)<<16)+(reading[1]<<8)+(reading[2]) 
# Debug statements provide additional prinout information for debuggin purposes.
# You can leave those commented out.
# Debug
#	print(reading)
#----------- End of conversion of the Channel ----------
	volts = (valor/max_reading) * (vref)

#######
# See table 1 of the LTC2947 data sheet. 
# If the voltage > Vref/2, then it gives an error condition....
#
# So lets check the first byte
#
#######

	if( (reading[0]& 0b11000000) == 0b11000000): # we found the error
		print ("*************\n")
		print ("Input voltage to channel 0x%x is either open or more than %5.2f mvs.\
				The reading may not be correct. Value read in is %5.2f mvs." \
				% ((adc_channel), 1000 * 0.5 * vref/gain, 1000 * volts))
		print ("*************")
#       else:
#		print (">>>Voltage read in on channel 0x%x is %12.8f Volts" % ((adc_channel),volts))
# Note - print is on stdout - so you can redirect that to a file if needed. You 
# can also comment out the verbiage to suite your needs.

#       time.sleep(tiempo)
# If needed pause the reading..
#
# Note - the pause is usually put in the main routine and not subroutine.
# NAK conditions are not checked for in this sample code. You can add that if needed.
#
	return volts
#====================================================================================

#time.sleep(tiempo)
# Initial startup - a recommendation is to sleep tiempo seconds to give ADC time to stabilize.
# Found that it was worth it. Somehow Python and Pi get into a race situation without it. You can
# expriment omitting the initial sleep.
#
chs_mutl = [1000, 1000, 1000, 1000, 1000] 

def read_data():
	rts = []
	for i in range (0, 5):
		rts.append(round(chs_mutl[i]*read_ch_data(i2c_address,chs[i]), 2))
		time.sleep(tiempo)
	sys.stdout.flush()
	return rts

if __name__=='__main__':
	# Probe testing
	init()
	tic = time.time()
	while 1:
		print(read_data())
	#	print(round(read_data()[0],1),round(read_data()[1],1))
		print("time(s)=",round(time.time() - tic, 2))
		print("mv","gain=",gain)

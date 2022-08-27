## DNA BIT - Heater PID Control System
# Author: Sam Kelly, Gareth Fotouhi, Cifeng Fang
# v3.0.1 - 1. Wrap code into OOD style classes
# 2. Implemented a PID controller with manually SUM of error offset for
#    temperature overshoot porfile design
# 3. Use one ltc2495 adc to do both thermistor and photodiodes sensing

import time
import socket
import numpy as np
import os

import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice as PWM

import ltc2495_DIFF_forHeater as thermistor
import ltc2495_SIG_forOptical as photoDiodes

class wireHeater():
	def __init__(self, pinNum, sensorNum, paras = [76, 20, 0.6, 20]):
		## ADC init ##

		thermistor.init()

		## Initial Values ##
		self.oTC_error = 0
		self.TC_sum_error = 0
		# heater "value" (% value, as it is related to duty cycle)
		self.h = 0

		## Define Parameters ##
		# Target temperature in celcise
		self.target, self.KP, self.KI, self.KD = paras

		# max heater value
		self.h_max = 100

		# min heating value
		self.h_min = 0

		# energy counter (Sums the percentage of max current used during heating.)
		#self.E = 0

		#logs the previous error for the derivative calc
		self.TC_errorR = 0

		self.timer = 0

		## Set Parameters ##
		# control pin for heating system
		pwmPin = pinNum

		self.sensorNum = sensorNum

		# control pin for pow the thermistor
		#self.powThermistor = 27

		pwm_freq = 500
		## Script ##
		# Broadcom pin-numbering scheme
		GPIO.setmode(GPIO.BCM)

		GPIO.setup(pwmPin, GPIO.OUT)
		#GPIO.setup(self.powThermistor, GPIO.OUT)
		## Heating Control Loop ##
		self.heating = GPIO.PWM(pwmPin, pwm_freq)

		self.heating.start(self.h)
		self.th = time.time()
		#time that test was started
		self.t1 = time.time()

		self.tstart = 0
		self.current = 0
		self.currentTemperature = 0
		self.TC_error = 0
		self.TC_sum_error_max = 160 / self.KI
		self.TC_sum_error_min = -160 / self.KI

	def write_data(self):
		with open('./PID Data/data.csv', 'a+') as log:
			log.write(" {} {} {} {} {} {} {} \n".format(self.th-self.t1, \
			self.current, self.h, self.E, self.TC_error *self.KP, \
			self.TC_sum_error *self.KI, self.oTC_error *self.KD ))

	def convert2Temperature(self):
		R1, R2, R_s = 0.022, 0.1, 33 #in kOhms
		Vref = 3300
		B = 4390
		phi = self.current / Vref
		Z = phi * (R1 + R2)
		R_T = (R2 - Z) * R_s / (Z + R1)

		self.currentTemperature = round((1 / (1 / 298.15 + (1 / B) * np.log(R_T / 100))) - 273.15, 2)

	def PIDcontrol(self):
		now = time.time()
		self.TC_error = round(self.target - self.currentTemperature, 5)

		self.oTC_error = (self.TC_error - self.TC_errorR)/(now-self.th)
		self.TC_sum_error += (self.TC_error + self.TC_errorR)*\
		(now-self.th)/2
		# Clamp TC_sum_error Values
		self.TC_sum_error = max(min(self.TC_sum_error_max, self.TC_sum_error)\
		,self.TC_sum_error_min)

		self.h = (self.TC_error * self.KP) + (self.oTC_error * self.KD) \
		+ (self.TC_sum_error * self.KI)

		#Print out control parameter
		print("TargetT {} P_Gain {} I_Gain {} D_Gain {} TotalGain {}".format(self.target, \
		round(self.TC_error *self.KP, 3), round(self.TC_sum_error *self.KI, 3),\
		round(self.oTC_error *self.KD, 3), round(self.h, 3)))

		# Clamp Heater Values
		self.h = max(min(self.h_max, self.h), self.h_min)

		self.heating.ChangeDutyCycle(self.h)
		#time when heater value changed
		self.th = now

		#Remember current error for next PID control call
		self.TC_errorR = self.TC_error

	def rampUP(self):
		# Get current temperature
		self.current = thermistor.getTat(self.sensorNum)
		self.convert2Temperature()
		self.th = time.time()
		self.TC_errorR = round(self.target - self.currentTemperature, 5)

		#self.PIDcontrol()
		self.h = 100
		self.heating.ChangeDutyCycle(self.h)

		#self.E += (self.h/100)*(now - self.th)
		print("TargetT {} ".format(self.target))
		print("TC {} Heater % {} Total time {}"\
		.format(round(self.currentTemperature,3),round(self.h,3), \
		round(self.th - self.t1,3)))
		#self.write_data()

	def settling(self):
		# Get current temperature
		self.current = thermistor.getTat(self.sensorNum)
		self.convert2Temperature()

		self.PIDcontrol()

		#self.E += (self.h/100)*(now - self.th)

		print("TC {} Heater % {} Total time {} Reaction Time {}" \
		.format(round(self.currentTemperature,3), round(self.h,3), \
		round(self.th - self.t1,3),round(self.th - self.tstart,3)))
		#self.write_data()
		self.timer = time.time() - self.tstart


	def heatStep(self, target, heatTime, transmitter):
		self.target = target

		while self.TC_errorR >= 0:
			self.rampUP()
			transmitter.sendDataPoint(round(self.th - self.t1,2), [0, 0, 0, 0, 0], self.currentTemperature)
			#time.sleep(0.15)

		self.tstart = time.time()
		self.TC_sum_error = 18

		while self.timer < heatTime:
			#time.sleep(0.15)
			self.settling()
			transmitter.sendDataPoint(round(self.timer,2), [0, 0, 0, 0, 0], self.currentTemperature)

		self.timer = 0
		self.TC_errorR = 0

	def heatStepSA(self, target, heatTime, file, queue):
		self.target = target

		while self.TC_errorR >= 0:
			self.rampUP()
			dataItem = "{},{}\n".format(round(self.th - self.t1,2), self.currentTemperature)
			file.write(dataItem)

			#time.sleep(0.15)

		self.tstart = time.time()
		self.TC_sum_error = 18

		while self.timer < heatTime:
			#time.sleep(0.15)
			self.settling()
			if not queue.empty():
				queue.get()
			queue.put(2)
			dataItem = "{},{}\n".format(round(self.th - self.t1,2), self.currentTemperature)
			file.write(dataItem)

		self.timer = 0
		self.TC_errorR = 0

	def stop(self):
		self.heating.ChangeDutyCycle(0)
		#GPIO.cleanup()

class Optical:
    def __init__(self, smoothingTimes, LEDslt, lvl = 1):
        photoDiodes.init()
        # Broadcom pin-numbering scheme
        GPIO.setmode(GPIO.BCM)
        if LEDslt:
                self.led = PWM(6, frequency = 200) # pin 6 at 200Hz PWM
        else:
                self.led = PWM(21, frequency = 100)

        self.data = np.zeros(5)
        self.N = smoothingTimes
        self.dutycycle = lvl

    def read(self):

        # Clean last measurements
        self.data = np.zeros(5)

        # Turn on LED
        self.led.value = self.dutycycle

        # Read data from ltc2495
        for i in range(self.N):
                self.data += photoDiodes.read_data()
        self.data = np.round(self.data / self.N, 4)
        # Turn off LED
        self.led.value = 0


        return self.data

    def calibrateSensors(self):
        # Measuring background and sensor offset
        offsets = np.zeros(5)
        for i in range(5):
                offsets += photoDiodes.read_data()
        offsets = np.round(self.data / 5, 4)
        return offsets

    def stop(self):
        self.led.value = 0
        #GPIO.cleanup()


class Transmitter(object):

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((TCP_IP, TCP_PORT))

    def sendDataPoint(self, timeVal, opticsDatas, heaterTemp):
        self.preSend()
        msg = '{},{},{},{},{},{},{}\n'.format(timeVal,opticsDatas[0],\
        opticsDatas[1],opticsDatas[2],opticsDatas[3],opticsDatas[4]\
        , heaterTemp)
        self.socket.send(msg.encode())
        data = self.socket.recv(BUFFER_SIZE)
        print(data.decode() + "sent")
        self.postSend()

    def signalEnd(self):
        self.sendDataPoint(-1,0,0)

    def close(self):
        self.socket.close()

    def preSend(self):
        pass

    def postSend(self):
        pass

class TransmitterSingleSocket(Transmitter):
    def __init__(self):
        Transmitter.__init__(self)
        self.connect()

def lysisRunSA(queue):
	_dir = "/home/pi/bit/v6board/bit/bitGUImultiProcess/dataArchive"
	timestr = time.strftime("%Y%m%d-%H%M%S_Lysis")
	_dir = os.path.join(_dir, timestr)
	if not os.path.exists(_dir):
		os.makedirs(_dir)
	f = open(os.path.join(_dir, "data_log.csv"), "a")
	f.write("Time, Temperature\n")

	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (12 for lysis, 13 for CEAC), paras = [target, P, I, D]
	# sensorNum 0 for lysis, 1 for CEAC
	heater = wireHeater(12, 0, [73, 30, 2, 40])

	# heat profile in duple of target and duration
	heatProfile = [[65, 300]]

	for stepHeating in heatProfile:
		heater.heatStepSA(stepHeating[0], stepHeating[1], f, queue)

	heater.stop()


def react_detectRunSA(queue, rts):
	_dir = "/home/pi/bit/v6board/bit/bitGUImultiProcess/dataArchive"
	timestr = time.strftime("%Y%m%d-%H%M%S_React")
	_dir = os.path.join(_dir, timestr)
	if not os.path.exists(_dir):
		os.makedirs(_dir)
	f = open(os.path.join(_dir, "data_log.csv"), "a")
	f.write("Time,PD1,PD2,PD3,PD4,PD5,Temperature\n")

	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (12 for lysis, 13 for CEAC), paras = [target, P, I, D]
	# sensorNum 0 for lysis, 1 for CEAC
	heater = wireHeater(13, 1, [73, 12, 0.225, 20])

	## Create optics object
	optics = Optical(2, LEDslt = 1, lvl = 0) # Avg from 2 measurements
	darkBackground = optics.read() # Measuring black background of sensors
	optics.dutycycle = 1

	tic = time.time()
	opticsDatas = darkBackground

	while heater.TC_errorR >= 0:
		heater.rampUP()
		timeVal = round(time.time() - tic, 2)
		dataItem = "{},{},{},{},{},{},{}\n".format(timeVal,opticsDatas[0],\
        opticsDatas[1],opticsDatas[2],opticsDatas[3],opticsDatas[4]\
        , heater.currentTemperature)
		f.write(dataItem)
		time.sleep(0.2)

	heater.tstart = time.time()
	heater.TC_sum_error = 680

	counter = 0
	# Get rid of unstable sensor reading at the beginning
	startCnt = 25
	slopLimit = 12 # 12 mv of single increase as Positive
	threshold = np.ones(5, dtype = float) * slopLimit
	datas = []
	volDiffs = [0, 0, 0, 0, 0]

	while heater.timer < 2400:
		heater.settling()
		if not queue.empty():
			queue.get()
		queue.put(2)
		opticsDatas = optics.read()

		timeVal = round(time.time() - tic, 2)
		dataItem = "{},{},{},{},{},{},{}\n".format(timeVal,opticsDatas[0],\
        opticsDatas[1],opticsDatas[2],opticsDatas[3],opticsDatas[4]\
        , heater.currentTemperature)
		f.write(dataItem)

		if counter >= startCnt:
			datas.append(opticsDatas)
		counter += 1
		time.sleep(1.6)

	heater.timer = 0
	datas = np.transpose(np.array(datas))
	for i in range(5):
		# Define sensor reading raise as maximum sum of consecutive diff subarray of size 100
		volDiffs[i] = consecutiveSum(np.diff(smooth(datas[i])), 100)

	results = np.greater_equal(volDiffs, threshold)

	volDiffsItem = "Voltage difference,{},{},{},{},{}\n".format(volDiffs[0],\
        volDiffs[1],volDiffs[2],volDiffs[3],volDiffs[4])
	f.write(volDiffsItem)

	resultsItem = "Result,{},{},{},{},{}\n".format(results[0],\
        results[1],results[2],results[3],results[4])
	f.write(resultsItem)
	heater.stop()
	optics.stop()

	for i, value in enumerate(results):
		rts[i] = value.astype(int)

def smooth(x,window_len=10,window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError("smooth only accepts 1 dimension arrays.")

    if x.size < window_len:
        raise ValueError("Input vector needs to be bigger than window size.")


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")


    s = np.r_[x[window_len-1:0:-1],x,x[-2:-window_len-1:-1]]
    #print(len(s))
    if window == 'flat': #moving average
        w = np.ones(window_len,'d')
    else:
        w = eval('np.'+window+'(window_len)')

    y = np.convolve(w/w.sum(),s,mode='valid')
    return np.round(y, decimals = 3)


def consecutiveSum(arr, window_len):
    if arr.ndim != 1:
        raise ValueError("smooth only accepts 1 dimension arrays.")

    arrSize = arr.size

    if arrSize < window_len:
        length = arrSize
    length = window_len
    maxSum = np.float64(1.0)
    for i in range(length):
        maxSum += arr[i]
    windowSum = maxSum
    for i in range(length,arrSize):
        windowSum += arr[i] - arr[i - length]
        maxSum = np.maximum(maxSum, windowSum)
    return maxSum


def lysisDemo(queue):
	_dir = "/home/pi/bit/v6board/bit/bitGUImultiProcess/dataArchive"
	timestr = time.strftime("%Y%m%d-%H%M%S_Lysis")
	_dir = os.path.join(_dir, timestr)
	if not os.path.exists(_dir):
		os.makedirs(_dir)
	f = open(os.path.join(_dir, "data_log.csv"), "a")
	f.write("Time, Temperature\n")

	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (12 for lysis, 13 for CEAC), paras = [target, P, I, D]
	# sensorNum 0 for lysis, 1 for CEAC
	heater = wireHeater(12, 0, [73, 30, 2, 40])

	# heat profile in duple of target and duration
	heatProfile = [[65, 60]]

	for stepHeating in heatProfile:
		heater.heatStepSA(stepHeating[0], stepHeating[1], f, queue)

	heater.stop()


def react_detectDemo(queue, rts):
	_dir = "/home/pi/bit/v6board/bit/bitGUImultiProcess/dataArchive"
	timestr = time.strftime("%Y%m%d-%H%M%S_React")
	_dir = os.path.join(_dir, timestr)
	if not os.path.exists(_dir):
		os.makedirs(_dir)
	f = open(os.path.join(_dir, "data_log.csv"), "a")
	f.write("Time,PD1,PD2,PD3,PD4,PD5,Temperature\n")

	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (12 for lysis, 13 for CEAC), paras = [target, P, I, D]
	# sensorNum 0 for lysis, 1 for CEAC
	heater = wireHeater(13, 1, [73, 12, 0.225, 20])

	## Create optics object
	optics = Optical(2, LEDslt = 1, lvl = 0) # Avg from 2 measurements
	darkBackground = optics.read() # Measuring black background of sensors
	optics.dutycycle = 1

	tic = time.time()
	opticsDatas = darkBackground

	while heater.TC_errorR >= 0:
		heater.rampUP()
		timeVal = round(time.time() - tic, 2)
		dataItem = "{},{},{},{},{},{},{}\n".format(timeVal,opticsDatas[0],\
        opticsDatas[1],opticsDatas[2],opticsDatas[3],opticsDatas[4]\
        , heater.currentTemperature)
		f.write(dataItem)
		time.sleep(0.2)

	heater.tstart = time.time()
	heater.TC_sum_error = 680

	counter = 0
	# Get rid of unstable sensor reading at the beginning
	startCnt = 0
	slopLimit = 12 # 12 mv of single increase as Positive
	threshold = np.ones(5, dtype = float) * slopLimit
	datas = []
	volDiffs = [0, 0, 0, 0, 0]

	while heater.timer < 60:
		heater.settling()
		if not queue.empty():
			queue.get()
		queue.put(2)
		opticsDatas = optics.read()

		timeVal = round(time.time() - tic, 2)
		dataItem = "{},{},{},{},{},{},{}\n".format(timeVal,opticsDatas[0],\
        opticsDatas[1],opticsDatas[2],opticsDatas[3],opticsDatas[4]\
        , heater.currentTemperature)
		f.write(dataItem)

		if counter >= startCnt:
			datas.append(opticsDatas)
		preDatas = opticsDatas
		counter += 1
		time.sleep(1.6)

	heater.timer = 0
	datas = np.transpose(np.array(datas))
	for i in range(5):
		# Define sensor reading raise as maximum sum of consecutive diff subarray of size 100
		volDiffs[i] = consecutiveSum(np.diff(smooth(datas[i])), 4)

	results = np.greater_equal(volDiffs, threshold)

	volDiffsItem = "Voltage difference,{},{},{},{},{}\n".format(volDiffs[0],\
        volDiffs[1],volDiffs[2],volDiffs[3],volDiffs[4])
	f.write(volDiffsItem)

	resultsItem = "Result,{},{},{},{},{}\n".format(results[0],\
        results[1],results[2],results[3],results[4])
	f.write(resultsItem)
	heater.stop()
	optics.stop()

	for i, value in enumerate(results):
		rts[i] = value.astype(int)


## Create data transmitter object
TCP_IP = '192.168.1.18'
TCP_PORT = 5500
BUFFER_SIZE = 100

def lysisRun():
	# Connect the socket to the port where the server is listening
	server_address = (TCP_IP, TCP_PORT)
	print ('connecting to %s port %s' % server_address)
	transmitter = TransmitterSingleSocket()

	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (12 for lysis, 13 for CEAC), paras = [target, P, I, D]
	# sensorNum 0 for lysis, 1 for CEAC
	heater = wireHeater(12, 0, [73, 30, 2, 40])

	# heat profile in duple of target and duration
	heatProfile = [[90, 300]]

	for stepHeating in heatProfile:
		heater.heatStep(stepHeating[0], stepHeating[1], transmitter)

	heater.stop()

def react_detectRun():
	# Connect the socket to the port where the server is listening
	server_address = (TCP_IP, TCP_PORT)
	print ('connecting to %s port %s' % server_address)
	transmitter = TransmitterSingleSocket()

	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (12 for lysis, 13 for CEAC), paras = [target, P, I, D]
	# sensorNum 0 for lysis, 1 for CEAC
	heater = wireHeater(13, 1, [73, 12, 0.225, 20])

	## Create optics object
	optics = Optical(2, LEDslt = 1, lvl = 0) # Avg from 2 measurements
	darkBackground = optics.read() # Measuring black background of sensors
	optics.dutycycle = 1

	tic = time.time()
	opticsDatas = darkBackground

	while heater.TC_errorR >= 0:
		heater.rampUP()
		timeVal = round(time.time() - tic, 2)
		transmitter.sendDataPoint(timeVal, opticsDatas, heater.currentTemperature)
		time.sleep(0.2)

	heater.tstart = time.time()
	heater.TC_sum_error = 680

	while heater.timer < 3600:
		heater.settling()
		opticsDatas = optics.read()

		timeVal = round(time.time() - tic, 2)
		transmitter.sendDataPoint(timeVal, opticsDatas, heater.currentTemperature)
		time.sleep(1.6)

	heater.timer = 0
	heater.stop()
	optics.stop()
	#transmitter.signalEnd()
	#transmitter.close()

if __name__=='__main__':
	## Create data transmitter object
	#TCP_IP = '192.168.1.18'
	#TCP_PORT = 5500
	#BUFFER_SIZE = 100
	# Connect the socket to the port where the server is listening
	#server_address = (TCP_IP, TCP_PORT)
	#print ('connecting to %s port %s' % server_address)
	#transmitter = TransmitterSingleSocket()
	## Create heater object
	# Differential pair Channel A + and Channel B - at Gain 1x
	# pwm pinNum (18 for lysis, 12 for CEAC), paras = [target, P, I, D]
	#heater = wireHeater(18, [73, 30, 2, 40])

	## Create optics object
	optics = Optical(2, LEDslt = 1, lvl = 1) # Avg from 2 measurements

	tic = time.time()

	try:
		# Probe testing
		tic = time.time()
		while 1:
			print(optics.read())
			print("mv")
			print("time(s)=",round(time.time() - tic, 2))
			time.sleep(1.6)

	except KeyboardInterrupt:
		#heater.stop()
		optics.stop()
		#transmitter.signalEnd()
		#transmitter.close()

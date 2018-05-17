import RPi.GPIO as GPIO
import time
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()

GAIN = 1

E  = 17
s0 = 18
s1 = 27
s2 = 22
s3 = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(E, GPIO.OUT)

#headers
print ('LED | D1 | D2 | D3 | D4'.format(*range(5)))
print '-----', '-----', '-----', '-----', '-----'

def led_1():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.LOW)
	GPIO.output(s1, GPIO.LOW)
	GPIO.output(s2, GPIO.LOW)
	GPIO.output(s3, GPIO.LOW)
	
def led_2():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.HIGH)
	GPIO.output(s1, GPIO.LOW)
	GPIO.output(s2, GPIO.LOW)
	GPIO.output(s3, GPIO.LOW)
	
def led_3():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.LOW)
	GPIO.output(s1, GPIO.HIGH)
	GPIO.output(s2, GPIO.LOW)
	GPIO.output(s3, GPIO.LOW)
	
def led_4():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.HIGH)
	GPIO.output(s1, GPIO.HIGH)
	GPIO.output(s2, GPIO.LOW)
	GPIO.output(s3, GPIO.LOW)
	
def led_5():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.LOW)
	GPIO.output(s1, GPIO.LOW)
	GPIO.output(s2, GPIO.HIGH)
	GPIO.output(s3, GPIO.LOW)
	
def led_6():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.HIGH)
	GPIO.output(s1, GPIO.LOW)
	GPIO.output(s2, GPIO.HIGH)
	GPIO.output(s3, GPIO.LOW)
	
def led_7():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.LOW)
	GPIO.output(s1, GPIO.HIGH)
	GPIO.output(s2, GPIO.HIGH)
	GPIO.output(s3, GPIO.LOW)
	
def led_8():
	GPIO.output(E, GPIO.LOW)
	GPIO.output(s0, GPIO.HIGH)
	GPIO.output(s1, GPIO.HIGH)
	GPIO.output(s2, GPIO.HIGH)
	GPIO.output(s3, GPIO.LOW)
	
	
def turnoff():
	GPIO.output(E, GPIO.HIGH)

def adc_read():
	value1 = adc.read_adc(0, GAIN)
	voltage1 = (value1*0.000125)
	value2 = adc.read_adc(1, GAIN)
	voltage2 = (value2*0.000125)
	value3 = adc.read_adc(2, GAIN)
	voltage3 = (value3*0.000125)
	value4 = adc.read_adc(3, GAIN)
	voltage4 = (value4*0.000125)
	return voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

if __name__ == "__main__":
	try:
		while 1:
			led_1()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led1", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)

			led_2()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led2", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)
			
			led_3()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led3", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)

			led_4()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led4", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)
			
			led_5()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led5", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)

			led_6()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led6", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)
			
			led_7()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led7", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)

			led_8()
			time.sleep(0.25)
			#readADC
			value1 = adc.read_adc(0, GAIN)
			voltage1 = (value1*0.000125)
			value2 = adc.read_adc(1, GAIN)
			voltage2 = (value2*0.000125)
			value3 = adc.read_adc(2, GAIN)
			voltage3 = (value3*0.000125)
			value4 = adc.read_adc(3, GAIN)
			voltage4 = (value4*0.000125)
			print "led8", voltage1, voltage2, voltage3, voltage4 #data coming from LED1 collected by all detectors

			turnoff()
			time.sleep(0.05)

	except KeyboardInterrupt:
		GPIO.cleanup()

print("done")
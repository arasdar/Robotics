//#include "wiringPi.h"
#include <wiringPi.h>

////#include "Adafruit_ADS1015_2.h"
////#include 'projects/Gozdeh/Adafruit_ADS1015_2.h'
//#include "projects/Gozdeh/Adafruit_ADS1015_2.h"

//Adafruit_ADS1115 ads(0x48);
//
//int E  = 17;
//int s0 = 18;
//int s1 = 27;
//int s2 = 22;
//int s3 = 23;
//
//int16_t adc0, adc1, adc2, adc3;
//
//float voltage1, voltage2, voltage3, voltage4;

int main()
{
//	wiringPiSetup();
//	ads.begin();
//	pinMode(s0, OUTPUT);
//	pinMode(s1, OUTPUT);
//	pinMode(s2, OUTPUT);
//	pinMode(s3, OUTPUT);
//	pinMode(E, OUTPUT);
	
//	while (1)
//	{
//		firstsource();
//	    delay(20);
//	    adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		secondsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		thirdsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		fourthsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		fifthsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		sixthsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		seventhsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//
//		eighthsource();
//		delay(20);
//		adc0 = ads.readADC_SingleEnded(0);
//		voltage1 = (adc0 * 0.1875)/1000;
//		adc1 = ads.readADC_SingleEnded(0);
//		voltage2 = (adc1 * 0.1875)/1000;
//		adc2 = ads.readADC_SingleEnded(0);
//		voltage3 = (adc2 * 0.1875)/1000;
//		adc3 = ads.readADC_SingleEnded(0);
//		voltage4 = (adc3 * 0.1875)/1000;
//		sendDataToSerial("A", voltage1);
//		sendDataToSerial("B", voltage2);
//		sendDataToSerial("C", voltage3);
//		sendDataToSerial("D", voltage4);
//		printf("\n");
//
//		nochannel();
//		delay(5);
//	}// while
}// main

//
//int sendDataToSerial(char symbol, float data)
//{
//	printf("%s", symbol);
//	printf("%2.2f\n", data);
//}
//
//int firstsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, LOW);
//	digitalWrite(s1, LOW);
//	digitalWrite(s2, LOW);
//	digitalWrite(s3, LOW);
//}
//
//int secondsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, HIGH);
//	digitalWrite(s1, LOW);
//	digitalWrite(s2, LOW);
//	digitalWrite(s3, LOW);
//}
//
//int thirdsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, LOW);
//	digitalWrite(s1, HIGH);
//	digitalWrite(s2, LOW);
//	digitalWrite(s3, LOW);
//}
//
//int fourthsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, HIGH);
//	digitalWrite(s1, HIGH);
//	digitalWrite(s2, LOW);
//	digitalWrite(s3, LOW);
//}
//
//int fifthsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, LOW);
//	digitalWrite(s1, LOW);
//	digitalWrite(s2, HIGH);
//	digitalWrite(s3, LOW);
//}
//
//int sixthsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, HIGH);
//	digitalWrite(s1, LOW);
//	digitalWrite(s2, HIGH);
//	digitalWrite(s3, LOW);
//}
//
//int seventhsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, LOW);
//	digitalWrite(s1, HIGH);
//	digitalWrite(s2, HIGH);
//	digitalWrite(s3, LOW);
//}
//
//int eighthsource()
//{
//	digitalWrite(E, LOW);
//	digitalWrite(s0, HIGH);
//	digitalWrite(s1, HIGH);
//	digitalWrite(s2, HIGH);
//	digitalWrite(s3, LOW);
//}
//
//int nochannel()
//{
//	digitalWrite(E, HIGH);
//}

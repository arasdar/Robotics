// Compile with
// g++ -lrt -lwiringPi -lpthread -lcrypt -o DCMotorTest PWM.cpp Adafruit_MotorHAT.cpp DCMotorTest.cpp

// Run with 
// ./DCMotorTest

#include "stdio.h"
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstdlib>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <string.h>
#include "projects/AdafruitStepperMotorHAT/Adafruit_MotorHAT.h"

void error(char *msg)
{
  perror(msg);
  exit(1);
}

void sendData(int sockfd, int x)
{
  int n;
  char buffer[32];
  sprintf( buffer, "%d\n", x );
  if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
    error( const_cast<char *>( "ERROR writing to socket") );
  buffer[n] = '\0';
}

int getData( int sockfd )
{
  char buffer[32];
  int n;

  if ( (n = read(sockfd,buffer,31) ) < 0 )
    error( const_cast<char *>( "ERROR reading from socket") );
  buffer[n] = '\0';
  return atoi( buffer );
}

Adafruit_MotorHAT hat;

void ctrl_c_handler(int s)
{
  std::cout << "Caught signal " << s << std::endl;
  hat.resetAll();
  exit(1); 
}

int main(int argc,char ** argv)
{
  signal(SIGINT, ctrl_c_handler);
  Adafruit_DCMotor& myMotor1 = hat.getDC(1);
  Adafruit_DCMotor& myMotor2 = hat.getDC(2);

  int sockfd, newsockfd, portno = 44444, clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  int data;

  printf( "using port #%d\n", portno );

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
     error( const_cast<char *>("ERROR opening socket") );
  bzero((char *) &serv_addr, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons( portno );
  if (bind(sockfd, (struct sockaddr *) &serv_addr,
           sizeof(serv_addr)) < 0)
     error( const_cast<char *>( "ERROR on binding" ) );
  listen(sockfd,5);
  clilen = sizeof(cli_addr);

  //--- infinite wait on a connection ---
  
  printf( "waiting for new client...\n" );
  if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
      error( const_cast<char *>("ERROR on accept") );
  printf( "opened new communication with client\n" );

    //---- wait for a number from client ---

  while (true)
    {
      int inByte = getData( newsockfd );
      printf( "got %d\n", inByte);
      if ( inByte < 0 )
         //break;
        close( newsockfd );

        //--- if -2 sent by client, we can quit ---
        if ( inByte == -2 )
        break;

        switch(inByte)
        {
            case 1: //Moving Forward
            {
              myMotor1.run(FORWARD);
              myMotor2.run(FORWARD);

              //Serial.println("case 1 activated.");

              for (int i=0; i<20; i++) {
                myMotor1.setSpeed(250);
                myMotor2.setSpeed(250);
                delay(10);
              }
              break;
             }

             case 2: //Moving Backward
             {
              myMotor1.run(BACKWARD);
              myMotor2.run(BACKWARD);

              //Serial.println("case 2 activated.");

              for (int i=0; i!=20; i++)
                {
                  myMotor1.setSpeed(250);
                  myMotor2.setSpeed(250);
                  delay(10);
                }
              break;
              }

             case 3: //Turn left
             {
              myMotor1.run(FORWARD);
              //myMotor2.run(FORWARD);

              //Serial.println("case 3 activated.");

              for (int i=0; i!=20; i++)
                {
                  myMotor1.setSpeed(250);
                  myMotor2.setSpeed(250);
                  delay(10);
                }
              break;
              }

             case 4: //Turn right
             {
              //myMotor1.run(FORWARD);
              myMotor2.run(FORWARD);

              //Serial.println("case 4 activated.");

              for (int i=0; i!=20; i++)
                {
                  myMotor1.setSpeed(250);
                  myMotor2.setSpeed(250);
                  delay(10);
                }
              break;
              }

             case 7: //360 CCW
             {
              myMotor1.run(FORWARD);
              myMotor2.run(BACKWARD);

              //Serial.println("case 7 activated.");

              for (int i=0; i!=20; i++)
                {
                  myMotor1.setSpeed(250);
                  myMotor2.setSpeed(250);
                  delay(10);
                }
              break;
              }

             case 8: //360 CW
             {
              myMotor1.run(BACKWARD);
              myMotor2.run(FORWARD);

              //Serial.println("case 8 activated.");

              for (int i=0; i!=20; i++)
                {
                  myMotor1.setSpeed(250);
                  myMotor2.setSpeed(250);
                  delay(10);
                }
              break;
              }
            }

        //inByte = 0;
        myMotor1.setSpeed(0);
        myMotor2.setSpeed(0);

        myMotor1.run(RELEASE);
        myMotor2.run(RELEASE);

    delay(100);
        }

  return 0;
}


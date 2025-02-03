#include <arduino.h>
#include "UC6580_Config.h"

UCCONFIG::UCCONFIG(){}

void UCCONFIG::AutoConfig()
{
  //Serial1.print("$CFGPRT,,115200,,\r\n"); // Optional: Set GPS baud rate to 115200
  //Serial1.write("$CFGNAV,100,100,100\r\n");      // Set navigation rate to 10 Hz (XX is the checksum, needs calculating)
  
  //while (Serial1.available()) {
  //  char c = Serial1.read();
  //  Serial.write(c);
  //}
  Serial1.write("$CFGNAV\r\n");
  delay(250);
  //while (Serial1.available()) {
  //  char c = Serial1.read();
  //  Serial.write(c);
  //}
  Serial1.write("$CFGSYS,h35155\r\n");
  delay(750);
  // Disable  messages
  Serial1.write("$CFGMSG,0,0,1\r\n");   // Enable GGA
  delay(250);
  Serial1.write("$CFGMSG,0,1,0\r\n");   // Disable GLL
  delay(250);
  Serial1.write("$CFGMSG,0,2,0\r\n");   // Disable GSA
  delay(250);
  Serial1.write("$CFGMSG,0,3,0\r\n");   // Disable GSV
  delay(250);
  Serial1.write("$CFGMSG,0,4,0\r\n");   // Disable RMC
  delay(250);
  Serial1.write("$CFGMSG,0,5,0\r\n");   // Disable VTG
  delay(250);
  Serial1.write("$CFGMSG,0,6,0\r\n");   // Disable ZDA
  delay(250);
  Serial1.write("$CFGMSG,0,7,0\r\n");   // Disable GST
  delay(250);
  Serial1.write("$CFGMSG,0,8,0\r\n");   // Disable GBS
  delay(250);
  Serial1.write("$CFGMSG,6,0,0\r\n");
  delay(250);
  Serial1.write("$CFGMSG,6,1,0\r\n");
  delay(250);
}
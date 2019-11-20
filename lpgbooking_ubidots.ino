#include "UbidotsMicroESP8266.h"
#define TOKEN  "A1E-dTbpqZbAFHsXRTucWazsckOxwAPiGM"  // Put here your Ubidots TOKEN
#define WIFISSID "project" // Put here your Wi-Fi SSID
#define PASSWORD "12345678" // Put here your Wi-Fi password
Ubidots client(TOKEN);
char rdata = '0';
String rxd = " ";
void setup()
{
  Serial.begin(9600);
  client.wifiConnection(WIFISSID, PASSWORD);
  //client.setDebug(true); // Uncomment this line to set DEBUG on
  Serial.println("Waiting for serial data....");
}
void loop()
{
  while(Serial.available() == 0) //wait for serial data
  {
    Serial.print('.');
    delay(100);
  }
  /*
  while(rdata != '*')
  {
    rdata = Serial.read();
  }
  rxd = rdata;
  while(rdata != '#')
  {
    rdata = Serial.read();
    rxd += rdata;
  }*/
  //*50.00/30.00/31.05#
  //*Gas/Fire/Weight#
  rxd = Serial.readStringUntil('#');
  //rxd = rxd+'#';
  Serial.println(rxd);
  rxd.replace('\n',' ');
  rxd.replace('\r',' ');
  rxd.trim();
  int gstart = rxd.indexOf('*');
  int gend = rxd.indexOf('/');
  int fend = rxd.indexOf('/',gend+1);
  //int wend = rxd.indexOf('#');
  String gstr = rxd.substring(gstart+1,gend); //Gas
  String fstr = rxd.substring(gend+1,fend);   //Fire
  String wstr = rxd.substring(fend+1);        //Weight
  float gas = gstr.toFloat();
  float fire = fstr.toFloat();
  float weight = wstr.toFloat();
  if(gas>0)// && fire>0 && weight>0)
  {
    Serial.println("Gas: "+String(gas));
    Serial.println("Fire: "+String(fire));
    Serial.println("Weight: "+String(weight));
    client.add("Gas",gas);
    client.add("Weight",weight);
    client.add("Fire",fire);
    client.sendAll(true);
    //delay(5000);
  }
  rxd = ' ';
}

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>3
#include <OSCMessage.h>
#include <Wire.h>
#include "VL53L1X.h"

// WIFI
#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

VL53L1X Distance_Sensor;

WiFiUDP Udp;

//the Arduino's IP
IPAddress ip(192, 168, 1, 11);
//destination IP
IPAddress outIp(192, 168, 1, 2);

const unsigned int outPort = 7000;    // local port to listen on

int count = 0;

void setup() {

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  Udp.begin(8080);

  VL53L1X Distance_Sensor;

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");

  Distance_Sensor.setTimeout(500);
  if (!Distance_Sensor.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor!");
    while (1);
  }

  Distance_Sensor.setDistanceMode(VL53L1X::Long);
  Distance_Sensor.setMeasurementTimingBudget(50000);
  Distance_Sensor.startContinuous(50);
}


void loop() {

  //delay(100);
  //printCurrentNet();
  
  Distance_Sensor.read();
  Serial.print("Distance(mm): ");
  Serial.print(Distance_Sensor.ranging_data.range_mm);
  Serial.println();
  int d = Distance_Sensor.ranging_data.range_mm;
  
  //the message wants an OSC address as first argument
  OSCMessage msg("/dist2");
  //OSCMessage msg("");
  msg.add(d);
  
  Udp.beginPacket(outIp, outPort);
    msg.send(Udp); // send the bytes to the SLIP strea
  Udp.endPacket(); // mark the end of the OSC Packet
  
  msg.empty(); // free space occupied by message  
  delay(100);

  count++;

  /*Serial.print("working");*/
  

}

/*
  Copyright ® 2019 August devMobile Software, All Rights Reserved

  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
  KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
  PURPOSE.

  You can do what you want with this code, acknowledgment would be nice.

  http://www.devmobile.co.nz

*/
#include <stdlib.h>
#include <LoRa.h>
#include <sha204_library.h>
#include "Seeed_HM330X.h"

//#define DEBUG
//#define DEBUG_TELEMETRY
//#define DEBUG_LORA

const byte SensorPayloadLength = 28 ;
const byte SensorPayloadBufferSize  = 29 ;
const byte SensorPayloadPM1_0Position = 4;
const byte SensorPayloadPM2_5Position = 6;
const byte SensorPayloadPM10_0Position = 8;

HM330X sensor;
byte SensorPayload[SensorPayloadBufferSize];
  
// LoRa field gateway configuration (these settings must match your field gateway)
const byte DeviceAddressMaximumLength = 15 ;
const char FieldGatewayAddress[] = {"LoRaIoT1"};
const float FieldGatewayFrequency =  915000000.0;
const byte FieldGatewaySyncWord = 0x12 ;

// Payload configuration
const int ChipSelectPin = 10;
const int ResetPin = 9;
const int InterruptPin = 2;

// LoRa radio payload configuration
const byte SensorIdValueSeperator = ' ' ;
const byte SensorReadingSeperator = ',' ;
const unsigned long SensorUploadDelay = 60000;

// ATSHA204 secure authentication, validation with crypto and hashing (currently only using for unique serial number)
const byte Atsha204Port = A3;
atsha204Class sha204(Atsha204Port);
const byte DeviceSerialNumberLength = 9 ;
byte deviceSerialNumber[DeviceSerialNumberLength] = {""};

const byte PayloadSizeMaximum = 64 ;
byte payload[PayloadSizeMaximum];
byte payloadLength = 0 ;


void setup()
{
  Serial.begin(9600);

#ifdef DEBUG
  while (!Serial);
#endif
 
  Serial.println("Setup called");

  Serial.print("Field gateway:");
  Serial.print(FieldGatewayAddress ) ;
  Serial.print(" Frequency:");
  Serial.print( FieldGatewayFrequency,0 ) ;
  Serial.print("MHz SyncWord:");
  Serial.print( FieldGatewaySyncWord ) ;
  Serial.println();
  
   // Retrieve the serial number then display it nicely
  if(sha204.getSerialNumber(deviceSerialNumber))
  {
    Serial.println("sha204.getSerialNumber failed");
    while (true); // Drop into endless loop requiring restart
  }

  Serial.print("SNo:");
  DisplayHex( deviceSerialNumber, DeviceSerialNumberLength);
  Serial.println();

  Serial.println("LoRa setup start");

  // override the default chip select and reset pins
  LoRa.setPins(ChipSelectPin, ResetPin, InterruptPin);
  if (!LoRa.begin(FieldGatewayFrequency))
  {
    Serial.println("LoRa begin failed");
    while (true); // Drop into endless loop requiring restart
  }

  // Need to do this so field gateway pays attention to messsages from this device
  LoRa.enableCrc();
  LoRa.setSyncWord(FieldGatewaySyncWord);

#ifdef DEBUG_LORA
  LoRa.dumpRegisters(Serial);
#endif
  Serial.println("LoRa Setup done.");

  // Configure the Seeedstudio CO2, temperature & humidity sensor
  Serial.println("HM3301 setup start");
  if(sensor.init())
  {
    Serial.println("HM3301 init failed");
    while (true); // Drop into endless loop requiring restart
  
  }
  delay(100);
  Serial.println("HM3301 setup done");

  PayloadHeader((byte *)FieldGatewayAddress,strlen(FieldGatewayAddress), deviceSerialNumber, DeviceSerialNumberLength);

  Serial.println("Setup done");
  Serial.println();
}


void loop()
{
  unsigned long currentMilliseconds = millis();  
  byte sum=0;
  short pm1_0 ;
  short pm2_5 ;
  short pm10_0 ;

  Serial.println("Loop called");

  if(sensor.read_sensor_value(SensorPayload,SensorPayloadBufferSize) == NO_ERROR)
  {
    // Calculate then validate the payload "checksum"
    for(int i=0;i<SensorPayloadLength;i++)
    {
        sum+=SensorPayload[i];
    }
    if(sum!=SensorPayload[SensorPayloadLength])
    {
        Serial.println("Invalid checksum");
        return;
    }    

    PayloadReset();
    
    pm1_0 = (u16)SensorPayload[SensorPayloadPM1_0Position]<<8|SensorPayload[SensorPayloadPM1_0Position+1];
    Serial.print("PM1.5: ");
    Serial.print(pm1_0);
    Serial.println("ug/m3 ") ;

    PayloadAdd( "P10", pm1_0, false);
    
    pm2_5 = (u16)SensorPayload[SensorPayloadPM2_5Position]<<8|SensorPayload[SensorPayloadPM2_5Position+1];
    Serial.print("PM2.5: ");
    Serial.print(pm2_5);
    Serial.println("ug/m3 ") ;

    PayloadAdd( "P25", pm2_5, 1, false);

    pm10_0 = (u16)SensorPayload[SensorPayloadPM10_0Position]<<8|SensorPayload[SensorPayloadPM10_0Position+1];
    Serial.print("PM10.0: ");
    Serial.print(pm10_0);
    Serial.println("ug/m3 ");

    PayloadAdd( "P100", pm10_0, 0, true) ;

    #ifdef DEBUG_TELEMETRY
      Serial.println();
      Serial.print("RFM9X/SX127X Payload length:");
      Serial.print(payloadLength);
      Serial.println(" bytes");
    #endif

    LoRa.beginPacket();
    LoRa.write(payload, payloadLength);
    LoRa.endPacket();
  }
  Serial.println("Loop done");
  Serial.println();
  
  delay(SensorUploadDelay - (millis() - currentMilliseconds ));
}


void PayloadHeader( const byte *to, byte toAddressLength, const byte *from, byte fromAddressLength)
{
  byte addressesLength = toAddressLength + fromAddressLength ;

  payloadLength = 0 ;

  // prepare the payload header with "To" Address length (top nibble) and "From" address length (bottom nibble)
  
  payload[payloadLength] = (toAddressLength << 4) | fromAddressLength ;
  payloadLength += 1;

  // Copy the "To" address into payload
  memcpy(&payload[payloadLength], to, toAddressLength);
  payloadLength += toAddressLength ;

  // Copy the "From" into payload
  memcpy(&payload[payloadLength], from, fromAddressLength);
  payloadLength += fromAddressLength ;
}


void PayloadAdd( const char *sensorId, float value, byte decimalPlaces, bool last)
{
  byte sensorIdLength = strlen( sensorId ) ;

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( dtostrf(value, -1, decimalPlaces, (char *)&payload[payloadLength]));
  if (!last)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
  
#ifdef DEBUG_TELEMETRY
  Serial.print("PayloadAdd float-payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadAdd( char *sensorId, int value, bool last )
{
  byte sensorIdLength = strlen(sensorId) ;

  memcpy(&payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen(itoa( value,(char *)&payload[payloadLength],10));
  if (!last)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
  
#ifdef DEBUG_TELEMETRY
  Serial.print("PayloadAdd int-payloadLength:" );
  Serial.print(payloadLength);
  Serial.println( );
#endif
}


void PayloadAdd( char *sensorId, unsigned int value, bool last )
{
  byte sensorIdLength = strlen(sensorId) ;

  memcpy(&payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen(utoa( value,(char *)&payload[payloadLength],10));
  if (!last)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
  
#ifdef DEBUG_TELEMETRY
  Serial.print("PayloadAdd uint-payloadLength:");
  Serial.print(payloadLength);
  Serial.println( );
#endif
}


void PayloadReset()
{
  byte fromAddressLength = payload[0] & 0xf ;
  byte toAddressLength = payload[0] >> 4 ;
  
  payloadLength = toAddressLength + fromAddressLength + 1;
}


void DisplayHex( byte *byteArray, byte length) 
{
  for (int i = 0; i < length ; i++)
  {
    // Add a leading zero
    if ( byteArray[i] < 16)
    {
      Serial.print("0");
    }
    Serial.print(byteArray[i], HEX);
    if ( i < (length-1)) // Don't put a - after last digit
    {
      Serial.print("-");
    }
  }
}    

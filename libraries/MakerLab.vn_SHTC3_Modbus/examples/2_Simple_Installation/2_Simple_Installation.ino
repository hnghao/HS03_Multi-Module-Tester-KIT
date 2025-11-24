/**
 * This program code allows you to configure 2 important parameters of the "SHTC3 RS485 Modbus RTU".
 * That are "Address" and "Baudrate" parameters of the sensor.
 *
 * When opening the "Serial Monitor" of the Arduino IDE.
 * Please choose Baudrate "115200" and "No line ending".
 */

#include "SHTC3Modbus.h"

/* ------------------------------------------------------------------------- */

#define RX_SHT 2
#define TX_SHT 3

SHTC3 demo(RX_SHT, TX_SHT, BAUD_4800, 0x01);

String inputData = "";

/* ------------------------------------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  Serial.println(F("-----------------------------------------------"));
  Serial.println(F("Start configuration SHTC3 Modbus (HSHOP)"));
  Serial.println(F("Please choose \"No line ending\""));
  Serial.println("");
  Serial.println(F("Enter [0] to init the sensor with Baudrate 2400"));
  Serial.println(F("Enter [1] to init the sensor with Baudrate 4800"));
  Serial.println(F("Enter [2] to init the sensor with Baudrate 9600"));
  Serial.println("");
  Serial.println(F("Enter [A] to check Address of sensor"));
  Serial.println("");
  Serial.println(F("Enter [A---] to set value Address for sensor"));
  Serial.println(F("With --- is the value DEC, range from [001-254]"));
  Serial.println("");
  Serial.println(F("Enter [B] to check Baudrate of sensor"));
  Serial.println("");
  Serial.println(F("Enter [B0] to set Baudrate 2400"));
  Serial.println(F("Enter [B1] to set Baudrate 4800"));
  Serial.println(F("Enter [B2] to set Baudrate 9600"));
  Serial.println(F("-----------------------------------------------"));
  Serial.println("");

  demo.begin(4800);
}

void loop()
{
  Serial.println(F("-----------------"));
  Serial.println(F("Enter command ..."));
  Serial.println("");

  while (Serial.available() == 0)
  {
  }                                      // Wait for the user to input command
  inputData = Serial.readString();       // Read the string of data you just entered from the Serial Port
  int lengthOfData = inputData.length(); // Save the length of the data string just entered (total characters sent)
  Serial.println("-> " + inputData);     // Show what you just typed

  if (inputData == "A")
  {
    Serial.print(F("Address: "));
    Serial.println(demo.readAddress());
  }
  else if (inputData[0] == 'A' && lengthOfData == 4)
  {
    uint8_t address = inputData.substring(1).toInt();
    if (address >= 1 && address <= 254)
    {
      if (demo.setAddress(address))
      {
        Serial.print(F("OK 0X"));
        Serial.println(address, HEX);
      }
      else
      {
        Serial.println(F("FAIL"));
      }
    }
    else
    {
      Serial.println(F("Invalid Address"));
    }
  }
  else if (inputData == "0")
  {
    demo.begin(2400);
    Serial.println(F("Init Baud 2400"));
  }
  else if (inputData == "1")
  {
    demo.begin(4800);
    Serial.println(F("Init Baud 4800"));
  }
  else if (inputData == "2")
  {
    demo.begin(9600);
    Serial.println(F("Init Baud 9600"));
  }
  else if (inputData == "B")
  {
    Serial.print(F("Baudrate: "));
    Serial.println(demo.readBaudrate());
  }
  else if (inputData == "B0")
  {
    if (demo.setBaudrate(BAUD_2400))
    {
      Serial.println(F("OK 2400"));
    }
    else
    {
      Serial.println(F("FAIL"));
    }
  }
  else if (inputData == "B1")
  {
    if (demo.setBaudrate(BAUD_4800))
    {
      Serial.println(F("OK 4800"));
    }
    else
    {
      Serial.println(F("FAIL"));
    }
  }
  else if (inputData == "B2")
  {
    if (demo.setBaudrate(BAUD_9600))
    {
      Serial.println(F("OK 9600"));
    }
    else
    {
      Serial.println(F("FAIL"));
    }
  }
  else
  {
    Serial.println("");
  }

  inputData = ""; // Delete the string data just entered
}

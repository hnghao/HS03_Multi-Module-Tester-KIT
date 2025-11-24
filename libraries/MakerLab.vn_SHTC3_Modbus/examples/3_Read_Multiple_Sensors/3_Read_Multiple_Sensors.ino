/**
 * This is a simple demo code that reads the Temperature and Humidity.
 * Of the 3 sensors SHTC3 at the same time on bus RS485 via UART.
 *
 * Note, it is necessary to configure 3 sensors in advance with the following parameters:
 * _ Sensor 1 (Baudrate 4800bps, Address 0x01)
 * _ Sensor 2 (Baudrate 2400bps, Address 0x02)
 * _ Sensor 3 (Baudrate 9600bps, Address 0x0A)
 */

#include "SHTC3Modbus.h"

/* ------------------------------------------------------------------------- */

#define RX_SHT 2
#define TX_SHT 3

/* Using with SoftwareSerial, default (RX=2) and (TX=3) */
SHTC3 demo_1(RX_SHT, TX_SHT, BAUD_4800, 0x01);
SHTC3 demo_2(RX_SHT, TX_SHT, BAUD_2400, 0x02);
SHTC3 demo_3(RX_SHT, TX_SHT, BAUD_9600, 0x0A);

dataSHTC3 data;

/* ------------------------------------------------------------------------- */

void setup()
{
  /* ----------------------- Test with SoftwareSerial ---------------------- */
  Serial.begin(115200);
  Serial.println(F("Start reading SHTC3 Modbus (HSHOP)"));
}

void loop()
{
  /* ---------------------------- Read Sensor 1 ---------------------------- */
  demo_1.begin(4800);
  data = demo_1.getData();

  Serial.print(F("Sensor 1: "));
  Serial.print(data.temperatureC);
  Serial.print(F("°C\t"));
  Serial.print(data.humidity);
  Serial.println(F("%"));

  delay(250);

  /* ---------------------------- Read Sensor 2 ---------------------------- */
  demo_2.begin(2400);
  data = demo_2.getData();

  Serial.print(F("Sensor 2: "));
  Serial.print(data.temperatureC);
  Serial.print(F("°C\t"));
  Serial.print(data.humidity);
  Serial.println(F("%"));

  delay(250);

  /* ---------------------------- Read Sensor 3 ---------------------------- */
  demo_3.begin(9600);
  data = demo_3.getData();

  Serial.print(F("Sensor 3: "));
  Serial.print(data.temperatureC);
  Serial.print(F("°C\t"));
  Serial.print(data.humidity);
  Serial.println(F("%"));
  Serial.println(F("--- --- ---"));

  delay(1500);
}

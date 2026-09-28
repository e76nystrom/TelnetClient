// Device connected to stationary GPS is a TCP client
// Device connected to moveable GPS is a TCP server
// moveable GPS starts and waits for stationary GPS to connect to it
// staicGPS -> RTKData -> client -> LAN or Wifi -> server -> RTKData -> movableGPS

#define WIRED_LAN
#include "cfg.h"
#include <Arduino.h>
#include "wired.h"

// #include <SPI.h>
// #define _ETHERNET_WEBSERVER_LOGLEVEL_ 4 // NOLINT(*-reserved-identifier)
// #include "Ethernet_GenericX.h"
// #include <utility/w5100.h>
// #include "FreeRTOS.h"

constexpr int uxTopUsedPriority = configMAX_PRIORITIES - 1;

#define RESTART
#define TFT

#if defined(TFT)

#include "TFT.h"
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip

#if !defined(ST7735_DRIVER)
#error "ST7735_DRIVER is not defined"
#endif

#if TFT_DC   != 16
#error "TFT_DC != 16"
#endif

#if TFT_MOSI != 17
#error "TFT_MOSI != 17"
#endif

#if TFT_SCLK != 18
#error "TFT_SCLK != 18"
#endif

#if TFT_CS   != 21
#error "TFT_CS != 21"
#endif

#endif	/* TFT */

#define RX_PIN 39		/* serial 2 rx pin */
#define TX_PIN 40		/* serial 2 tx pin */

#define GPS_LIB

#if defined(GPS_LIB)

#include "dbgPin.h"
#include "gpsLib.h"

#endif	/* GPS_LIB */

#if defined(TFT)

#define TFT_GREY 0x5AEB // New color

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
int fontHeight;
int fontWidth;

#endif

void setup()
{
 buildCRC24qTable();

 Serial.begin(115200);

 dbgInit();
 
 while (!Serial && millis() < 500)
 {}

 Serial.println("Serial testing");
 puts("puts testing");

 int i = 20;
 while (i > 0)
 {
  if (i % 10 == 0)
    Serial.printf("%d", i / 10);
  Serial.print('.');
  Serial.flush();
  delay(100);
  i -= 1;
 }
 Serial.println('0');

#if defined(SERVER)
 puts("server started connect to remote gps");
#endif
#if defined(CLIENT)
 puts("client started connect to reference gps");
#endif

 Serial2.setRxBufferSize(1600);
 Serial2.setTxBufferSize(1600);
 Serial2.begin(115200, SERIAL_8N1, 39, 40); // rxPin, txPin
 dbg0Set();
 printf("UART2 initialized rx %d tx %d\n", RX_PIN, TX_PIN);
 dbg0Clr();
 dbg1Set();
 Serial2.printf("started\n\r");
 dbg1Clr();

#if defined(WIRED_LAN)
 wiredReset();
#endif	/* WIRED_LAN */

#if defined(TFT)
 dbg0Set();
 printf("setup running on Core: %d\n", xPortGetCoreID());

 printf("TFT_DC %d TFT_MOSI %d TFT_SCLK %d TFT_CS %d\n",
	TFT_DC, TFT_MOSI, TFT_SCLK, TFT_CS);

 tft.init();

 #define FONT 1

 tft.fillScreen(TFT_BLACK);
 tft.setCursor(0, 0, FONT);
 fontHeight = tft.fontHeight() + 1;
 fontWidth = tft.textWidth("0", FONT);
 printf("font height %d\n", fontHeight), 
 tft.setTextColor(TFT_RED, TFT_BLACK);
 tft.setTextSize(1);
 tft.println("Initializing");
 dbg0Clr();
#endif	/* TFT */

#if defined(WIRED_LAN)

#if defined(SERVER)
 wiredInit(SERVER_NAME);
#endif	/* SERVER */

#if defined(CLIENT)
 wiredInit(CLIENT_NAME);
#endif	/* CLIENT */

#endif	/* WIRED_LAN */

#if defined(TFT)
 tft.setCursor(0, 0);
 int yPos = 0;
 int xPos = tft.drawString(wifiHostName, 0 ,yPos);
 xPos += tft.drawString(" ", xPos ,yPos);
 tft.drawString(ipAddress, xPos ,0);
#endif	/* TFT */
 
#if defined(WIRED_LAN)
 
 wiredStart();

#endif	/* WIRED_LAN */

 while (Serial2.available())
  Serial2.read();
}

void loop()
{

#if defined(TFT)
 static unsigned int tmr0;
 unsigned int t0 = millis();
 if ((t0 - tmr0) > 1000)
 {
  tmr0 = t0;

  const float temp = temperatureRead();

  char buf[20];
  snprintf(buf, sizeof(buf), "%4.1f ", temp);
  tft.setCursor(0, static_cast<int16_t>(1 * fontHeight));
  tft.print(buf);
 }

 if (gpsInfo.update)
 {
  gpsInfo.update = false;
  char buf[20];
  tft.drawString(gpsInfo.timeBuf, 0, 2 * fontHeight, FONT);
  snprintf(buf, sizeof(buf), "%d %2d   ", gpsInfo.fix, gpsInfo.sats);
  tft.drawString(buf, 6 * fontWidth, 2 * fontHeight);

  snprintf(buf, sizeof(buf), " %13.10f", gpsInfo.lat);
  tft.drawString(buf, 0, 3 * fontHeight);
  snprintf(buf, sizeof(buf), "%14.10f", gpsInfo.lon);
  tft.drawString(buf, 0, 4 * fontHeight);
 }

#endif	/* TFT */


#if defined(SERVER)

 if (rtk.ser.state != RCV_IDLE)
 {
  if ((millis() - rtk.ser.t) > 100)
  {
   rtk.ser.state = RCV_IDLE;
   puts("receive timeout");
  }
 }

 wiredData();

#if defined(GPS_LIB)

 pollSerial();
 processSerial();

#else

//  while (Serial2.available())
//  {
//   uint8_t c = Serial2.read();
// #if 0
//   const uint8_t c0 = c < ' ' ? ' ' : c;
//   printf("*%02x %c ", c, c0);
//   fflush(stdout);
// #endif
//   rcvBuf[rcvFil] = c;
//   rcvFil++;
//   if (rcvFil >= RCV_BUF_LEN ||
//       c == '\n')
//   {
// #if 0
//    fputs("\n", stdout);
//    printHex(rcvBuf, rcvFil);
// #endif
//    tcpServer.write(rcvBuf, rcvFil);
//    rcvFil = 0;
//   }
//  } /* while */

#endif	/* GPS_LIB */

 if (Serial.available() > 0)
 {
  char c = Serial.read();
  if (c == '?')
  {
   putc(c, stdout);
   printf("rtk state %d\n" ,rtk.ser.state);
  }
 }

#endif	/* SERVER */

#if defined(CLIENT)

 wiredRead();

#endif	/* CLIENT */

} /* loop */

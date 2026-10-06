// Device connected to stationary GPS is a TCP server
// Device connected to moveable GPS is a TCP client
// moveable GPS starts and waits for stationary GPS to connect to it
// static GPS -> RTKData -> server -> LAN or Wifi -> client -> RTKData -> movable GPS

//#define WIRED_LAN
//#include "cfg.h"
#include <Arduino.h>

#if defined(WIRED_LAN)
#include "wired.h"
#endif  /* WIRED_LAN */

#if defined(WIFI_LAN)
#include "esp_netif.h"
#include "wired.h"
#include "wifi.h"
#endif  /* WIFI_LAN */

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

//#include "dbgPin.h"
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
 puts("server started connect to reference gps");
#endif
#if defined(CLIENT)
 puts("client started connect to remote gps");
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

 wiredReset();  // reset for w5500 and tft display

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
 Serial.println("begin server");
 wiredInit( SERVER_NAME);
#endif	/* SERVER */

#if defined(CLIENT)
 wiredInit(CLIENT_NAME);
#endif	/* CLIENT */

#endif	/* WIRED_LAN */

#if defined(WIFI_LAN)

#if defined(SERVER)
 wifiInit(SERVER_NAME);
#endif	/* SERVER */

#if defined(CLIENT)
puts("begin client");
 wifiInit(CLIENT_NAME);
#endif	/* CLIENT */

 wifiConnect();

#endif  /* WIFI_LAN */

#if defined(TFT)
 tft.setCursor(0, 0);
 int yPos = 0;
#if defined(WIRED_LAN)
 int xPos = tft.drawString(wifiHostName, 0 ,yPos);
 xPos += tft.drawString(" ", xPos ,yPos);
 tft.drawString(ipAddress, xPos ,0);
#endif  /* WIRED_LAN */
#if defined(WIFI_LAN)
 esp_netif_t *netIf = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
 const char *buf = nullptr;
 if (const esp_err_t err =esp_netif_get_hostname(netIf, &buf);
     err == ESP_OK && buf != nullptr)
 {
  printf("hostname %s\n", buf);
  int xPos = tft.drawString(buf, 0 ,yPos);
  xPos += tft.drawString(" ", xPos ,yPos);
  tft.drawString(ipAddress, xPos ,0);
 }
#endif  /* WIFI_LAN */
#endif	/* TFT */
 
#if defined(WIRED_LAN)
  wiredStart();
#endif	/* WIRED_LAN */

 while (Serial2.available())
  Serial2.read();
}

void loop()
{
 const unsigned int t0 = millis();

#if defined(TFT)
 static unsigned int tmr0;
 if ((t0 - tmr0) > 1000)
 {
  tmr0 = t0;
  char buf[20];
  const float temp = temperatureRead();
#if defined(WIRED_LAN)
  snprintf(buf, sizeof(buf), "%4.1f ", temp);
#endif  /* WIRED_LAN */
#if defined(WIFI_LAN)
  const signed char rssi = wifiRSSI();
  snprintf(buf, sizeof(buf), "%3d %4.1f %4d ", rssi, temp, rtk.rxCount);
#endif  /* WIFI_LAN */
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

#if defined(WIRED_LAN)
 wiredData();
#endif  /* WIRED_LAN */

#if defined(WIFI_LAN)
 if (connected)
 {
  if (t0 - lastSend.timestamp > 5000)
  {
   lastSend.timestamp = t0;
   if (client != nullptr)
   {
    client->write("PING\n");
    // printf("send ping\n");
   }
  }
 }
#endif  /* WIFI_LAN */

 pollSerial();
 processSerial();

 if (Serial.available() > 0)
 {
  const char c = Serial.read();
  if (c == '?')
  {
   putc(c, stdout);
   printf("rtk state %d\n" ,rtk.ser.state);
  }
 }

#endif	/* SERVER */

#if defined(CLIENT)

#if defined(WIRED_LAN)
 wiredRead();
#endif  /* WIRED_LAN */

#if defined(WIFI_LAN)

 pollSerial();
 processSerial();

 if (connected)
 {
  if (t0 - lastSend.timestamp > 5000)
  {
   lastSend.timestamp = t0;
   if (client != nullptr)
   {
    client->write("PING\n");
    // printf("send ping\n");
   }
  }
 }
 else
 {
  if ((millis() - connectTmr) > 5000)
  {
   connectTmr = t0;
   connectToServer();
   printf("try to reconnect\n");
  }
 }
#endif  /* WIFI_LAN */

#endif	/* CLIENT */

} /* loop */

#include <SPI.h>
#define _ETHERNET_WEBSERVER_LOGLEVEL_ 4 // NOLINT(*-reserved-identifier)
#include "Ethernet_GenericX.h"
#include <utility/w5100.h>
#include "FreeRTOS.h"

constexpr int uxTopUsedPriority = configMAX_PRIORITIES - 1;

#if defined(CLIENT)
constexpr char CLIENT_NAME[] = "Client1";
#endif	/* CLIENT */

constexpr char SERVER_NAME[] = "Server1";

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

#define RTK_SEND

#define PORT     8088

#if defined(SERVER)
EthernetServer tcpServer(PORT);
static void processData(EthernetClient c);
#endif	/* SERVER */

#if defined(CLIENT)
EthernetClient client;
bool connected;
void releaseStuckSockets();
#endif	/* CLIENT */

#define RX_PIN 39		/* serial 2 rx pin */
#define TX_PIN 40		/* serial 2 tx pin */

#define W5500_RST_PORT 15

#define GPS_LIB

#if defined(GPS_LIB)

#include "../include/cfg.h"
#include "../include/dbgPin.h"
#include "gpsLib.h"

#else

#define DBG0_PIN 4
#define DBG1_PIN 5

inline void dbg0Set()
{
 REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DBG0_PIN));
}

inline void dbg0Clr()
{
 REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DBG0_PIN));
}

inline void dbg1Set()
{
 REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DBG1_PIN));
}

inline void dbg1Clr()
{
 REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DBG1_PIN));
}

#define DBG_PRT
#if defined(DBG_PRT)
int prt;
uint32_t crcBuf[1024];
#endif

void printHex(const uint8_t *data, size_t len)
{
 int col = 0;
 for (size_t i = 0; i < len; i++)
 {
  if (col == 0)
  {
   printf("  %04X: ", static_cast<unsigned int>(i));
  }
  printf("%02X ", data[i]);
  col += 1;
  if (col == 16)
  {
   col = 0;
   printf("\n");
  }
 }
 if (col != 0)
  printf("\n");
}

char* nextArg(char* p0)
{
 while (true)
 {
  const char c0 = *p0;
  if (c0 == 0)
   break;
  p0 += 1;
  if (c0 == ',')
  {
   break;
  }
 }
 return p0;
}

char *getNum(char *p0, int n, int *result)
{
 int val = 0;
 while (n > 0)
 {
  const char c1 = *p0++;
  val *= 10;
  val += c1 - '0';
  n -= 1;
 }
 *result = val;
 return p0;
}

int getNum(char **p0, int n)
{
 char *p1 = *p0;
 int val = 0;
 while (n > 0)
 {
  const char c1 = *p1++;
  val *= 10;
  val += c1 - '0';
  n -= 1;
 }
 *p0 = p1;
 return val;
}

/* ── CRC-24Q constants ───────────────────────────────────────────────────── */
 
#define CRC24Q_POLY      0x1864CFBu  /* Generator polynomial                 */
#define RTCM3_PREAMBLE   0xD3u       /* Mandatory first byte of every frame  */
#define RTCM3_HDR_LEN    3           /* Preamble + 2 length/reserved bytes   */
#define RTCM3_CRC_LEN    3           /* 24-bit CRC appended at end           */
#define RTCM3_MIN_FRAME  (RTCM3_HDR_LEN + RTCM3_CRC_LEN)
 
/* ── CRC-24Q lookup table (generated once on first use) ─────────────────── */
 
static uint32_t crc24qTable[256];
 
static void buildCRC24qTable()
{
 for (uint32_t i = 0; i < 256; i++)
 {
  uint32_t crc = i << 16;
  for (int j = 0; j < 8; j++)
  {
   crc <<= 1;
   if (crc & 0x1000000u)
    crc ^= CRC24Q_POLY;
  }
  crc24qTable[i] = crc & 0xFFFFFFu;
 }
}

inline uint32_t crc24(const uint32_t crc, const unsigned char c)
{
 return ((crc << 8) ^ crc24qTable[((crc >> 16) ^ c) & 0xFFu]) & 0xFFFFFFu;
}

// #if defined(RTK_SEND)
// //const char* serverIP = "192.168.0.7";
// const char* serverIP = "192.168.0.237";
// #endif	/* RTK_SEND */
//
// #if defined(RTK_RECV)
// AsyncServer server(port);
// #endif	/* RTK_RECV */

enum RCV_STATE {RCV_IDLE, RCV_GET_LEN, RCV_GET_DATA, RCV_TEXT};

constexpr size_t RTK_BUF_SIZE = 1024;

typedef struct S_RTK_DATA
{
 RCV_STATE state;
 unsigned int t0;
 uint64_t startTime;
 uint32_t crc;
 int count;
 int len;
 int fil;
 char buf[RTK_BUF_SIZE];
 unsigned int t0Accum;
 int rxAccum;
 int rxCount;
} T_RTK_DATA, *P_RTK_DATA;

T_RTK_DATA rtk;

#endif	/* GPS_LIB */

#if defined(TFT)
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
#endif

#define TFT_GREY 0x5AEB // New colour

void setup()
{
 buildCRC24qTable();
 Serial.begin(115200);

 pinMode(DBG0_PIN, OUTPUT);
 pinMode(DBG1_PIN, OUTPUT);
 
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
 Serial.print('0');
 Serial.flush();

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

 pinMode(W5500_RST_PORT,OUTPUT);
 digitalWrite(W5500_RST_PORT,LOW);
 delay(10);
 digitalWrite(W5500_RST_PORT,HIGH);
 delay(50);

#if 1
 uint64_t baseMac = ESP.getEfuseMac();
 printf("mac %llx\n", baseMac);
 auto p = reinterpret_cast<uint8_t *>(&baseMac);
 for (i = 0; i < 6; i++)
 {
  printf("%02X", *p++);
  if (i < 5)
   printf(":");
 }
 printf("\n");
#endif

#if defined(TFT)
 dbg0Set();
 printf("setup running on Core: %d\n", xPortGetCoreID());

 printf("TFT_DC %d TFT_MOSI %d TFT_SCLK %d TFT_CS %d\n",
	TFT_DC, TFT_MOSI, TFT_SCLK, TFT_CS);

 tft.init();

 tft.fillScreen(TFT_BLACK);
 tft.setCursor(0, 0, 1);
 const int fontHeight = tft.fontHeight();
 printf("font height %d\n", fontHeight), 
 tft.setTextColor(TFT_RED, TFT_BLACK);
 tft.setTextSize(1);
 tft.println("Initializing");
 dbg0Clr();
#endif

 puts("starting on " ETHERNET_GENERIC_VERSION);

#define USE_THIS_SS_PIN   10

 printf("%s %d\n", "ESP32 setCsPin:", USE_THIS_SS_PIN);

 puts("ethernet init start");
 Ethernet.init(USE_THIS_SS_PIN);
 puts("ethernet init done");

#if defined(SERVER)
 Serial.println("begin server");
 Ethernet.setHostname(SERVER_NAME);
#endif	/* SERVER */

#if defined(CLIENT)
 puts("begin client");
 Ethernet.setHostname(CLIENT_NAME);
 #endif	/* CLIENT */

 puts("begin done");

  // Just info to know how to connect correctly
 // To change for other SPI
 puts("Currently Used SPI pinout:");
 printf("%s %d\n", "MOSI:", MOSI);
 printf("%s %d\n", "MISO:", MISO);
 printf("%s %d\n", "SCK:",  SCK);
 printf("%s %d\n", "SS:",   SS);

#if defined(SERVER)
 const char *hostName = SERVER_NAME;
#endif
#if defined(CLIENT)
 const char *hostName = CLIENT_NAME;
#endif

 printf("hostname %s\n", hostName);

 char ipAddress[20];
 strncpy(ipAddress, Ethernet.localIP().toString().c_str(), sizeof(ipAddress));

 printf("Connected! IP address %s\n", ipAddress);
 printf("%s\n", ipAddress);

#if defined(TFT)
 tft.setCursor(0, 0);
 int yPos = 0;
 int xPos = tft.drawString(hostName, 0 ,yPos);
 xPos += tft.drawString(" ", xPos ,yPos);
 tft.drawString(ipAddress, xPos ,0);
#endif	/* TFT */
 
 EthernetChip_t chip = Ethernet.getChip();
 if ( (chip == w5500) || (chip == w6100) || (Ethernet.getAltChip() == w5100s) )
 {
  if (chip == w6100)
   fputs("W6100 => ", stdout);
  else if (chip == w5500)
   fputs("W5500 => ", stdout);
  else if (Ethernet.getAltChip() == w5100s)
   fputs("W5100S => ", stdout);

  printf("Speed %s Duplex %s Link status %d\n",
         Ethernet.speedReport(), Ethernet.duplexReport(),
         Ethernet.linkStatus());
 }

 // give the Ethernet shield a second to initialize:
 delay(1000);
#if defined(SERVER)
 tcpServer.begin();

 IPAddress ip = Ethernet.localIP();
 printf("server address %s %d\n", ip.toString().c_str(), PORT);
#endif	/* SERVER */

#if defined(CLIENT)
 puts("connecting...");

 if (client.connect(SERVER_NAME, PORT))
 {
  puts("connected");
  connected = true;
 }
 else
 {
  puts("connection failed");
 }
#endif /* CLIENT */

 while (Serial2.available())
  Serial2.read();
}

#if defined(SERVER)
constexpr size_t RCV_BUF_LEN = 128;
uint8_t rcvBuf[RCV_BUF_LEN];
int rcvFil = 0;
#endif  /* SERVER */

#if defined(CLIENT)
void checkSerial();
void checkLan(EthernetClient c);
uint32_t conTmr;
uint8_t serialOutBuf[1800];
#endif

void loop()
{
#if defined(SERVER)

 if (rtk.state != RCV_IDLE)
 {
  if ((millis() - rtk.t0) > 100)
  {
   rtk.state = RCV_IDLE;
   puts("receive timeout");
  }
 }

 EthernetClient client = tcpServer.available();
 if (client)
 {
  processData(client);
 }

 while (Serial2.available())
 {
  uint8_t c = Serial2.read();
#if 1
  const uint8_t c0 = c < ' ' ? ' ' : c;
  printf("*%02x %c ", c, c0);
  fflush(stdout);
#endif
  rcvBuf[rcvFil] = c;
  rcvFil++;
  if (rcvFil >= RCV_BUF_LEN ||
      c == '\n')
  {
   fputs("\n", stdout);
   printHex(rcvBuf, rcvFil);
   tcpServer.write(rcvBuf, rcvFil);
   rcvFil = 0;
  }
 }

 if (Serial.available() > 0)
 {
  char c = Serial.read();
  if (c == '?')
  {
   putc(c, stdout);
   printf("rtk state %d\n" ,rtk.state);
  }
 }

#endif	/* SERVER */

#if defined(CLIENT)

 if (connected)
 {
  if (client.connected())
  {
   checkSerial();

   if (client.available())
   {
    const ssize_t len = client.read(serialOutBuf, sizeof(serialOutBuf));
    Serial2.write(serialOutBuf, len);

    printf("Received %u byte(s)\n", len);
   }
  }
  else
  {
   puts("disconnecting.");
   client.stop();
   connected = false;
   conTmr = millis();
  }
 }
 else
 {
  while (Serial.available() > 0)
   Serial.read();

  uint32_t t0 = millis();
  if ((t0 - conTmr) > 2000)
  {
#if defined(RESTART)
   Serial.println("restarting esp32");
   Serial.flush();
   delay(100);
   ESP.restart();
#else
   Serial.println("reset and start again");
   digitalWrite(W5500_RST_PORT,g574LOW);
   delay(10);
   digitalWrite(W5500_RST_PORT,HIGH);
   delay(50);

#if defined(TFT)
   tft.init();

   tft.fillScreen(TFT_BLACK);
   tft.setCursor(0, 0, 2);
   tft.setTextColor(TFT_WHITE,TFT_BLACK);  tft.setTextSize(1);
   tft.println("Hello World!");
#endif	/* TFT */

   Serial.println("call software reset");
   W5100Class::softReset();       // reset the chip
   delay(500);			  // give the chip time to recover

   Ethernet.setHostname(CLIENT_NAME);
   uint16_t index = 1;
   Ethernet.begin(mac[index]);

   Serial.print(F("Connected! IP address: "));
   Serial.println(Ethernet.localIP());

   EthernetChip_t chip = Ethernet.getChip();
   Serial.printf("chip %02x\n", chip);

   Serial.print(F("Speed: "));
   Serial.print(Ethernet.speedReport());
   Serial.print(F(", Duplex: "));
   Serial.print(Ethernet.duplexReport());
   Serial.print(F(", Link status: "));
   Serial.println(Ethernet.linkReport());

   Serial.println("try to connect");
   if (client.connect(server, PORT))
   {
    Serial.println("connected");
    connected = true;
   }
   else
   {
    conTmr = t0;
   }
#endif	/* RESTART */
  }
 }

#endif	/* CLIENT */

} /* loop */

#if defined(SERVER)

static void processData(EthernetClient c)
{
 size_t len = c.read(reinterpret_cast <uint8_t*>(rtk.buf), RTK_BUF_SIZE);
#if 1
 const auto *bytes = reinterpret_cast <const uint8_t *>(rtk.buf);
 printHex(bytes, len);
#endif

#if defined(GPS_LIB)

 processRemData(rtk.buf, len);

#else
 const auto *ptr = rtk.buf;
 while (len > 0)
 {
  len -= 1;
  const char ch = *ptr++;
  switch (rtk.state)
  {
  case RCV_IDLE:
   if (ch == 0xd3)
   {
    rtk.count = 2;
    Serial2.write(ch);
    rtk.buf[0] = ch;
    rtk.crc = crc24qTable[ch];
    crcBuf[0] = rtk.crc;
    rtk.fil = 1;
    rtk.t0 = millis();
    rtk.state = RCV_GET_LEN;
   }
   else //if (ch == '$')
   {
    rtk.t0 = millis();
    rtk.state = RCV_TEXT;
    Serial2.write(ch);
#if 0
    Serial.print(ch);
    Serial.flush();
#endif
   }
   break;
    
  case RCV_GET_LEN:
   Serial2.write(ch);
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ ch) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.len = (rtk.len << 8) + ch;
   rtk.buf[rtk.fil] = ch;
   rtk.fil += 1;
   rtk.count -= 1;
   if (rtk.count == 0)
   {
    rtk.state = RCV_GET_DATA;
    rtk.len &= 0x3ff;
    // printf("dLen %d\n", dLen);
#if defined(DBG_PRT)
    //if (rtk.len == 19)
    if ((prt == 0) && (rtk.len == 19))
    {
     prt = 1;
    }
#endif	/* DBG_PRT */
    rtk.len += 3;
   }
   break;

  case RCV_GET_DATA:
   Serial2.write(ch);
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ ch) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.buf[rtk.fil] = ch;
   rtk.fil += 1;
   rtk.len -= 1;
   if (rtk.len == 0)
   {
#if 0
    const int type = (rtk.buf[3] << 4) | (rtk.buf[4] >> 4);
    printf("len %4d type %4d CRC %08x\n", rtk.fil, type, rtk.crc);
#endif
#if defined(DBG_PRT)
    if (prt == 1)
    {
     printHex(reinterpret_cast<const uint8_t *>(rtk.buf), rtk.fil);
     printHex(reinterpret_cast<const uint8_t *>(crcBuf), rtk.fil << 2);
     prt = 0;
    }
#endif	/* DBG_PRT */
    rtk.state = RCV_IDLE;
   }
   break;

  case RCV_TEXT:
   Serial2.write(ch);
#if 0
   Serial.print(ch);
#endif
   if (ch == '\n')
   {
    Serial.flush();
    rtk.state = RCV_IDLE;
   }
   break;
  }
 }
#endif  /* GPS_LIB */
}

#endif	/* SERVER */

#if defined(CLIENT)
void checkSerial()
{
 if (rtk.state != RCV_IDLE)
 {
  if ((millis() - rtk.t0) > 100)
  {
   rtk.state = RCV_IDLE;
   printf("receive timeout\n");
  }
 }

#if defined(GPS_LIB)
 
 processSerial();
}

#else

 while (Serial2.available() > 0)
 {
  dbg1Set();
  unsigned char c = Serial2.read();
  switch (rtk.state)
  {
  case RCV_IDLE:
   if (c == 0xd3)
   {
    dbg0Set();
    rtk.count = 2;
    rtk.buf[0] = c;
    rtk.crc = crc24qTable[c];
    crcBuf[0] = rtk.crc;
    rtk.fil = 1;
    rtk.t0 = millis();
    rtk.state = RCV_GET_LEN;
    rtk.startTime = esp_timer_get_time();
   }
   else // if (c == '$')
   {
    rtk.t0 = millis();
    rtk.state = RCV_TEXT;
    rtk.buf[0] = c;
    rtk.fil = 1;
    // Serial.write(static_cast<char>(c));
    // Serial.flush();
   }
   break;
    
  case RCV_GET_LEN:
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ c) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.len = (rtk.len << 8) + c;
   rtk.buf[rtk.fil] = c;
   rtk.fil += 1;
   rtk.count -= 1;
   if (rtk.count == 0)
   {
    rtk.state = RCV_GET_DATA;
    rtk.len &= 0x3ff;
    // printf("rtkLen %d\n", rtk.len);
#if 0 && defined(DBG_PRT)
    // if ((prt == 0) && (rtk.len == 19))
    if (rtk.len == 19)
    {
     prt = 1;
    }
#endif	/* DBG_PRT */
    rtk.len += 3;
   }
   break;

  case RCV_GET_DATA:
   rtk.crc = ((rtk.crc << 8) ^ crc24qTable[((rtk.crc >> 16) ^ c) & 0xFFu]) & 0xFFFFFFu;
   crcBuf[rtk.fil] = rtk.crc;
   rtk.buf[rtk.fil] = c;
   rtk.fil += 1;
   rtk.len -= 1;
   if (rtk.len == 0)
   {
    rtk.rxAccum += rtk.fil;
#if 1
    const auto msgT = static_cast<uint32_t>(esp_timer_get_time() - rtk.startTime);
    int type = (rtk.buf[3] << 4) | (rtk.buf[4] >> 4);
    printf("rtkLen %4d type %4d rtkCRC %08x %5d %u\n",
	   rtk.fil, type, rtk.crc, rtk.rxAccum, msgT);
#endif
    rtk.t0Accum = millis();
#if defined(RTK_SEND)
//     sendBinary(reinterpret_cast<const uint8_t *>(rtk.buf), (ssize_t) rtk.fil);
    client.write(rtk.buf, rtk.fil);
#endif	/* RTK_SEND */

#if 0 && defined(DBG_PRT)
     if (prt == 1)
     {
      printHex(reinterpret_cast<const u_int8_t *>(rtk.buf), rtk.fil);
      printHex(reinterpret_cast<const u_int8_t *>(crcBuf), rtk.fil << 2);
      prt = 0;
     }
#endif	/* DDBG_PRT */
    dbg0Clr();
    rtk.state = RCV_IDLE;
   }
   break;

  case RCV_TEXT:
   // Serial.print(static_cast<char>(c));
   rtk.buf[rtk.fil] = c;
   rtk.fil += 1;
   if (c == '\n')
   {
    rtk.buf[rtk.fil] = 0;
    if (connected)
    {
     uint8_t s = client.status();
     size_t sent = client.write(rtk.buf, rtk.fil);
     Serial.printf("status %d sent %d\n", s, sent);
     if (s == 0)
     {
      Serial.println("write fail disconnecting");
      client.stop();
      connected = false;
      conTmr = millis();
     }
    }

    Serial.print(rtk.buf);
    Serial.println();

#if 0
    if (rtk.buf[0] == '$')
    {
     /* $GNGGA, 091628.00, 3844.78718183,N, 07755.96337656,W, 7,28,0.5,135.9670,M,-33.6653,M, ,*44 */
     if (strncmp(rtk.buf, "$GNGGA", 6) == 0)
     {
      char *p = nextArg(rtk.buf);

      // int h, m, s;
      // p = getNum(p, 2 , &h);
      // p = getNum(p, 2 , &m);
      // p = getNum(p, 2 , &s);
      // const int gpsTime = ((h * 60 + m) * 60) + s;

      int gpsTime = getNum(&p, 2) * 60;
      gpsTime += getNum(&p, 2);
      gpsTime *= 60;
      gpsTime += getNum(&p, 2);

      p = nextArg(p);
      int tmp = getNum(&p, 2);
      const double lat = static_cast<double>(tmp) + strtod(p, &p) / 60.0;
      p = nextArg(p);
      p = nextArg(p);
      tmp = getNum(&p, 2);
      const double lon = -(static_cast<double>(tmp) + strtod(p, &p) / 60.0);
      printf("gpsTime %6d lat %13.10f lon %14.10f\n", gpsTime, lat, lon);
     }
    }
#endif

    rtk.fil = 0;
    rtk.state = RCV_IDLE;
   }
   break;
  }
  dbg1Clr();
 }
}

#endif	/* GPS_LIB  */


void checkLan(EthernetClient c)
{
 uint8_t buf[1800];
 ssize_t len = c.read(buf, sizeof(buf));
 printf("Received %u byte(s)\n", len);
 Serial2.write(buf, len);
}

void releaseStuckSockets()
{
 for (uint8_t i = 0; i < MAX_SOCK_NUM; i++)
  {
  uint8_t s = W5100Class::readSnSR(i);
  switch (s)
  {
  case SnSR::CLOSED:
   break;
  case SnSR::CLOSE_WAIT:
  case SnSR::FIN_WAIT:
  case SnSR::CLOSING:
  case SnSR::TIME_WAIT:
  case SnSR::LAST_ACK:
   W5100Class::execCmdSn((SOCKET)i, Sock_CLOSE);
   break;
  default:
   Serial.printf("socket %d state %02x", i, s);
  }
 }
}
#endif	/* CLIENT */

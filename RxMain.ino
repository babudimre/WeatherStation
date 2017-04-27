/* --------------------------------------------------- INCLUDES -------------------------------------------------- */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
//#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include "Gsender.h"
/* ------------------------------------------------ END OF INCLUDES ---------------------------------------------- */
/* -------------------------------------------- ESP DEPENDENT CONSTANTS ------------------------------------------ */
typedef unsigned char  uint8;
typedef struct device_config
{
   uint8         address;                                        /* Device static address, received at frame byte 0*/
   const char*   url;                                            /* Device sub-url address of node                 */
} device_config_t;
#define GPIO0                   0
#define GPIO1                   1
#define GPIO2                   2
#define GPIO3                   3
#define GPIO4                   4
#define GPIO5                   5
#define GPIO6                   6
#define GPIO7                   7
#define GPIO8                   8
#define GPIO9                   9
#define GPIO10                 10
#define GPIO11                 11
#define GPIO12                 12
#define GPIO13                 13
#define GPIO14                 14
#define GPIO15                 15
#define GPIO16                 16
/* --------------------------------------- END OF ESP DEPENDENT CONSTANTS ---------------------------------------- */
/* ------------------------------------------------ CONFIGURATION ------------------------------------------------ */
#define SPI_MISO               GPIO12                            /* GPIO12; SPI master in slave out nRF24L01+ */
#define SPI_MOSI               GPIO13                            /* GPIO13; SPI master out slave in nRF24L01+ */
#define SPI_CLK                GPIO14                            /* GPIO14; SPI serial clock nRF24L01+ */
#define RF24_IRQ               GPIO5                             /* GPIO5;  interrupt pin nRF24L01+ */
#define RF24_CS                GPIO15                            /* GPIO15;  chip select nRF24L01+ */
#define RF24_CE                GPIO4                             /* GPIO4;  chip enable nRF24L01+ */
#define PAYLOAD_SIZE           6                                 /* 1-32 bytes */
#define NUMBER_OF_NODES        4                                 /* Number of transmitter nodes */
#define MAX_SAMPLES_ON_GRAPH   (1400/NUMBER_OF_NODES)            /* Maximal number of sample points shown on graph (2400 depends on implementation) */
#define SAMPLE_RATE            (30)                              /* [Minutes], Transmit Rate */
const device_config_t device_config[NUMBER_OF_NODES] = 
{
   {0x55, "node1"},                                              /* Device1: address = 0x55, url = http://192.168.1.xxx/node1 */
   {0x66, "node2"},                                              /* Device2: address = 0x66, url = http://192.168.1.xxx/node2 */
   {0x77, "node3"},                                              /* Device3: address = 0x77, url = http://192.168.1.xxx/node3 */
   {0x88, "node4"}                                               /* Device4: address = 0x88, url = http://192.168.1.xxx/node4 */
}; 

const char* ssid =             "Guest";                          /* Name of Wifi */
const char* pass =             "18NikolaTesla56";                /* Password of Wifi */
/* -------------------------------------------- END OF CONFIGURATION --------------------------------------------- */
/* ------------------------------------------------- VARIABLES --------------------------------------------------- */
uint8 RxBuffer[PAYLOAD_SIZE];                                    /* Receive buffer for radio memory */
uint8 Address;                                                   /* Transmitter id from radio */
float temperature[NUMBER_OF_NODES][MAX_SAMPLES_ON_GRAPH];        /* Actual temperature for every nodes every sample points */
float max_temperature[NUMBER_OF_NODES];                          /* Maximal temperature for every nodes */
uint32 max_temp_location = 0;                                    /* Number of sample point where the maximum stored */
float min_temperature[NUMBER_OF_NODES];                          /* Minimal temperature for every nodes */
uint32 min_temp_location = 0;                                    /* Number of sample point where the minimum stored */
float sum_temperature[NUMBER_OF_NODES];                          /* Sum of measured temperature values. Used for average calculation */
float avg_temperature[NUMBER_OF_NODES];                          /* Average temperature for every nodes */
uint32 avg_temp_location = 0;                                    /* Number of sample point where the average stored */
float voltage[NUMBER_OF_NODES];                                  /* Actual battery voltage for every nodes every sample points */
uint8 shift_time[MAX_SAMPLES_ON_GRAPH];                          /* See description at ShiftSamplesbyOne function */
float shift_measurement[MAX_SAMPLES_ON_GRAPH];                   /* See description at ShiftSamplesbyOne function */
uint32 NumberOfSample[NUMBER_OF_NODES];                          /* Number of the received sample */
uint32 AvailableSamplePoints[NUMBER_OF_NODES];                   /* Number of the already available samples */
uint8 day[NUMBER_OF_NODES][MAX_SAMPLES_ON_GRAPH];                /* Actual day for every nodes every sample points */
uint8 hour[NUMBER_OF_NODES][MAX_SAMPLES_ON_GRAPH];               /* Actual hour for every nodes every sample points */
uint8 min[NUMBER_OF_NODES][MAX_SAMPLES_ON_GRAPH];                /* Actual minute for every nodes every sample points */
uint8 day_now;                                                   /* Day from NTP Server - Updated in every loop */
uint8 hour_now;                                                  /* Hour from NTP Server - Updated in every loop */
uint8 min_now;                                                   /* Minute from NTP Server - Updated in every loop */
uint32 time_now;                                                 /* Seconds since 1970 - Updated in every loop */
uint32 measurement_time[NUMBER_OF_NODES];                        /* Seconds since 1970 -> updated when new sample received */
uint8 id;                                                        /* Internal identifier of the transmitter node */
IPAddress timeServerIP;                                          /* Time server IP address */
IPAddress localIP;                                               /* Local IP Address. Used in the generate HTML webpages section */
byte packetBuffer[48];                                           /* buffer for NTP */
WiFiUDP udp;                                                     /* UDP for NTP  */
String NodeName[4];                                              /* Device name stored in EEPROM */
String WifiName;                                                 /* Wifi SSED stored in EEPROM */
String WifiPass;                                                 /* Wifi password stored in EEPROM */
uint8 PasswordLength;
String EmailAddress;                                             /* email address stored in EEPROM */
Gsender *gsender =             Gsender::Instance();              /* Prototype for Gmail email message */
ESP8266WebServer               server (80);                      /* Webserver Port Config   */
const char* ntpServerName =    "time.nist.gov";                  /* NTP server domains      */
uint16 localPort =             2390;                             /* Port for NTP UDP server */
uint8 package_sent = 0;
/* --------------------------------------------- END OF VARIABLES ------------------------------------------------ */
/* --------------------------------------------------- MACROS ---------------------------------------------------- */
#define W_REGISTER             0x20                              /* RF24, command for write register */
#define R_RX_PAYLOAD           0x61                              /* RF24, command for read paylad    */
#define W_TX_PAYLOAD           0xA0                              /* RF24, command for write payload  */
#define RTX_CSN_Low()          digitalWrite(RF24_CS, LOW)        /* RF24, controls bit chip select   */
#define RTX_CSN_High()         digitalWrite(RF24_CS, HIGH)       /* RF24, controls bit chip select   */
#define RTX_CE_Low()           digitalWrite(RF24_CE, LOW)        /* RF24, controls bit chip enable   */
#define RTX_CE_High()          digitalWrite(RF24_CE, HIGH)       /* RF24, controls bit chip enable   */
#define RF24_IRQ_state()       (digitalRead(RF24_IRQ) != HIGH)   /* RF24, read interrupt pin state   */
/* adapt ccs c embedded functions to arduino */
#define delay_us(x)            delayMicroseconds(x)              /* Delay x microseconds              */
#define delay_ms(x)            delay(x)                          /* Delay x miliseconds               */
#define bit_test(x, n)         bitRead(x, n)                     /* Check if var x in position n is 1 */
#define bit_clear(x, n)        bitClear(x, n)                    /* Set 0 the var x in position n     */
#define bit_set(x, n)          bitSet(x, n)                      /* Set 1 the var x in position n     */
#define output_high(pin)       digitalWrite(pin, HIGH)           /* Set high pin                      */
#define output_low(pin)        digitalWrite(pin, LOW)            /* Set low pin                       */
#define input(pin)             (digitalRead(pin) == HIGH)        /* Read pin                          */
#define MONDAY                 4                                 /* Macro for NTP server */
#define TUESDAY                5                                 /* Macro for NTP server */
#define WEDNESDAY              6                                 /* Macro for NTP server */
#define THURSDAY               0                                 /* Macro for NTP server */
#define FRIDAY                 1                                 /* Macro for NTP server */
#define SATURDAY               2                                 /* Macro for NTP server */
#define SUNDAY                 3                                 /* Macro for NTP server */
#define PRINT_IP(p)            p+=String(localIP[0]);p+=F(".");p+=String(localIP[1]);p+=F(".");p+=String(localIP[2]);p+=F(".");p+=String(localIP[3]);
#define SAMPLE_PER_DAY         (1440 / SAMPLE_RATE)              /* Sample / day */
/* ----------------------------------------------- END OF MACROS ------------------------------------------------- */
/* ------------------------------------------------- FUNCTIONS --------------------------------------------------- */
void setup()
{
  EEPROM.begin(512);
  ConfigureSerialPort();
  InitVariables();
  ConfigureESP8266();
  ConfigureTimeServer();
  ConfigureWebServer();
  ConfigureRadio();
  ConvertTime();         /* Get time from Time Server */
}

void loop()
{
  Radio_ProcessData();   /* Receive and process data from nodes */
  server.handleClient(); /* Handle webserver */
  //SendMail();

}

#define SEND_TIME 21
void SendMail(void)
{
  if (package_sent != 1)
  {
    uint8 nodeid;
    uint8 i;
    uint8 samples_to_send;
    
    if (hour_now == SEND_TIME)
    {
      String subject = "Napi összesítő";
      String p;
      p+=F("Szia!<br>");
      for (nodeid = 0; nodeid < NUMBER_OF_NODES; nodeid++)
      {
        if(NumberOfSample[nodeid] < SAMPLE_PER_DAY)
          samples_to_send = NumberOfSample[nodeid];
        else
          samples_to_send = SAMPLE_PER_DAY;
        for (i = 0; i < samples_to_send; i++)
        {
          p+=F("Összesen "); p+=String(samples_to_send); p+=F(" darab mérés érkezett a(z) "); p+=String(NodeName[id]); p+=(" állomástól a mai nap folyamán.<br>");
        
          p+=F("Napi átlag: "); p+=String(avg_temperature[nodeid]); p+=F("&deg;C   <br>\
           Napi minimum: "); p+=String(min_temperature[nodeid]); p+=F("&deg;C  <br>\
           Napi maximum: "); p+=String(max_temperature[nodeid]); p+=F("&deg;C    <br>\
           Feszültség: ");
          p+=String(voltage[nodeid]); p+=F("V ("); p+=String((voltage[nodeid] - 1) / 0.02); p+=F(")<br>");
        }
      }
      for (nodeid = 0; nodeid < NUMBER_OF_NODES; nodeid++)
      {
        for (i = 0; i < samples_to_send; i++)
        {
          p+=F("<b>Mérések innen: "); p+=String(NodeName[id]); p+=F("</b><br>[Óra/Perc Nap, Hőmérséklet]<br>");
          p+=F("['");
          if (hour[nodeid][i] < 10)
            p+=F("0");
          p+=String(hour[nodeid][i]); p+=F(":");
          if (min[nodeid][i] < 10)
            p+=F("0");
          p+=String(min[nodeid][i] % 60);
          if (day[nodeid][i] == THURSDAY)
            p+=F(",  Csütörtök");
          else if (day[nodeid][i] == FRIDAY)
            p+=F(",  Péntek");
          else if (day[nodeid][i] == SATURDAY)
            p+=F(",  Szombat");
          else if (day[nodeid][i] == SUNDAY)
            p+=F(",  Vasárnap");
          else if (day[nodeid][i] == MONDAY)
            p+=F(",  Hétfő");
          else if (day[nodeid][i] == TUESDAY)
            p+=F(",  Kedd");
          else if (day[nodeid][i] == WEDNESDAY)
            p+=F(",  Szerda");
          else
            p+=F(" ");
          p+=F("', "); p+=String(temperature[nodeid][i]); p+=F(" ;degC],<br><br>\
          Üdvözlettel:\
          Időjárásállomás");
        }
      }
      StopWebServer();
      CloseWebServer();
      gsender->Subject(subject)->Send("babudimre@gmail.com", p); 
      ConfigureWebServer();
      StartWebServer();     
      package_sent = 1;
      p = "";
    }
    if (hour_now == SEND_TIME + 1)
    {
      package_sent = 0;
    }
  }
}

void InitVariables(void)
{
  int i;
  int j;
  int nodeid = 0;
  char temp;
  
  for (i = 0; i < NUMBER_OF_NODES; i++)
  {
    min_temperature[i] = 55.0;
    max_temperature[i] = -55.0;
    NumberOfSample[i] = 0;
    AvailableSamplePoints[i] = 0;
    sum_temperature[i] = 0;
  }
  for(nodeid = 0; nodeid < NUMBER_OF_NODES; nodeid++)
  {
    for(i = 0; i < 15; i++)
    {
      temp = EEPROM.read(i + (nodeid * 15));
      if(temp != '\0')
        NodeName[nodeid] += char(temp);
    }
    if(NodeName[nodeid] == "")
      NodeName[nodeid] = "Állomás" + String(nodeid + 1);
  }
  for (i = 60; i < 90; i++)
  {
    temp = EEPROM.read(i);
    if(temp != '\0')
      WifiName += char(temp);
  }
  for (i = 90; i < 120; i++)
  {
    temp = EEPROM.read(i);
    if(temp != '\0')
    {
      WifiPass += char(temp);
      PasswordLength++;
    }
  }
  for (i = 120; i < 160; i++)
  {
    temp = EEPROM.read(i);
    if(temp != '\0')
    {
      EmailAddress += char(temp);
    }
  }
  
  package_sent = 0;
}

void pulse_CSN()
{
  RTX_CSN_High();
  delay_us(20);
  RTX_CSN_Low();
}
uint8 bb_xfer(uint8 spi_data)
{
  delay_us(50);
  uint16 rt;
  uint8 result = 0;
  boolean d = 0;

  for (rt = 0; rt < 8; rt++)
  {
    d = bit_test(spi_data, 7 - rt);
    if (d)
      output_high(SPI_MOSI);
    else
      output_low(SPI_MOSI);

    if (input(SPI_MISO))
      bit_set(result, 7 - rt);
    else
      bit_clear(result, 7 - rt);

    delay_us(10);
    output_high(SPI_CLK);
    delay_us(60);
    output_low(SPI_CLK);
    delay_us(10);
  }
  output_low(SPI_MOSI);
  return (result);
}

void Radio_ConfigureRX(void)
{
  uint16 i;

  RTX_CSN_Low();
  RTX_CE_Low();
  bb_xfer(W_REGISTER); /* PRX, CRC enabled */
  bb_xfer(0x39);
  pulse_CSN();
  delay_ms(2);
  /*-----------*/
  bb_xfer(0x21);  /* dissable auto-ack for all channels */
  bb_xfer(0x00);
  pulse_CSN();
  /*-----------*/
  bb_xfer(0x23); /* address width = 5 bytes */
  bb_xfer(0x03);
  pulse_CSN();
  /*-----------*/
  bb_xfer(0x26); /* data rate = 250kbps */
  bb_xfer(0x26);
  pulse_CSN();
  /*-----------*/
  bb_xfer(0x31);  /* 4 byte payload */
  bb_xfer(PAYLOAD_SIZE);
  pulse_CSN();
  /*-----------*/
  bb_xfer(0x25); /* set channel 2 */
  bb_xfer(0x02);
  pulse_CSN();
  /*-----------*/
  bb_xfer(0x30); /* set address E7E7E7E7E7 */
  for (i = 0; i <= 5; i++)
  {
    bb_xfer(0xe7);
  }
  pulse_CSN();
  /*-----------*/
  bb_xfer(W_REGISTER); //PWR_Up=1 */
  bb_xfer(0x3b);
  RTX_CSN_High();
  RTX_CE_High();
}

void Radio_ReadData(uint8* rx_buff)
{
  uint16 i;
  RTX_CSN_Low();
  bb_xfer(R_RX_PAYLOAD); /* Read RX payload */
  for (i = 0; i < PAYLOAD_SIZE; i++)
  {
    rx_buff[i] = bb_xfer(0x00);
  }
  pulse_CSN();

  /*-----------*/
  bb_xfer(0xE2); /* Flush RX FIFO */
  pulse_CSN();
  /*-----------*/
  bb_xfer(0x27); /* reset int */
  bb_xfer(0x40);
  RTX_CSN_High();
}

void Radio_InitPorts(void)
{
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(RF24_IRQ, INPUT);
  pinMode(RF24_CS, OUTPUT);
  pinMode(RF24_CE, OUTPUT);
}

void ConfigureRadio(void)
{
  Radio_InitPorts();
  RTX_CE_Low();
  RTX_CSN_High();
  delay_ms(500);
  Radio_ConfigureRX();
  delay_ms(100);
}
void Radio_ProcessData(void)
{
  /* Payload from radio:
     -------------------------------------------------------------------
     |  byte 5  |  byte 4  |  byte 3  |  byte 2  |  byte 1  |  byte 0  |
     -------------------------------------------------------------------
     | Address  | Temp_MSB | Temp_LSB | Volt_MSB | Volt_LSB |   CRC    |
     -------------------------------------------------------------------
     Address: ID of transmitter node. Configured on device_config[].address array.
     Temperature: Received temp value from transmitter node. [-55 to 125 C]
     Temperature = (((Temp_MSB << 8) | Temp_LSB) / 360) - 55
     Voltage: Battery level of transmitter node. Supplied from 2*AAA battery.
     Voltage = ((Volt_MSB << 8) | Volt_LSB) / 100)
     CRC: Payload checksum
     CRC = SUM(byte0:byte5) & 0xFF
  */
  id = 0;

  if (digitalRead(RF24_IRQ) != HIGH)
  {
    Radio_ReadData(RxBuffer);

    ConvertTime();         /* Get time from Time Server */

    Address = RxBuffer[0];
    /* Identify the transmitter node */
    while ((Address != device_config[id].address) &&
           (id < NUMBER_OF_NODES))
    {
      id++;
    }
    if (id >= NUMBER_OF_NODES)
    {
      Serial.println("No match");
    }
    else
    {
      /*
          The transmitter node sends at least 5 times the package in row, but we should accept only one. The NRF24L01 is able
          to acknowledge the messages from the transmitter node, but I think it's not the proper way, because the transmitter
          stop the transmission if it does not receive the ack. In case of the distance is too long between the transmitter
          and receiver the Tx node goes to bus-off.
          Anyway I should test auto-acknowledge of Tx node because it could decrease the current consumption.
      */
      if (((time_now - measurement_time[id]) > 60) ||  /* The last package is not within 1 minute */
          (NumberOfSample[id] == 0))                   /* OR No sample available yet */

      {
        if (NumberOfSample[id] == 0)
        {
          AvailableSamplePoints[id] = 1;
        }
        else if (NumberOfSample[id] < MAX_SAMPLES_ON_GRAPH)
        {
          AvailableSamplePoints[id] = NumberOfSample[id];
        }
        else
        {
          AvailableSamplePoints[id] = MAX_SAMPLES_ON_GRAPH;
        }

        day[id][NumberOfSample[id]] = day_now;
        hour[id][NumberOfSample[id]] = hour_now;
        min[id][NumberOfSample[id]] = min_now;
        measurement_time[id] = time_now;
        /* offset: 55, factor: 360 */
        temperature[id][NumberOfSample[id]] = ((((RxBuffer[1] << 8) | RxBuffer[2]) / 360.0) - 55.0);
        /* offset: 0, factor: 10 */
        voltage[id] = ((RxBuffer[3] << 8) | RxBuffer[4]) / 100.0;

        if (max_temperature[id] < temperature[id][NumberOfSample[id]])
        {
          max_temperature[id] = temperature[id][NumberOfSample[id]];
          max_temp_location = NumberOfSample[id];
        }
        if (min_temperature[id] > temperature[id][NumberOfSample[id]])
        {
          min_temperature[id] = temperature[id][NumberOfSample[id]];
          min_temp_location = NumberOfSample[id];
        }
        sum_temperature[id] += temperature[id][NumberOfSample[id]];
        avg_temperature[id] = sum_temperature[id] / (NumberOfSample[id] + 1);

        if (NumberOfSample[id] < MAX_SAMPLES_ON_GRAPH - 1)
        {
          NumberOfSample[id]++;
        }
        else
        {
          ShiftSamplesbyOne(id);
        }
      }
    }
  }
}

/* When we reach the maximal samples (configure the SAMPLE_PER_DAY)
   erase the latest sample point and shift the whole measurements by
   one to make a free slot for the new sample.
   The nodeid parameter is the identifier of the sender node. */

void ShiftSamplesbyOne(uint8 nodeid)
{
  uint32 i = 0;

  for (i = 0; i < MAX_SAMPLES_ON_GRAPH; i++)
    shift_time[i] = hour[nodeid][i];
  for (i = 0; i < MAX_SAMPLES_ON_GRAPH; i++)
    hour[nodeid][i] = shift_time[i + 1];

  for (i = 0; i < MAX_SAMPLES_ON_GRAPH; i++)
    shift_time[i] = min[nodeid][i];
  for (i = 0; i < MAX_SAMPLES_ON_GRAPH; i++)
    min[nodeid][i] = shift_time[i + 1];

  for (i = 0; i < MAX_SAMPLES_ON_GRAPH; i++)
    shift_measurement[i] = temperature[nodeid][i];
  for (i = 0; i < MAX_SAMPLES_ON_GRAPH; i++)
    temperature[nodeid][i] = shift_measurement[i + 1];
}

void ConfigureSerialPort(void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();
}

void ConfigureESP8266(void)
{
  int AttemptCounter = 0;
  char ssid_eep[30];
  char pass_eep[30];

  WifiName.toCharArray(ssid_eep, 30);
  WifiPass.toCharArray(pass_eep, 30);

  if ((WifiName != "") && (WifiPass != "")) /* If config exists in eeprom */
  {
    Serial.print("Connecting to ");
    Serial.println(ssid_eep);
    Serial.print("Password:");
    Serial.println(pass_eep);
    WiFi.begin(ssid_eep, pass_eep);
  }
  else  /* Use hardcoded wifi name & password */
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
  }
  while ((WiFi.status() != WL_CONNECTED) &&
         (AttemptCounter < 20))
  {
    delay(500);
    Serial.print(".");
    AttemptCounter++;
  }

  if (AttemptCounter >= 20)
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    AttemptCounter = 0;
    while ((WiFi.status() != WL_CONNECTED) &&
           (AttemptCounter < 8))
    {
      delay(500);
      Serial.print(".");
      AttemptCounter++;
    }
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  localIP=WiFi.localIP();
}

void ConfigureTimeServer(void)
{
  udp.begin(localPort);
  udp.localPort();
}

uint8 node = 0;
void ConfigureWebServer(void)
{
  MDNS.begin ("weatherstation");
  MDNS.addService("http", "tcp", 80);
  /* Index */
  server.on ( "/", Index );
  
  server.on ( "/node1", []() {
    server.send ( 200, "text/html", CreateWebpage(0));
  });
  server.on ( "/node2", []() {
    server.send ( 200, "text/html", CreateWebpage(1));
  });
  server.on ( "/node3", []() {
    server.send ( 200, "text/html", CreateWebpage(2));
  });

  server.on ( "/node4", []() {
    server.send ( 200, "text/html", CreateWebpage(3));
  });
  /* Webpage for configuration */
  server.on ( "/config", Config );

  /* 404 */
  server.onNotFound ( handleNotFound );
  StartWebServer();
}

inline void StopWebServer(void)
{
   server.stop();
}

inline void CloseWebServer(void)
{
  server.close();
}

inline void StartWebServer(void)
{
  server.begin();
}

void ConvertTime(void)
{
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP);
  delay(1000);

  int cb = udp.parsePacket();
  if (cb)
  {
    udp.read(packetBuffer, 48);

    uint32 highWord = word(packetBuffer[40], packetBuffer[41]);
    uint32 lowWord = word(packetBuffer[42], packetBuffer[43]);
    uint32 secsSince1900 = highWord << 16 | lowWord;
    const uint32 seventyYears = 2208988800UL;
    uint32 epoch = secsSince1900 - seventyYears;

    hour_now = ((epoch  % 86400L) / 3600);
    min_now = (epoch  % 3600) / 60;
    day_now = ((epoch / 86400L) % 7); /* 0=T, 1=F, 2=Sa, 3=Su, 4=M, 5=Tu, 6=W */
    time_now = epoch;
    /* UTC -> CET */
    hour_now += 2;
    if (hour_now >= 24)
    {
      hour_now -= 24;
      if (day_now == 6) /* Sze */
        day_now = 0; /* Cs */
      else
        day_now += 1; /* e.g. H -> K */
    }
  }
}

uint32 sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  memset(packetBuffer, 0, 48);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  udp.beginPacket(address, 123);
  udp.write(packetBuffer, 48);
  udp.endPacket();
}


void Index(void)
{
  String p;
  int nodeid = 0;

  p =
    F("<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Transitional//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd'>\
   <html xmlns='http://www.w3.org/1999/xhtml' xml:lang='hu' lang='hu'>\
   <head>\
   <meta http-equiv='content-type' content='text/html; charset=UTF-8' />\
   <style type='text/css'>\
      html, body {\
         width: 100%;\
         height: 100%;\
         margin: 0px;\
      }\
      body {\
         background-image: -webkit-linear-gradient(0deg, rgb(40, 100, 230) 0%, rgb(66, 152, 160) 100%);\
         background-color: transparent;\
      }\
      table {\
          border: 0px solid black;\
          padding: 5px;\
          border-spacing: 45px;\
      }\
      th{\
          border: 2px solid black;\
          padding: 5px;\
      }\
      td{\
          border: 0px solid grey;\
          padding: 5px;\
          background-color: rgb(40, 100, 230);\
      }\
     .button {\
          background-color: rgb(66, 152, 160);\
          border: 1px solid black;\
          color: white;\
          padding: 5px 0px;\
          width: 100%;\
          text-align: center;\
          text-decoration: none;\
          display: inline-block;\
          font-size: 18px;\
          cursor: pointer;\
     }\
     .button1 {width: 120px;font-size: 14px;}\
     a:hover {\
          background-color: rgb(66, 152, 190);\
          box-shadow: 0 10px 10px 0 rgba(0,0,0,0.24),0 10px 40px 0 rgba(0,0,0,0.19);\
     }\
   </style>\
   </head>\
   <body>\
   <center><table style='width:820px'>");

  for (nodeid = 0; nodeid < NUMBER_OF_NODES; nodeid++)
  {
    p+=F("\
     <tr>\
       <td>\
         <font size='4' color='#FFFFFF'>\
         <code>\
         <center><b>"); p+=NodeName[nodeid]; p+=F("</b> (Állomás "); p+=String(nodeid + 1); p+=F(") </center>");
    if (AvailableSamplePoints[nodeid] != 0)
    {
      p+=F("Hőmérséklet: ");
      p+=String(temperature[nodeid][NumberOfSample[nodeid] - 1]); p+=F("&deg;C    <br>\
         Átlag: ");
      p+=String(avg_temperature[nodeid]); p+=F("&deg;C   <br>\
         Minimum: ");
      p+=String(min_temperature[nodeid]); p+=F("&deg;C  \
         &nbsp;&nbsp;&nbsp;\
         Maximum: ");
      p+=String(max_temperature[nodeid]); p+=F("&deg;C    <br>\
         Feszültség: ");
      p+=String(voltage[nodeid]); p+=F("V ("); p+=String((voltage[nodeid] - 1) / 0.02); p+=F(" %)   <br>\
         Az utolsó mérés ");
      if (hour[nodeid][NumberOfSample[nodeid] - 1] < 10)
        p+=F("0");
      p+=String(hour[nodeid][NumberOfSample[nodeid] - 1]); p+=F(":");
      if (min[nodeid][NumberOfSample[nodeid] - 1] < 10)
        p+=F("0");
      p+=String(min[nodeid][NumberOfSample[nodeid] - 1]); p+=F("-kor történt.\
         <a href='http://"); PRINT_IP(p) p+=F("/"); p+=device_config[nodeid].url; p+=F("' class='button'>Megnyitás >></a>");
    }
    else
    {
      p+=F("Úgy néz ki, hogy megszakadt a kapcsolat.. Lehet lemerült az elem? ");
    }
    p+=F("\
         </font>\
       </td>\
     </tr>");
  }
  p+=F("\
   </table></center>\
   </code>\
   <br><br<br>\
   <code>\
   <a href='http://"); PRINT_IP(p) p+=F("/config"); p+=F("' class='button button1' style='float: right;'>Konfiguráció >></a>\
   </code>\
   </body>\
   </html>");
  server.send ( 200, "text/html", p );
}

String CreateWebpage(uint8 nodeid)
{
  String p;
  uint32 i = 0;

  p =
    F("<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Transitional//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd'>\
    <html xmlns='http://www.w3.org/1999/xhtml' xml:lang='hu' lang='hu'>\
       <head>\
          <meta http-equiv='content-type' content='text/html; charset=UTF-8' />\
          <title>"); p+=NodeName[nodeid]; p+=F("</title>\
          <style type='text/css'>\
          html, body {\
             width: 100%;\
             height: 100%;\
             margin: 0px;\
          }\
          body {\
             background-image: -webkit-linear-gradient(0deg, rgb(40, 100, 230) 0%, rgb(66, 152, 160) 100%);\
             background-color: transparent;\
          }\
         .button {\
             background-color: rgb(66, 152, 160);\
             border: 1px solid black;\
             color: white;\
             padding: 5px 0px;\
             width: 300px;\
             text-align: center;\
             text-decoration: none;\
             display: inline-block;\
             font-size: 15px;\
             cursor: pointer;\
          }\
          a:hover {\
             background-color: rgb(66, 152, 190);\
             box-shadow: 0 10px 10px 0 rgba(0,0,0,0.24),0 10px 40px 0 rgba(0,0,0,0.19);\
          }\
          </style>\
          <meta http-equiv='refresh' content='5'/>\
          <script type='text/javascript' src='https://www.gstatic.com/charts/loader.js'></script>\
          <script type='text/javascript'>\
             google.charts.load('current', {'packages':['corechart']});\
             google.charts.setOnLoadCallback(drawChart);\
             function drawChart() {\
                var data = google.visualization.arrayToDataTable([\
                ['Ido', 'Homerseklet',  ],");
  for (i = 0; i < (AvailableSamplePoints[nodeid]); i++)           /* ['HH:MM:SS, Temp], */
  {
    p+=F("['");
    if (hour[nodeid][i] < 10)
      p+=F("0");
    p+=String(hour[nodeid][i]); p+=F(":");
    if (min[nodeid][i] < 10)
      p+=F("0");
    p+=String(min[nodeid][i] % 60);
    if (day[nodeid][i] == THURSDAY)
      p+=F("  Cs");
    else if (day[nodeid][i] == FRIDAY)
      p+=F("  P");
    else if (day[nodeid][i] == SATURDAY)
      p+=F("  Szo");
    else if (day[nodeid][i] == SUNDAY)
      p+=F("  V");
    else if (day[nodeid][i] == MONDAY)
      p+=F("  H");
    else if (day[nodeid][i] == TUESDAY)
      p+=F("  K");
    else if (day[nodeid][i] == WEDNESDAY)
      p+=F("  Sze");
    else
      p+=F(" ");
    p+=F("', "); p+=String(temperature[nodeid][i]); p+=F("],");
  }
  p+=F("]);\
                var options = {\
                   curveType: 'function',\
                   hAxis: {\
                      textStyle: {color: '#FFFFFF', fontSize: 17, fontName: 'Calibri'},\
                      slantedText: true,\
                      slantedTextAngle: 45},\
                   vAxis: {\
                      textStyle: {color: '#FFFFFF', fontSize: 17, fontName: 'Calibri', bold: true}},\
                   series: {\
                      0: {color: '#00FF00', visibleInLegend: false, lineWidth: 2, pointSize: 7}},\
                   chartArea: {\
                      left:50,\
                      top:20,\
                      width:'100%',\
                      height:'80%'},\
                   backgroundColor: { fill:'transparent' }};\
                var chart = new google.visualization.LineChart(document.getElementById('curve_chart'));\
                chart.draw(data, options);\
             }\
          </script>\
       </head>\
       <body>\
          <center><div id='curve_chart' style='width: screen.width; height: 80vh'></center></div>\
          <font color=#FFFFFF>");
  p+=F("<b>"); p+=NodeName[nodeid]; p+=F("<br></b>\
          Pillanatnyi Hőmérséklet: "); p+=String(temperature[nodeid][NumberOfSample[nodeid] - 1]); p+=F(" &deg;C <br>\
          Maximum: "); p+=String(max_temperature[nodeid]); p+=F(" &deg;C&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
          Minimum: "); p+=String(min_temperature[nodeid]); p+=F(" &deg;C&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
          Átlag: "); p+=String(avg_temperature[nodeid]); p+=F(" &deg;C<br>\
          Feszültség: "); p+=String(voltage[nodeid]);  p+=F(" V  ("); p+=String((voltage[nodeid] - 1) / 0.02); p+=F(" %)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
  if (voltage[nodeid] > 2.5)
    p +=
      F("Elemek megfelelőek, csere nem szükséges.<br>");
  else if (voltage[nodeid] > 1.7)
    p +=
      F("Elemek merülőben, csere még nem szükséges.<br>");
  else
    p +=
      F("Alacsony feszültség, kérlek cseréld ki az elemeket!<br>");
  p +=
    F("</font>\
          <a href='http://"); PRINT_IP(p) p+=F("' class='button'><< Vissza a kezdőlapra</a>\
       </body>\
  </html>");

  return p;
}
void Config(void)
{
  String p;
  String data = "";
  int nodeid = 0;
  int i = 0;

  p=F("<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Transitional//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd'>\
   <html xmlns='http://www.w3.org/1999/xhtml' xml:lang='hu' lang='hu'>\
   <head>\
   <meta http-equiv='content-type' content='text/html; charset=UTF-8'/>\
   <style type='text/css'>\
      html, body {\
         width: 100%;\
         height: 100%;\
         margin: 0px;\
      }\
      body {\
         background-image: -webkit-linear-gradient(0deg, rgb(40, 100, 230) 0%, rgb(66, 152, 160) 100%);\
         background-color: transparent;\
      }\
      input[type=submit] {\
          background-color: rgb(66, 152, 160);\
          border: 1px solid black;\
          color: white;\
          padding: 5px 0px;\
          width: 80px;\
          text-align: center;\
          text-decoration: none;\
          display: inline-block;\
          font-size: 14px;\
          cursor: pointer;\
      }\
      input[type=text] {\
          width: 130px;\
          box-sizing: border-box;\
          border: 2px solid #ccc;\
          border-radius: 4px;\
          font-size: 14px;\
          background-color: white;\
          background-position: 10px 10px;\
          background-repeat: no-repeat;\
          padding: 3px 0px 3px 00px;\
          -webkit-transition: width 0.4s ease-in-out;\
          transition: width 0.4s ease-in-out;\
      }\
      input[type=text]:focus {\
          width: 200px;\
      }\
      input[type=submit]:hover {\
          background-color: rgb(66, 152, 190);\
          box-shadow: 0 10px 10px 0 rgba(0,0,0,0.24),0 10px 40px 0 rgba(0,0,0,0.19);\
      }\
      .button {\
          background-color: rgb(66, 152, 160);\
          border: 1px solid black;\
          color: white;\
          padding: 5px 0px;\
          width: 355px;\
          text-align: center;\
          text-decoration: none;\
          display: inline-block;\
          font-size: 15px;\
          cursor: pointer;\
      }\
      a:hover {\
          background-color: rgb(66, 152, 190);\
          box-shadow: 0 10px 10px 0 rgba(0,0,0,0.24),0 10px 40px 0 rgba(0,0,0,0.19);\
      }\
   </style>\
   </head>\
   <body>\
      <center>\
      <br><br><br>\
      <code><b><font color=#FFFFFF size='4px'>");
  for (nodeid = 0; nodeid < NUMBER_OF_NODES; nodeid++)
  {
    p+=F("<form action='/config' method='post'>");
    p+=F("<p>Állomás "); p+=String(nodeid); p+=F(" Neve: <input type='text' name='"); p+=device_config[nodeid].url; p+=F("' placeholder='"); p+=NodeName[nodeid];
    p+=F("' maxlength='15'>&nbsp;&nbsp;<input type='submit' value='Küldés'><br></p>");
    p+=F("</form>");
  }
  p+=F("\
   <br><br>\
   <form action='/config' method='post'>\
   <p>WiFi Neve: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<input type='text' name='WIFI' placeholder='"); p+=WifiName; p+=F("' maxlength='30'>&nbsp;&nbsp;<input type='submit' value='Küldés'><br></p>\
   </form>\
   <form action='/config' method='post'>\
   <p>WiFi Jelszó: &nbsp;&nbsp;&nbsp;<input type='text' name='PASS' placeholder='**********' maxlength='30'>&nbsp;&nbsp;<input type='submit' value='Küldés'><br></p>\
   </form> /*\
   <br><br>\
   <form action='/config' method='post'>\
   <p>E-mail Cím: &nbsp;&nbsp;&nbsp;&nbsp;<input type='text' name='EMAIL' placeholder='"); p+=EmailAddress; p+=F("' maxlength='40'>&nbsp;&nbsp;<input type='submit' value='Küldés'><br></p>\
   </form>*/");
  p+=F("\
         <br><br><br>\
          <a href='http://"); PRINT_IP(p) p+=F("' class='button'><< Vissza a kezdőlapra</a>\
      </font></b></code>\
      </center>\
   </body>\
   </html>");
  
  for (nodeid = 0; nodeid < NUMBER_OF_NODES; nodeid++)
  {
     if (server.hasArg(device_config[nodeid].url))
     {
       NodeName[nodeid] = server.arg(device_config[nodeid].url);
       data = NodeName[nodeid];
       for (i = 0; i < 15; i++)
         EEPROM.write((i + (nodeid * 15)), data[i]);
     }
  }

  if (server.hasArg("WIFI"))
  {
    WifiName = server.arg("WIFI");
    for (i = 0; i < 30; i++)
      EEPROM.write(60 + i, WifiName[i]);
  }
  if (server.hasArg("PASS"))
  {
    WifiPass = server.arg("PASS");
    for (i = 0; i < 30; i++)
      EEPROM.write(90 + i, WifiPass[i]);
  }
 /* if (server.hasArg("EMAIL"))
  {
    EmailAddress = server.arg("EMAIL");
    for (i = 0; i < 40; i++)
      EEPROM.write(120 + i, EmailAddress[i]);
  }*/
  EEPROM.commit();
  
  server.send ( 200, "text/html", p );
}

void handleNotFound(void)
{
  String p;
  p=F("404 - Nincs ilyen oldal!\n\n");
  p+=F("URI: ");
  p+=server.uri();
  p+=F("\nMethod: ");

  p+=( server.method() == HTTP_GET ) ? "GET" : "POST";
  p+=F("\nArguments: ");
  p+=server.args();
  p+=F("\n");

  for ( uint8 i = 0; i < server.args(); i++ )
  {
    p+=" " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text/plain", p );
}
/* ---------------------------------------------- END OF FUNCTIONS ----------------------------------------------- */

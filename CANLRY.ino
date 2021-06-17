/**
   CANLRY-
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager

#include <SPI.h>
#include <mcp_can.h>

/**
   Toggle the production of debug messages on the serial output.
   Set to 1 to enable the messages on the serial port
*/
#define DEBUG        0

#define LORRY "192.168.0.162"   // need to do name lookup
#define PORT  23

#define CONFIG_AP "canlorry8266"
#define HOSTNAME  "canlorry"

#define BASEADDR  400   // DCC address of the lorry
/**
   The following block of #defines configures the pins that are
   used for various special functions:
     CHIPSELECT  is the select pin for the CANBUS interface
     INTPIN      is the "interrupt" pin used by the CANBUS interface
                 to signal that a CANBUS packet is ready to be read
     MODESELECT  is the jumper used to determine the operating mode
*/
#define CHIPSELECT  15 // D8 (SS) 2//D4
#define INTPIN      16 //D0
#define WIFILED     4 //D2 Orange
#define LORRYLED    5 //D1 Green
#define CANLED      0 //D3 Yellow

/**
   The mode select macro tests the jumper in the mode select pin to
   determine the mode of operation. It returns true if the CANDC is
   working on a CBUS that does not also have a CANCMD present.
*/
#define  ISSTANDALONE    false

/**
   The CANID that this module will use when it sends CBUS messages.
   This is actually the same CANID as a CANCMD would use since if this module
   sends any messages it is emulatuing a CANCMD.
*/
#define MYCANID      118

int n_tracks = 1;
void releaseLoco(int session);
void locoRequest(int address, int flags);
void setSpeedSteps(int, int);
void locoSpeed(int session, int reverse, int speed);
void locoSession(int session, int address, int reverse, int speed);
void sendPLOC(int session);
void emergencyStopAll();
void setDCTrack(int session, int reverse, int speed);
void sendError(int addr, int code);
/**
   The CBUS interface object
*/
MCP_CAN CAN0(CHIPSELECT);                        // MCP CAN library

#define MAX_SESSIONS  64                         // Max number of sessions we can deal with

struct {
  int      dcc_address;
  uint8_t  speed;
  uint8_t  flags;
  bool     connected;
} sessions[MAX_SESSIONS];

/**
   Definitions of the flags bits
*/
#define SF_REV   0x01      // Train is running in reverse
#define SF_FREE  0x80      // The session is currently unused

unsigned long pktcnt = 0;
bool canState = false;

WiFiClient client;


void configModeCallback (WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

/**
   Arduino setup routine
   Configures the I/O pins, initialises the CANBUS library
   and sets up the initial session stack
*/
void setup()
{
  int i;

  for (i = 0; i < MAX_SESSIONS; i++)
    sessions[i].flags = SF_FREE;

  pinMode(LORRYLED, OUTPUT);
  pinMode(WIFILED, OUTPUT);
  pinMode(CANLED, OUTPUT);
  digitalWrite(LORRYLED, HIGH);
  digitalWrite(WIFILED, LOW);
  digitalWrite(CANLED, LOW);
  Serial.begin(115200);
  CAN0.begin(CAN_125KBPS, 16);                   // init can bus : baudrate = 500k

  digitalWrite(WIFILED, HIGH);
  pinMode(INTPIN, INPUT);                       // Setting pin for /INT input
  i = 0;
  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(CONFIG_AP)) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(WIFILED, HIGH);
  digitalWrite(LORRYLED, LOW);
  digitalWrite(CANLED, LOW);

  if (!MDNS.begin(HOSTNAME))
  { // Start the mDNS responder for esp8266.local
    Serial.println("Error setting up MDNS responder!");
  }

  for (int i = 0; i < MAX_SESSIONS; i++)
    sessions[MAX_SESSIONS].connected = false;

  if (ISSTANDALONE)
    Serial.print("Standalone ");
  Serial.println("CANLRY starting.");

}

/**
   Arduino loop routine. Essentially look for CBUS messages
   and dispatch them to the appropriate handlers
*/
void loop()
{
  int id;
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  if (!digitalRead(INTPIN))                    // If pin 0 is low, read receive buffer
  {
    pktcnt++;
    if ((pktcnt & 0x0f) == 0)
      canState = !canState;
    digitalWrite(CANLED, canState);
    CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    rxId = CAN0.getCanId();                    // Get message ID
    switch (rxBuf[0])
    {
      case 0x21:                              // Release loco command
        releaseLoco(rxBuf[1]);
        break;

      case 0x47:                              // Session speed and direction
        locoSpeed(rxBuf[1], rxBuf[2] & 0x80, rxBuf[2] & 0x7f);
        break;

      case 0x40:                              // Request loco sesison (RLOC)
        id = rxBuf[2] + ((rxBuf[1] & 0x3f) << 8);
        locoRequest(id, 0);
        break;

      case 0x61:                              // Request loco sesison (GLOC)
        id = rxBuf[2] + ((rxBuf[1] & 0x3f) << 8);
        locoRequest(id, rxBuf[3]);
        break;

      case 0xE1:                              // PLOC
        id = rxBuf[3] + ((rxBuf[2] & 0x3f) << 8);
        locoSession(rxBuf[1], id,  rxBuf[4] & 0x80, rxBuf[4] & 0x7f);
        break;

      case 0x44:                              // Set Speed Step Range
        setSpeedSteps(rxBuf[1], rxBuf[2]);
        break;

      case 0x0A:                              // Emergency stop all
        emergencyStopAll();
        break;

      default:
#if DEBUG
        Serial.print("ID: ");
        Serial.print(rxId, HEX);
        Serial.print("  Data: ");
        for (int i = 0; i < len; i++)               // Print each byte of the data
        {
          if (rxBuf[i] < 0x10)                    // If data byte is less than 0x10, add a leading zero
          {
            Serial.print("0");
          }
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
#endif
        break;
    }
  }
  if (client.connected())
  {
    while (client.available())
      client.read();
  }
}

/*
   A loco release command for the given session
*/
void
releaseLoco(int session)
{
  if (session >= MAX_SESSIONS)
    return;
  sessions[session].flags = SF_FREE;
  if (sessions[session].dcc_address == BASEADDR)
  {
    client.stop();
    digitalWrite(LORRYLED, LOW);
    sessions[session].connected = false;
  }
}

/*
   A speed and direction packet for a particular session.
   Validation the session ID is within range and has not be released
*/
void
locoSpeed(int session, int reverse, int speed)
{

  if (session >= MAX_SESSIONS || (sessions[session].flags & SF_FREE) == SF_FREE)
    return;
  sessions[session].speed = speed;
  if (sessions[session].dcc_address == BASEADDR)
  {
    setDCTrack(session, reverse, speed);
  }
}

/*
   A throttle has requested access to a particular loco address
   This routine is only used if there is no CANCMD on the bus that will
   allocate sessions.
*/
void
locoRequest(int address, int flags)
{
  int i;
  if (ISSTANDALONE)
  {
    for (i = 0; i < MAX_SESSIONS; i++)
    {
      if (sessions[i].dcc_address == address && (sessions[i].flags & SF_FREE) == 0)
      {
        // Loco is already used in a session
        if (flags == 0)
          sendError(address, 2);    // Send a Taken error
        else if (flags == 1)        // Steal
        {
          sendError(address, 8);
          sessions[i].flags = SF_FREE;
          break;
        }
        else if (flags == 2)        // Share
        {
          sendPLOC(i);
        }
        else
          sendError(address, 7);    // Invalid request
        return;
      }
    }

    // If we have got this far then the loco is not in use
    for (i = 0; i < MAX_SESSIONS; i++)
    {
      if ((sessions[i].flags & SF_FREE) == 0)
      {
        Serial.println("Connect to lorry....");
        if (!client.connect(LORRY, PORT))
        {
          Serial.println("Failed to connect to lorry");
          sendError(address, 7);

          digitalWrite(LORRYLED, LOW);
          return;
        }
        Serial.println("Connected");
        digitalWrite(LORRYLED, HIGH);
        locoSession(i, address, 0, 0);
        sendPLOC(i);
        return;
      }
    }
    sendError(address, 1);      // No free sessions
  }
}

/*
   The command station has allocated a session to a locomotive
*/
void
locoSession(int session, int address, int reverse, int speed)
{
  Serial.printf("locoSession: %d, %d, %d, %d\r\n", session, address, reverse, speed);
  if (session >= MAX_SESSIONS)
  {
    Serial.println("Session out of range");
    return;
  }

  sessions[session].dcc_address = address;
  sessions[session].speed = speed;
  sessions[session].flags = reverse ? SF_REV : 0;
  if (address == BASEADDR)
  {
    if (sessions[session].connected == false)

    {
      Serial.println("Connect to lorry....");
      if (!client.connect(LORRY, PORT))
      {
        Serial.println("Failed to connect to lorry");
        sendError(address, 7);

        digitalWrite(LORRYLED, LOW);
        return;
      }
      Serial.println("Connected");
      digitalWrite(LORRYLED, HIGH);
      sessions[session].connected = true;
    }
    else
      Serial.println("Already connected");
    setDCTrack(session, reverse, speed);
  }
}

void
setDCTrack(int session, int reverse, int speed)
{
  int pinidx = (sessions[session].dcc_address - BASEADDR) * 2;

#if DEBUG
  Serial.print("Set ");
  Serial.print(session);
  if (reverse)
    Serial.print(" backwards ");
  else
    Serial.print(" forwards ");
  Serial.println(speed);
#endif
  if (speed == 0)
  {
    // Serial.println("Send command S");
    client.printf("S\r\n");
  }
  else if (!reverse)
  {
    // Serial.printf("Send command B%d\n\r", speed);
    client.printf("B%d\r\n", speed);
  }
  else
  {
    // Serial.printf("Send command F%d\n\r", speed);
    client.printf("F%d\r\n", speed);
  }
}

/**
   Send a PLOC message in response to a CAB requesting a session for
   a DCC address
*/
void
sendPLOC(int session)
{
  unsigned char buf[8];

  buf[0] = 0xE1;
  buf[1] = session;
  buf[2] = (sessions[session].dcc_address >> 8) & 0x3f;
  buf[3] = sessions[session].dcc_address & 0xff;
  buf[4] = sessions[session].speed | ((sessions[session].flags & SF_REV) ? 0x80 : 0);
  buf[5] = 0;  // Zero function bytes
  buf[6] = 0;
  buf[7] = 0;
  CAN0.sendMsgBuf(MYCANID, 0, 8, buf);
}

/**
   Send an error packet in response the a loco request
*/
void
sendError(int addr, int code)
{
  unsigned char buf[8];

  buf[0] = 0x63;
  buf[1] = (addr >> 8) & 0x3f;
  buf[2] = addr & 0xff;
  buf[3] = code;
  CAN0.sendMsgBuf(MYCANID, 0, 4, buf);
}

void
setSpeedSteps(int session, int steps)
{
}

/**
   Stop all DC tracks
   Loop over every session and if it is not free set
   the speed to 0
*/
void
emergencyStopAll()
{
  int i;

  for (i = 0; i < MAX_SESSIONS; i++)
  {
    if ((sessions[i].flags & SF_FREE) == 0)
      locoSpeed(i, 0, 0);
  }
}

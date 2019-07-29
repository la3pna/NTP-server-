/* LM: The following is based on NTP_Server_01.ino, downloaded from:
 *     https://forum.arduino.cc/index.php?topic=197870.0
 *     
 *     Original source is named NTP_Server_01.
 *     This sketch includes modifications as summarized below.
 *     
 *                LM:   Platform Arduion Uno + u-blox NEO-M8N
 *     Modifications:   GPS com via _software_ serial (D5 and D6)
 *                      Substitute GPS date/time crack() for library method
 *                      Fix leap year bug for year > 1970
 *                      Invert order of NTP and GPS polling
 *                      Minor trivial changes
 *                      
 *            LA3PNA:   Added webserver for checking status
 *     Modifications:   IP of connected items listed at status page. 
 *                      OLED display at front panel using U8G2lib.
 *                      
 */

/*
  NTP Time Server:
 
 This code is in the public domain.
 */

//#include <U8g2lib.h>
//#include <U8x8lib.h>
//#include <avr/dtostrf.h>
//#include <Wire.h>

#define vers "NTP GPS V01A (Rev.LM)"
#define W5100_ETHERNET_SHIELD
#define debug true

#include <SPI.h>                  // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>          // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <TinyGPS.h>
//#include <SoftwareSerial.h>       // LM: Adapt for Uno

// Time Server Port
#define NTP_PORT 123

// Time Server MAC address
byte mac[] = {                    // LM: Substitute fake MAC address associated with IP
//0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
  0x02, 0xB6, 0x7E, 0x00, 0x00, 0x02 };

// NTP Server IP Address
IPAddress ip(192, 168, 2, 177);   // LM: DHCP address assigned by Asus router.  Change this
                                  //     as appropriate for the implementation context.
EthernetServer server(80);

static const int NTP_PACKET_SIZE = 48;

// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE]; 
String receivedIP[10];
String localIp;

// An Ethernet UDP instance 
EthernetUDP Udp;

//GPS instance
TinyGPS gps;

int year;
byte month, day, hour, minute, second, hundredths;
unsigned long date, time, age;
uint32_t timestamp, tempval;

////////////////////////////////////////

// LM: GPS message parsing
const char EOL = 10;    // End-of-line
const int MSGLEN = 67;  // GNRMC message length, 66 printable characters + \r 
String tmpMsg = "";
String gnrmcMsg = "";

// LM: Date/time handling
String sUTD = "";       // UT Date
String sUTC = "";       // UT Time
const int CENTURY = 2000;

// LM: Alternative debug
const boolean MYDEBUG = true;

#if (!defined(__SAMD21G18A__) && !defined(__SAMD21G16A__) && !defined(__SAMD21G16B__) && !defined(__SAMD21J18A__) && !defined(__SAMD21E17A__) && !defined(__SAMD21E18A__))
 #error "This program will only work on SAMD series boards like Empyrean and Zero"
#endif

// U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

void setup() {

delay(2000);

//#if true                 // LM: Was #if debug
  SerialUSB.begin(9600);    // LM: Was 115200 
//#endif

//  Serial1.begin(9600); // start GPS (software serial)
  while (!SerialUSB) {
    ; // wait for SerialUSB port to connect. Needed for native USB port only
  }

//#if debug
  SerialUSB.print("Version:");
  SerialUSB.println(vers);
//#endif
 
 /*
  u8g2.begin(); //somehow this is blocking?

 u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char buf[12] ="";
    u8g2.drawStr(0,8,"LA3PNA NTP + GPSDO");
    
  } while ( u8g2.nextPage() );
 */

 //Ethernet.init(10);
  // start Ethernet and UDP:
  
  // Ethernet.begin(mac,ip); // for IP
 //  Ethernet.begin(mac); // for DHCP
 // Udp.begin(NTP_PORT);
 //server.begin();
 
  // Disable everything but $GNRMC
  // Note the following sentences are for UBLOX NEO6MV2 GPS 
  
  // LM: These sentences are the same for the NEO-M8N   -  Comments added
 // Serial1.write("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");   // Lat/Lon data
//  Serial1.write("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");   // Vector track and speed over ground
//  Serial1.write("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");   // Detailed satellite data
//  Serial1.write("$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n");   // Fix information 
//  Serial1.write("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");   // Overall satellite data


    IPAddress Local = Ethernet.localIP();
    String templip = "";
    templip.concat(Local[0]);
    templip = templip + '.';
    templip.concat(Local[1]);
    templip = templip + '.';
    templip.concat(Local[2]);
   templip = templip + '.';
    templip.concat(Local[3]);
    localIp = templip;
    
  //  draw(localIp);

SerialUSB.print("IP address: ");
SerialUSB.println(localIp);
SerialUSB.println(ip);
SerialUSB.println("setup ok");
delay(10000);
}


////////////////////////////////////////

void loop() { // Original loop received data from GPS continuously (i.e. per second)
              // and called processNTP() when valid data were received (i.e. on the second).
              // This revision monitors NTP port continuously and attempts to retrieve
              // GPS data whenever an NTP request is received.
 listenForIncomming();
 processNTP();
}

////////////////////////////////////////

void processNTP() {

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if(packetSize)
  {
    Udp.read(packetBuffer,NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();
    String tempip = "";
    tempip.concat(Remote[0]);
    tempip = tempip + '.';
    tempip.concat(Remote[1]);
    tempip = tempip + '.';
    tempip.concat(Remote[2]);
   tempip = tempip + '.';
    tempip.concat(Remote[3]);
    addIPtolist(tempip);
    if (!getGPSdata())
      return;
#if debug
    SerialUSB.println();
    SerialUSB.print("Received UDP packet size ");
    SerialUSB.println(packetSize);
    SerialUSB.print("From ");

    for (int i =0; i < 4; i++)
    {
      SerialUSB.print(Remote[i], DEC);
      if (i < 3)
      {
        SerialUSB.print(".");
      }
    }
    
    
    SerialUSB.print(", port ");
    SerialUSB.print(PortNum);

    byte LIVNMODE = packetBuffer[0];
    SerialUSB.print("  LI, Vers, Mode :");
    SerialUSB.print(packetBuffer[0],HEX);

    byte STRATUM = packetBuffer[1];
    SerialUSB.print("  Stratum :");
    SerialUSB.print(packetBuffer[1],HEX);

    byte POLLING = packetBuffer[2];
    SerialUSB.print("  Polling :");
    SerialUSB.print(packetBuffer[2],HEX);

    byte PRECISION = packetBuffer[3];
    SerialUSB.print("  Precision :");
    SerialUSB.println(packetBuffer[3],HEX);

    for (int z = 0; z < NTP_PACKET_SIZE; z++) {
      SerialUSB.print(packetBuffer[z],HEX);
      if (((z+1) % 4) == 0) {
        SerialUSB.println();
      }
    }
    SerialUSB.println();
#endif


    packetBuffer[0] = 0b00100100;   // LI, Version, Mode
    packetBuffer[1] = 1 ;   // stratum
    packetBuffer[2] = 6 ;   // polling minimum
    packetBuffer[3] = 0xFA; // precision

    packetBuffer[7] = 0; // root delay
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0; // root dispersion
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    crack(sUTD, sUTC);
    timestamp = numberOfSecondsSince1900Epoch(year,month,day,hour,minute,second);

#if debug
    SerialUSB.println(timestamp);
    print_date(gps);
#endif

    tempval = timestamp;

    packetBuffer[12] = 71; //"G";
    packetBuffer[13] = 80; //"P";
    packetBuffer[14] = 83; //"S";
    packetBuffer[15] = 0; //"0";

    // reference timestamp
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval) & 0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;


    //copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    //receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval) & 0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    //transmitt timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;

    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;


    // Reply to the IP address and port that sent the NTP request

    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer,NTP_PACKET_SIZE);
    Udp.endPacket();
    if (MYDEBUG) displayGPStime();
  }
}




////////////////////////////////////////

// static bool getgps()

// LM: The library crack method always returned an invalid age. I don't know why.
//     Nor did gps.encode(c). So I modified this function to construct a string
//     from the $GNRMC message, and return true on detecting EOL.

bool getgps()
{
  char c;
  while (Serial.available())
  {
    c = Serial.read();
#if debug
    SerialUSB.write(c);// GPS data flowing
#endif
    if (gps.encode(c))
      return true;
    // LM -
    if (c == EOL)
      return true;
    if (c == '$') {
      tmpMsg = c;
    }
    else if (tmpMsg.length() > 0 && tmpMsg.length() < 80) {
      tmpMsg += c;
    }
  }
  return false;
}

////////////////////////////////////////

const uint8_t daysInMonth [] PROGMEM = { 
  31,28,31,30,31,30,31,31,30,31,30,31 }; //const or compiler complains

const unsigned long seventyYears = 2208988800UL; // to convert unix time to epoch

// NTP since 1900/01/01
static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s) {
  int leapAdjustment = 0;
  if (y >= 1970) {
    y -= 1970;
    leapAdjustment = 2;                         // LM: 1970 was NOT a leap year!
  }

  uint16_t days = d;
  
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
/*
  if (m > 2 && y % 4 == 0)
    ++days;
*/
  if (m > 2 && (y + leapAdjustment) % 4 == 0)   // LM: Weak but okay for the present
    ++days;
    
  days += 365 * y + (y + 3) / 4 - 1;
  return days*24L*3600L + h*3600L + mm*60L + s + seventyYears;
}


////////////////////////////////////////

#if debug

////////////////////////////////////////

// LM: Leaving this original code. As noted, gps library methods did not return good values.
static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    SerialUSB.print(F("*******    *******    "));
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d :",
    month, day, year, hour, minute, second);
    SerialUSB.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
}

////////////////////////////////////////

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  SerialUSB.print(sz);
}



#endif

// LM: Poll GPS when NTP request received. Time out if GPS does not return valid data.

boolean getGPSdata() {
  long startTime = millis();
  const long TIMEOUT = 5000;
  boolean validDataReceived = false;
  while (millis() < startTime + TIMEOUT) {
    if (getgps()) {
      gnrmcMsg = tmpMsg;
      tmpMsg = "";
      // $GNRMC message length is 67, including EOL - Ensure full length message
      if (gnrmcMsg.charAt(17) == 'A' && gnrmcMsg.length() == MSGLEN) {
        sUTC = gnrmcMsg.substring(7,13);
        sUTD = gnrmcMsg.substring(53,59);
        crack(sUTD, sUTC);
        validDataReceived = true;
        break;
      }    
    }  
  }
  return validDataReceived;
}

// My message utilities

void crack(String sDate, String sTime) {
  // sDate = ddmmyy
  // sTime = hhmmss
  // hundredths = 00
  year = sDate.substring(4).toInt() + CENTURY;
  month = sDate.substring(2,4).toInt();
  day = sDate.substring(0,2).toInt();
  hour = sTime.substring(0,2).toInt();
  minute = sTime.substring(2,4).toInt();
  second = sTime.substring(4).toInt();
  hundredths = 0;           // LM: GPS time is always acquired on the second (not used)
  age = 0;                  //     Not used in this adaptation
}


// My debug

void displayGPStime() {
    SerialUSB.print(month);
    SerialUSB.print("/");
    SerialUSB.print(day);
    SerialUSB.print("/");
    SerialUSB.print(year);
    SerialUSB.print("  ");
    SerialUSB.print(hour);
    SerialUSB.print(":");
    SerialUSB.print(formatMS(minute));
    SerialUSB.print(":");
    SerialUSB.print(formatMS(second));
    SerialUSB.println("  ");
/*
    timestamp = numberOfSecondsSince1900Epoch(year,month,day,hour,minute,second);
    SerialUSB.print("  timestamp: ");
    SerialUSB.println(timestamp);
*/
}

String formatMS(int ms) {
  // pad minute or second with leading 0
  String result = "";
  if (ms < 10) result = "0";
  return result + String(ms);
}

void listenForIncomming(){
    // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    SerialUSB.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        SerialUSB.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 23");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin  
          // Change from here to get the status page instead of it reporting pin status.
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) { 
            int sensorReading = analogRead(analogChannel);
            client.print("analog input ");
            client.print(analogChannel);
            client.print(" is ");
            client.print(sensorReading);
            client.println("<br />");
          }
          float flat, flon;
          unsigned long age;
            unsigned long chars;
  unsigned short sentences, failed;
          client.print(" is ");
          client.print("LAT=");
          client.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
          client.println("<br />");
          client.print(" LON=");
          client.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
          client.println("<br />");
          client.print(" SAT=");
          client.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
          client.println("<br />");
          client.print(" PREC=");
          client.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
          client.println("<br />");
          gps.stats(&chars, &sentences, &failed);
          client.print(" CHARS=");
          client.print(chars);
          client.println("<br />");
          client.print(" SENTENCES=");
          client.print(sentences);
          client.println("<br />");
          client.print(" CSUM ERR=");
          client.print(failed);
          client.println("<br />");
          client.println("<br />");
          client.println("<br />");
          client.print("Connected IPs:");
          client.println("<br />");
          
 for(int i=0;i<9;i++){
          client.print(receivedIP[i]);
          client.println("<br />");
  }
           client.println("<br />");
          client.println("</html>");
          break;
        }
        
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    SerialUSB.println("client disconnected");
    SerialUSB.println();
  };
}

void addIPtolist(String ip){
      for(int i=8; i>=0; i--){
        receivedIP[i+1] = receivedIP[i];
   //     SerialUSB.println(receivedIP[i]);
        }
      receivedIP[0] = ip;
  }

  void draw(String localip) {
  // graphic commands to redraw the complete screen should be placed here
/*
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char buf[12] ="";
    u8g2.setCursor(70,25);
     u8g2.print(localip);
   // u8g2.drawStr(70,25,dtostrf(temp2, 4, 2, buf));
    //u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,8,"1:");
    u8g2.drawStr(70,8,"2:");
    
  } while ( u8g2.nextPage() );*/
}


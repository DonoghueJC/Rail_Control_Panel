//./\_______________/\________________________________________________________/\_______________/\.
// |********************************************************************************************|.
// |+|+|+|+|+|+|+|+|+|                                                        |+|+|+|+|+|+|+|+|+|. 
// |+|+|+|+|+|+|+|+|+|    Primary systems and touch screen host               |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    for Arduino based rail control system               |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|                                                        |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    Release Version: 1.0                                |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    Sub Version: RC03_B (Beta)                          |+|+|+|+|+|+|+|+|+|. 
// |+|+|+|+|+|+|+|+|+|                                                        |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    Language: C/C++                                     |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    Platform: UNO R3                                    |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    IDE: Arduino 1.8.13                                 |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|                                                        |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|    Code Copyright (C) James C Donoghue                 |+|+|+|+|+|+|+|+|+|.  
// |+|+|+|+|+|+|+|+|+|                                                        |+|+|+|+|+|+|+|+|+|.   
// |********************************************************************************************|.
// |********************************************************************************************|.
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/.
/*
I2C Message Packet (32 Bytes)
*****************************  Byte 1: Init Byte(0xFF)   
**     HEADER (7 BYTES)    ->  Byte 2: Address of sender (0xFE for Master)  
** - - - - - - - - - - - - **  Bytes 3 - 7: Message Header
**     DATA (24 BYTES)     **
** - - - - - - - - - - - - **  
**TERMINATION (1 BYTE 0x11)**     
*****************************
*/
// - - - - - - - - - - - - - - - - - - - - - -

#include <Wire.h>
#include <SdFat.h>
#include <EEPROM.h>
#include <stdio.h>
#include <stdint.h>
#include <pgmspace.h>
#include <SPI.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_Arial.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <U8g2lib.h>
#include "Timer.h"
#include "pitches.h"
#include "TouchScreen.h"

#define FRAME_BUFFER_SIZE (240*320*2)
#define FRAME_BUFFER_SMALL (240*223*2)
#define ENABLE_ILI9341_FRAMEBUFFER
#define ROTATION 0

// - - - - -  I2C Command/Address definitions  - - - -

#define AUX_PROCESSOR 1       //Address of the AUX processor used to control points
#define NUM_POINTS 12
#define I2C_TIMEOUT_MS 500    // n millisecond timeout for ACK messages to be received
#define I2C_POLLING_FREQ 100  // n milliseconds between each I2C data request event 
#define RETRY_LIMIT 5
#define POINTS_CHECK 0x01

// - - - - -  I2C Buffers  - - - -

#define I2C_HEADER_SIZE 6
#define I2C_MESSAGE_SIZE (NUM_POINTS*2 + I2C_HEADER_SIZE + 2)

Timer t; //Timer for I2C data request sceduling
byte timer_ID;

// - - - - - TFT Screen Variables - - - -

#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI    11
#define TFT_SCLK    13
#define TFT_MISO    12

ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

const float fps = 23.98;
const int rate_us = 1e6 / fps;

uint16_t *tft_frame_buffer = nullptr;

// - - - - - Touch Screen Variables - - - -

#define INT8U unsigned char
#define YP A16  // analog pin
#define XM A17  // analog pin
#define YM A0   // digital pin
#define XP A1   // digital pin

//Measured ADC values for (0,0) and (210-1,320-1)

#define TS_MINX 116*2
#define TS_MAXX 890*2
#define TS_MINY 83*2
#define TS_MAXY 913*2

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// - - - - - - OLED Screen object - - - - - -

#define ALL_SCREENS 0x0C
#define NUM_SCREENS NUM_POINTS
#define image_width 128
#define image_height 64

byte selected_screen = 0x00;
bool all_Adr_select = false;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// - - - - - -  SD Card  - - - - - -

SdFs SD;
File file;
File bmp_file;
const char Startup[] = "CompLight_Intro.bin";
const char Loop[] = "CompLight_CLoop.bin";

byte system_error = 0;
/* --------- System Errors ----------
    0x00  No Error
    0x01  RTC/IIC Error   
    0x02  SD Card Error
   ---------------------------------- */



// - - - - - - Points Status - - - - - -
uint8_t point_stat[] = {0,0,0,0,0,0,0,0,0,0,0,0};        // Address 0 - 11 in EEPROM  - These two arrays should match afdter reading to confirm integrity
uint8_t point_stat_check[] = {0,0,0,0,0,0,0,0,0,0,0,0};   // Address 12 - 23 in EEPROM


// - - - - - -  Globals  - - - - - -

int mode = 0; // Current mode of operation
int boot_stage = 0; // Current point in Boot Sequence
int console_pos = 320; // Current position of the console
int bytes_read = 0;
uint32_t m = 0;
uint16_t * p = 0;
int frame_buff_size = FRAME_BUFFER_SIZE;

short refresh_count = 0;
bool toggle = 0;
byte screen = 0x00;
bool screen_update = 1;

// notes in the melody:
int melody[] = {NOTE_C5, NOTE_E5, NOTE_G5, 0, 0, NOTE_C5, NOTE_C5, NOTE_C5};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int note_durations[] = {16, 16, 8, 4, 2, 8, 16, 16};


// - - - - - - - - - - - - - - - - - - - - - -




//./\_______________/\________________________________________________________/\_______________/\.
// |********************************************************************************************|.
// |+|+|+|+|+|+|+|+|+|                  SETUP FUNCTION                        |+|+|+|+|+|+|+|+|+|.
// |********************************************************************************************|.
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/.
void setup()
{

  

  Serial.begin(115200);  
  
  // -=-=-=-=-=-=-=-=-=-=-=-=-=-   CONFIGURE PORTS AND PINS    -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  
  pinMode( 10, OUTPUT );              // TFT CS Pin
  pinMode( 11, INPUT );               // Normally an SPI pin, keep it's impedance High
  pinMode( 12, INPUT );               // Normally an SPI pin, keep it's impedance High
  pinMode( 13, INPUT );               // Normally an SPI pin, keep it's impedance High
  pinMode( 29, OUTPUT );              // Buzzer Pin
  pinMode( 30, OUTPUT );              // Power to Relays
  pinMode( 31, OUTPUT );              // Power to Track
  pinMode( 33, OUTPUT );              // OLED Address Select LSB
  pinMode( 34, OUTPUT );              // OLED Address Select
  pinMode( 35, OUTPUT );              // OLED Address Select
  pinMode( 36, OUTPUT );              // OLED Address Select MSB   (0xOC selects ALL OLED screens)
  pinMode( 37, OUTPUT );              // OLED Multi-Address Select (Must be pushedd High to enable addressing of all OLEDs at once)
  
  // -=-=-=-=-=-=-=-=-=-=-=- INITIALISE SD CARD AND TFT READ FILES -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  
  Serial.println("\nInitializing SD card...");

  if (!SD.begin(SdioConfig(DMA_SDIO))) {
      SD.errorHalt("Card failed, or not present");
      system_error = 0x02;
    }
  else
    Serial.println("card initialized.");
  SPI.begin();
  
  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (system_error){
    
    switch(system_error){
      case 0x01:
        Tft.drawString("IIC ERROR", 0, 10, 3, RED);
        Tft.drawString("Reset Device or check RTC.", 0, 50, 1, WHITE); 
        break;
      case 0x02:    
        Tft.drawString("CARD ERROR", 0, 10, 3, RED);
        Tft.drawString("Re-Insert SD Card Then Press Reset.", 0, 50, 1, WHITE); 
        break;
    }
  }

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=     INITIALISE IIC    -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  
  Wire1.begin();                 // join i2c bus as the master device
  t.every(I2C_POLLING_FREQ, RequestI2CData);
  for(int i = 0; i < I2C_MESSAGE_SIZE; i++)
  {
    I2CSendBuffer[i] = 0x00;
    I2CBuffer[i] = 0x00;
  }
  
  
  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=     INITIALISE TFT    -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  
  tft.begin(26000000);
  tft.setRotation(ROTATION);
  tft.useFrameBuffer(true);
  tft.fillScreen(ILI9341_BLACK);
  tft.updateScreen();
  tft_frame_buffer = tft.getFrameBuffer();
  delay(250);  
  tft.updateScreenAsync(true);

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=     INITIALISE OLED    -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  

  SetAddr(ALL_SCREENS); // ALL_SCREENS addreses all screens at once alowing shorter 'begin' sequence
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawXBMP(0, 0, image_width, image_height, BlankImg);
  u8g2.sendBuffer(); 
  u8g2.setFont(u8g2_font_logisoso20_tf); 
  u8g2.setFontRefHeightExtendedText();
  u8g2.setFontPosTop();
  u8g2.setFontDirection(2);  
    
  for (byte AdrLoop = 0x00; AdrLoop < NUM_SCREENS; AdrLoop++)
  {
    SetAddr(AdrLoop);
    u8g2_Init((int)AdrLoop + 1);
  }  
  

  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   

  mode++;
}



// /\_______________/\________________________________________________________/\_______________/\.
// |********************************************************************************************|.
// |+|+|+|+|+|+|+|+|+|                     MAIN LOOP                          |+|+|+|+|+|+|+|+|+|.
// |********************************************************************************************|.
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/.

void loop()
{
  t.update(); //Timer that deals with I2C Data Requests
  
  if (I2CSendFlag)  //I2C ACK Timeout Check
  {
    if(((millis() - I2CTimeout) > I2C_TIMEOUT_MS || (millis() - I2CTimeout) < 0) && retries > 0) //no Ack has been received within I2C_TIMEOUT_MS, try send again
    {
      IICSendMessage(I2CSendAddress);
      retries--;
    }    
    else if(retries == 0)
    {
      I2CSendFlag = false;
      timeoutError = true;
    }
  }

  if (I2CFlag == 0x02)   // A data packet is ready to be parsed
  { 
    // - - - - - - - - - - Point Check Complete - - - - - - - - - - - -
    if((String)I2CHeader == "CHKOK")
    {
      Serial.println("PointCheck Completed");   
      PointCheck = false;
    }

    // - - - - - - - - - -  Point Set Request  - - - - - - - - - - - -
    // Data byte 0: Address of point to be changed
    // Data byte 1: New direction of point
    if((String)I2CHeader == "POSET") 
    {
      Serial.println("Actioning Point Change");   
      point_stat[I2CData[0]] = I2CData[1];
      EEPROM.write(I2CData[0], I2CData[1]);
      EEPROM.write(I2CData[0]+NUM_POINTS, I2CData[1]);

      //Update screen
      SetAddr(I2CData[0]);
      u8g2.clearBuffer();
      u8g2.setDrawColor(1);
      if(point_stat[I2CData[0]] == 0x00)
      {
        u8g2_bitmapStr(I2CData[0]);
      }
      else
      {
        u8g2_bitmapTrn(I2CData[0]);
      }
      u8g2.sendBuffer();

      
    }

    // - - - - - - - - - - No Command Recognised - - - - - - - - - - - -
    else
    {
      Serial.print("Command not recognised: ");
      Serial.println((String)I2CHeader);
    }
    Serial.println("I2C Message parsed");
    I2CFlag = 0;
  }
  
  switch (mode)
  {
    case 1:  //Boot Sequence
      BootSequence();
      break;
    case 0:
    default:
      break;
  }
}


// -=-=-=-=-=-=-=-=-=-= Prep OLED Screen -=-=-=-=-=-=-=-=-=-=

void u8g2_Init(int PointNum) {  
  
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawXBMP(0, 0, image_width, image_height, BlankImg);
  u8g2.setDrawColor(0);
  
  char buffer [2];
  itoa( PointNum, buffer, 10);
  char* str = concat("POINTS ", buffer);
  if(PointNum < 10)
  {
    u8g2.drawStr(115,43,str);
  }
  else
  {
    u8g2.drawStr(120,43,str);
  } 
  u8g2.sendBuffer(); 
  free(str);
}



// -=-=-=-=-=-=-=-=-=-=    String Concat    -=-=-=-=-=-=-=-=-=-

char* concat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1) + strlen(s2) + 1); // +1 for the null-terminator
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}



// -=-=-=-=-=-=-=-=-=-= Multiplexer Address Setter -=-=-=-=-=-=-=-=-=-

void SetAddr(byte Addr) //Set the 4 bit address value that is shared between multiplexers to select buttons, screens and relays
{
    if(Addr == ALL_SCREENS)
    {
      digitalWrite(37, HIGH);
      all_Adr_select = true;
    }
    else if (all_Adr_select)
    {
      digitalWrite(37, LOW);
      all_Adr_select = false;
    }
    digitalWrite(33, Addr & 0x01);
    digitalWrite(34, (Addr & 0x02) >> 1);
    digitalWrite(35, (Addr & 0x04) >> 2);
    digitalWrite(36, (Addr & 0x08) >> 3);
}



// -=-=-=-=-=-=-=-=-=-=    Boot Sequence    -=-=-=-=-=-=-=-=-=-

void BootSequence()
{

//  - - - - Pre ScreenDraw Code - - - - -

  switch (boot_stage)
  {
    case 0:                                   // Start Splash Screen
      frame_buff_size = FRAME_BUFFER_SIZE;
      if (!file.open(Startup, O_READ)) {
        SD.errorHalt("opening failed");
        return;
      }
      boot_stage++;
      break;

      
    case 3:                                   // Load Console Display
      tft.writeRect(0, 223, 240, 97, (uint16_t*)ConsoleBody);
      tft.writeRect(20, 245, 26, 25, (uint16_t*)EEPROM_G);
      tft.writeRect(45, 245, 26, 25, (uint16_t*)SDCard_G);
      tft.writeRect(70, 245, 26, 25, (uint16_t*)Speaker_G);
      tft.writeRect(95, 245, 26, 25, (uint16_t*)Comms_G);
      tft.writeRect(120, 245, 26, 25, (uint16_t*)Relays_G);
      tft.writeRect(145, 245, 26, 25, (uint16_t*)TrainIDs_G);
      tft.writeRect(170, 245, 26, 25, (uint16_t*)Points_G);
      tft.writeRect(195, 245, 26, 25, (uint16_t*)TrackPower_G);
      boot_stage++;
      break;

      
    case 4:                                   // Load Loop Screen
    case 7:
    case 10:
    case 13:
    case 16:
    case 19:
    case 22:
    case 25:
    case 28:
      frame_buff_size = FRAME_BUFFER_SMALL;
      if (!file.open(Loop, O_READ)) {
        SD.errorHalt("opening failed");
        return;
      }      
      boot_stage++;
      break;


    default:
      break;
  }

  // - - - - - - - -  Screendraw on Frame Sync - - - - - - - - - - - 

  if ((tft.asyncUpdateActive() || (micros() - m >= rate_us)) && (boot_stage == 1 || boot_stage == 5 || boot_stage == 8 || boot_stage == 11 || boot_stage == 14 || boot_stage == 17 || boot_stage == 20 || boot_stage == 23 || boot_stage == 26 || boot_stage == 29)) {    //Draw a frame to screen when ready

    m = micros();                                       //reset m to current time just before each frame is drawn
    p = tft_frame_buffer;
    int rd = 0;
    rd = frame_buff_size;
    do {
      bytes_read = file.read(p, frame_buff_size);
      if (bytes_read <= 0) {
        break;
      }
      p += bytes_read / sizeof(uint16_t);
      rd -= bytes_read;
    } while (bytes_read > 0 && rd > 0);
    
    if (bytes_read <= 0) {
      boot_stage++;    
    }      
  }

  // - - - - - - - - - Post Screendraw Code - - - - - - - - - - -
  
  switch (boot_stage)
  {
    

    case 30:                                            // Generic close and loop around
    case 27:
    case 24:
    case 21:
    case 18:
    case 15:
    case 12:
    case 9:
    case 6:
      file.close();
      boot_stage -= 2;
      break;
    case 26:                                            // Wait for point check to complete, then switch on Track Power
      if(!PointCheck)
        {
          tft.writeRect(170, 245, 26, 25, (uint16_t*)Points_B);
          digitalWrite(31, HIGH); //Switch on power to Track
          Serial.println("Track Powered ON");
          tft.writeRect(195, 245, 26, 25, (uint16_t*)TrackPower_B);
          tft.writeRect(145, 245, 26, 25, (uint16_t*)TrainIDs_B);
          boot_stage += 3;
        }
      break;

    case 23:                                            // Update screens with point direction data
  
      SetAddr(selected_screen);
      delay(10);
      u8g2.clearBuffer();
      u8g2.setDrawColor(1);
      if(point_stat[selected_screen] == 0x00)
      {
        u8g2_bitmapStr(selected_screen);
      }
      else
      {
        u8g2_bitmapTrn(selected_screen);
      }
      u8g2.sendBuffer();
      delay(10);
      selected_screen++;
      if(selected_screen == NUM_POINTS)
      {
        boot_stage += 3;
      }
      break;
    
    case 20:                                            // Wait for confirmation from AUX Processor
      if(!I2CSendFlag)
      {
        if(!timeoutError)
        {
          tft.writeRect(120, 245, 26, 25, (uint16_t*)Relays_B);
          tft.writeRect(170, 245, 26, 25, (uint16_t*)Points_A);
          tft.writeRect(145, 245, 26, 25, (uint16_t*)TrainIDs_A);
        }
        else
        {
          Serial.println("Timeout Detected, no ACK received");
        }
        timeoutError = false;
        selected_screen = 0x00;                          //Prepare to update screens by selecting first screen
        boot_stage += 3;
      }      
      break; 

    case 17:                                            // Activate Relays & send points status to Aux
      tft.writeRect(120, 245, 26, 25, (uint16_t*)Relays_A);
      if(!I2CSendFlag)
      {
        digitalWrite(30, HIGH);                         //Switch on power to Points Relays     
        Serial.println("Points Relays Powered ON");
        IICCommand(POINTS_CHECK);
        PointCheck = true;
        boot_stage += 3;
      }      
      break;     

    case 14:                                            // Comms (I2C) Check
      tft.writeRect(95, 245, 26, 25, (uint16_t*)Comms_A);
      
      Wire1.requestFrom(1, 1);                          // request 1 byte from slave device #1   
      while (Wire1.available()) {   
        byte c = Wire1.read();      
        if (c = 0xFF) 
        {
          tft.writeRect(95, 245, 26, 25, (uint16_t*)Comms_B);        
        }
      }     
      boot_stage += 3;
      break;

    case 11:                                           // Sound Check
      tft.writeRect(70, 245, 26, 25, (uint16_t*)Speaker_A);
      TonesPlay(1);
      tft.writeRect(70, 245, 26, 25, (uint16_t*)Speaker_B);
      boot_stage += 3;
    break;

    case 8:                                             // SD Check
      tft.writeRect(45, 245, 26, 25, (uint16_t*)SDCard_A);
      // We already know SD card is working, reserve this area incase we want to check loading of a specific file/data *from* SD card at boot
      tft.writeRect(45, 245, 26, 25, (uint16_t*)SDCard_B);
      boot_stage += 3;
    break;
      
    case 2:                                             // End of Splash Screen
      file.close();
      boot_stage++;
      break;

    case 5:                                             //Initiate EEPROM
      tft.writeRect(20, 245, 26, 25, (uint16_t*)EEPROM_A);
      int ErrorFlag = 0;
      for(int i = 0; i < 12; i++)                       // Read EEPROM
      {
        point_stat[i] = EEPROM.read(i);
        point_stat_check[i] = EEPROM.read(i+12);
      }
      for(int i = 0; i < 12; i++)                       // Check EEPROM Integrity
      {
        if(point_stat[i] != point_stat_check[i])
          ErrorFlag = 1;
      }
      if(ErrorFlag)
      {
        for(int i = 0; i < 12; i++)                     // Reset EEPROM
        {
          EEPROM.write(i, 0x00);
          EEPROM.write(i+NUM_POINTS, 0x00);
          point_stat[i] = 0x00;
          point_stat_check[i] = 0x00;
          boot_stage += 3;
        }        
      }
      else
      {
        tft.writeRect(20, 245, 26, 25, (uint16_t*)EEPROM_B);        
        boot_stage += 3;
      }      
      break;
      
    default:
    break;
  }
  

}


// -=-=-=-=-=-=-=-=-=-=  IIC Request from slaves (master) -=-=-=-=-=-=-=-=-=-

void RequestI2CData()
{
  byte I2CBuffer[I2C_MESSAGE_SIZE];
  byte I2CSender = 0x00;
  char I2CHeader[I2C_HEADER_SIZE];
  char I2CData[I2C_MESSAGE_SIZE-I2C_HEADER_SIZE-1];
  byte I2CSendBuffer[I2C_MESSAGE_SIZE];
  char I2CSendHeader[I2C_HEADER_SIZE];
  char I2CSendData[I2C_MESSAGE_SIZE-I2C_HEADER_SIZE-1];
  bool I2CSendFlag = false;                     // bool to track if a message has been sent but no Ack recieved
  long I2CTimeout = 0;
  byte I2CSendAddress = 0x00;
  byte I2CFlag = 0x00;                          // 0x00: no message   0x01: Message Processing    0x02: Message Fully Recieved
  byte I2CIndex = 0x00;                         // Positional Index for recieving I2C Data
  short retries = 0;
  bool timeoutError = false;
  bool PointCheck = false;                      // a tracker to track if a point check is in process on the AUX Processor

  Wire1.requestFrom(1, I2C_MESSAGE_SIZE);       // request 1 byte from slave device #1  
  while(Wire1.available())                      // loop through all
  {
    byte c = Wire1.read();                      // receive byte as a bytes
    if (I2CFlag == 0x00 && c == 0xFF)
    {
      I2CFlag = 0x01;                           //Message start byte has been detected, set flag and start collecting data
      I2CIndex = 0x00;
    }
    if (I2CFlag == 0x01)
    {
      I2CBuffer[I2CIndex] = c;
      I2CIndex++;
      if (I2CIndex >= (I2C_MESSAGE_SIZE) && I2CBuffer[I2C_MESSAGE_SIZE - 1] == 0x11) // Full Message Recieved
      {
        Serial.println("Data Packet Received.");

        
        Serial.print("Buffer: ");
        for(int i = 0; i < 32; i++){
          Serial.print(I2CBuffer[i]);
          Serial.print(" ");
        }
        Serial.println();

        I2CSender = I2CBuffer[1];
        for(int i = 0; i < (I2C_HEADER_SIZE - 1); i++)
        {
          I2CHeader[i] = I2CBuffer[i+2];

          Serial.print(I2CHeader[i]);
          Serial.print(" ");
        }
        Serial.println();
        I2CHeader[I2C_HEADER_SIZE - 1] = '\0';
        for(int i = 0; i < (I2C_MESSAGE_SIZE - I2C_HEADER_SIZE - 2); i++)
        {
          I2CData[i] = I2CBuffer[i+I2C_HEADER_SIZE + 1];
        }
        // Check if message is an ACK (Header is 0x00)
        if(I2CHeader[0] == 0x00)
        {
          Serial.println("ACK Received");
          I2CSendFlag = false;
          for(int i = 0; i < I2C_MESSAGE_SIZE; i++)
          {
            I2CBuffer[i] = 0x00;
          }
          I2CFlag = 0x00;
        }
        else // not an ACK, so register the I2C flag to show that message is ready to be parsed and send Ack
        {
          IICSendAck(I2CSender);
          I2CFlag = 0x02;
        }
      }
      else if(I2CIndex >= (I2C_MESSAGE_SIZE) && I2CBuffer[I2C_MESSAGE_SIZE - 1] != 0x11) // Buffer full but corrupt
      {
        Serial.println("Corrupt Data Packet Received !!!!!!!!!!!!!!!!!");
        for(int i = 0; i < I2C_MESSAGE_SIZE; i++)
        {
          I2CBuffer[i] = 0x00;
        }
        I2CFlag = 0x00;
      }
    }
  }
}


// -=-=-=-=-=-=-=-=-=-=  IIC Command (master) -=-=-=-=-=-=-=-=-=-

void IICCommand(byte CommandType)
{
  switch(CommandType)
  {
    case 0x01:                                  //Initiate Points and check positions
      Serial.println("Sending POCHK Command");
      strcpy(I2CSendHeader, "POCHK");           // Define Command String
      Serial.print("Buffer: ");
      for(int i = 0; i < NUM_POINTS; i++)       // Fill out Points data (twice for check)
      {
        I2CSendData[i] = point_stat[i];
        I2CSendData[i + NUM_POINTS] = point_stat[i];
        Serial.print((byte)I2CSendData[i]);
        Serial.print(" ");
      }
      Serial.println();
      IICSendMessage(AUX_PROCESSOR);
      break;

    default:
      break;
  }
}

// -=-=-=-=-=-=-=-=-=-=  IIC Send Mesage (master) -=-=-=-=-=-=-=-=-=-

void IICSendMessage(byte Address)
{
  if(!I2CSendFlag)                              // check if there is a message still waiting to be sent and, if so, try again
  {
    Serial.println("Building Command Packet");
    // Copy data to buffer and build packet
    I2CSendBuffer[0] = 0xFF;
    I2CSendBuffer[1] = 0xFE;                    //Address of this master
    for(int i = 0; i < (I2C_HEADER_SIZE - 1); i++)
    {
      I2CSendBuffer[i + 2] = I2CSendHeader[i];
    }
    for(int i = 0; i < (I2C_MESSAGE_SIZE - I2C_HEADER_SIZE - 2); i++)
    {
      I2CSendBuffer[i + I2C_HEADER_SIZE + 1] = I2CSendData[i];
    }
    I2CSendBuffer[I2C_MESSAGE_SIZE - 1] = 0x11;
    retries = RETRY_LIMIT;
  }

  // Send packet to address provided
  Serial.println("Sending Command Packet");
  I2CSendFlag = true;
  I2CSendAddress = Address;
  I2CTimeout = millis();


  
  Serial.print("Buffer: ");
  for(int i = 0; i < 32; i++){
    Serial.print(I2CSendBuffer[i]);
    Serial.print(" ");
  }
  Serial.println();

  
  
  Wire1.beginTransmission(Address);
  Wire1.write(I2CSendBuffer, I2C_MESSAGE_SIZE);
  Wire1.endTransmission();
  
}

// -=-=-=-=-=-=-=-=-=-=  IIC Send Ack (slave) -=-=-=-=-=-=-=-=-=-

void IICSendAck(byte Address)                 //Does not set timeout or expect a return Ack
{
  // Build ACK packet
  I2CSendBuffer[0] = 0xFF;
  for(int i = 0; i < (I2C_MESSAGE_SIZE - 2); i++)
  {
    I2CSendBuffer[i + 1] = 0x00;
  }
  I2CSendBuffer[I2C_MESSAGE_SIZE - 1] = 0x11;  

  // Send packet to master

  Wire1.beginTransmission(Address);
  Wire1.write(I2CSendBuffer, I2C_MESSAGE_SIZE);  
  Wire1.endTransmission();
}

// -=-=-=-=-=-=-=-=-=-= Screen Redraw Function -=-=-=-=-=-=-=-=-=-

void RefreshScreen(void)
{
  
  return;
}

// -=-=-=-=-=-=-=-=-=-= Speaker Tones Play -=-=-=-=-=-=-=-=-=-

void TonesPlay(int index)
{
  switch (index)
  {
    case 1:
      tone(29, NOTE_C7, 100);
      //noTone(29);
      break;
  }
}

#define BUFFPIXEL 80

// -=-=-=-=-=-=-=-=-=-= WriteRexct BMP Draw -=-=-=-=-=-=-=-=-=-

void bmpDraw(const char *infilename, uint8_t xin, uint16_t yin) {

  File     bmp_file;
  int      bmpWidth, bmpHeight;         // W+H in pixels
  uint8_t  bmpDepth;                    // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;              // Start of image data in file
  uint32_t rowSize;                     // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL];       // pixel buffer (R+G+B per pixel)
  uint16_t buffidx = sizeof(sdbuffer);  // Current position in sdbuffer
  boolean  goodBmp = false;             // Set to true on valid header parse
  boolean  flip    = true;              // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  uint16_t awColors[320];               // hold colors for one row at a time...

  if((xin >= tft.width()) || (yin >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(infilename);
  Serial.println('\'');

  // Open requested file on SD card
  if (!(bmp_file = SD.open(infilename))) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmp_file) == 0x4D42) {                      // BMP signature

    (void)read32(bmp_file);                             // Read & ignore creator bytes
    bmpImageoffset = read32(bmp_file);                  // Start of image data
    
    // Read DIB header
    bmpWidth  = read32(bmp_file);
    bmpHeight = read32(bmp_file);
    if(read16(bmp_file) == 1) {                         // # planes -- must be '1'
      bmpDepth = read16(bmp_file);                      // bits per pixel
    
      if((bmpDepth == 24) && (read32(bmp_file) == 0)) { // 0 = uncompressed

        goodBmp = true;                                 // Supported BMP format -- proceed!        
        rowSize = (bmpWidth * 3 + 3) & ~3;              // BMP rows are padded (if needed) to 4-byte boundary
                
        if(bmpHeight < 0) {                             // If bmpHeight is negative, image is in top-down order.
          bmpHeight = -bmpHeight;                       
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((xin+w-1) >= tft.width())  w = tft.width()  - xin;
        if((yin+h-1) >= tft.height()) h = tft.height() - yin;

        for (row=0; row<h; row++) {        

          if(flip)                                      // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else                                          // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmp_file.position() != pos) {              // Need seek?
            bmp_file.seek(pos);
            buffidx = sizeof(sdbuffer);                 // Force buffer reload
          }

          for (col=0; col<w; col++) {                   // For each pixel...
                                                        // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) {          // Yes
              bmp_file.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0;                              // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            awColors[col] = tft.color565(r,g,b);
          } // end pixel
          tft.writeRect(0, row, w, 1, awColors);
        } // end scanline
      }
    }
  }

  bmp_file.close();
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

void u8g2_bitmapStr(byte Ident) {

  switch(Ident)
  {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x06:
    case 0x07:
    case 0x08:
      u8g2.drawXBMP(0, 0, image_width, image_height, StrImg01);
      break;
    case 0x09:
      u8g2.drawXBMP(0, 0, image_width, image_height, StrImg02);
      break;
    case 0x0A:
    case 0x0B:
      u8g2.drawXBMP(0, 0, image_width, image_height, StrImg03);
      break;
    case 0x04:
    case 0x05:
      u8g2.drawXBMP(0, 0, image_width, image_height, StrImg04);
      break;
    case 0xFF:
      u8g2.drawXBMP(0, 0, image_width, image_height, BlankImg);
      break;
    default:
      u8g2.drawXBMP(0, 0, image_width, image_height, StrImg01);
      break;
    
  }
}

void u8g2_bitmapTrn(byte Ident) {
 switch(Ident)
  {
    case 0x00:
    case 0x03:
    case 0x06:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg01);
      break;
    case 0x01:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg02);
      break;
    case 0x02:
    case 0x07:
    case 0x08:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg03);
      break;
    case 0x09:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg04);
      break;
    case 0x0A:
    case 0x0B:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg05);
      break;
    case 0x04:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg06);
      break;
    case 0x05:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg07);
      break;
    case 0xFF:
      u8g2.drawXBMP(0, 0, image_width, image_height, BlankImg);
      break;
    default:
      u8g2.drawXBMP(0, 0, image_width, image_height, TrnImg01);
      break;
    
  }
}

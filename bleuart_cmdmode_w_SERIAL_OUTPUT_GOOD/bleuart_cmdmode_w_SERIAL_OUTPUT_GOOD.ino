#include <Adafruit_NeoPixel.h>
 
//for neo pixel setup
#define PIN 8
 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
    #define BLUEFRUIT_HWSERIAL_NAME           Serial1
    #define BLUEFRUIT_UART_MODE_PIN         -1   // Not used with FLORA
    #define BLUEFRUIT_UART_CTS_PIN          -1   // Not used with FLORA
    #define BLUEFRUIT_UART_RTS_PIN          -1   // Not used with FLORA


    
/*=========================================================================*/


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
int led = 7;
int inPin = 9;
int val = 0;

void setup(void)
{
  
  
  //while (!Serial);  // required for Flora & Micro
  //delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));


  ble.begin(VERBOSE_MODE);
  /*if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }*/
  Serial.println( F("OK!") );

  /*if ( FACTORYRESET_ENABLE )
  {
    // Perform a factory reset to make sure everything is in a known state 
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }*/

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  //Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  //Serial.println(F("Then Enter characters to send to Bluefruit"));
  //Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(inPin, INPUT);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  
}
 
/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  
  
  // Check for user input
  char inputs[BUFSIZE+1];

  while ( ble.available() )
  //if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response status
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();

  
  
  String teststring;
  char bufferValue[65];
  
  bool redTrue;
  bool blueTrue;
  bool greenTrue;
  
  Serial.println(F("[Recv] ")); 
  Serial.println(ble.buffer);
  if (strcmp(ble.buffer, "hello") == 0)
  {
    //this is the hand shake to make sure communication is ready
    ble.print("AT+BLEUARTTX=");
    ble.println("HELLO MY FRIEND HELLO");
  }
  else if(strcmp(ble.buffer, "r") == 0)
  {
                    Serial.println("----------------you chose RED----------------------"); 
                    //delay(1000);
    
                    redTrue = true;
                    greenTrue = false;
                    blueTrue = false;
                    
                   // while(redTrue){
  
                    colorWipe(strip.Color(255, 0, 0), 500); // RED
                      delay(1000);
                    colorWipe(strip.Color(0, 0, 0), 500); // BLANK
                      delay(1000);
                    //}
                    ble.print("AT+BLEUARTTX=");
                    ble.println("RED");
    
  }
  else if (strcmp(ble.buffer, "g") == 0)
  {
      Serial.println("----------------you chose GREEN----------------------"); 
      
      //delay(1000);
    
      redTrue = false;
      greenTrue = true;
       blueTrue = false;
       
      //while(greenTrue){
  
                    colorWipe(strip.Color(0, 255, 0), 500); // GREEN
                      delay(1000);
                    colorWipe(strip.Color(0, 0, 0), 500); // BLANK
                      delay(1000);
                 //   }
                    ble.print("AT+BLEUARTTX=");
                    ble.println("GREEN");
      
      
    
  }
  else if(strcmp(ble.buffer, "b") == 0)
  {
    Serial.println("----------------you chose BLUE----------------------"); 
    //delay(1000);
    redTrue = false;
      greenTrue = false;
       blueTrue = true;
   // while(blueTrue){
  
                    colorWipe(strip.Color(0, 0, 255), 500); // BLUe
                      delay(1000);
                    colorWipe(strip.Color(0, 0, 0), 500); // BLANK
                      delay(1000);
                   // }
                      ble.print("AT+BLEUARTTX=");
                      ble.println("BLUE");
  }
   else if(strcmp(ble.buffer, "a") == 0)
  {
         Serial.println("----------------you chose RAINBOW----------------------"); 
         //delay(1000);
    
                    redTrue = false;
                    greenTrue = false;
                    blueTrue = true;
                 // while(true){
  
                    colorWipe(strip.Color(0, 0, 0), 500); // GREEN
                      delay(1000);
                    rainbowCycle(20);
                      delay(1000);
                   // }
                   ble.print("AT+BLEUARTTX=");
                    ble.println("RAINBOW");
  }

  


  

  

  
  //--iadded these lines-------
  Serial.print("[Send] back to phone");
  //Serial.println("hello returing your text twice");
  //Serial.println(ble.buffer);    
    
  //this command sends data back to the device it recieved inout from
 //String testString = "hello Wprld";
  //for (int i=0; i <= 64; i++){
    //  ble.print("AT+BLEUARTTX=");
     /// ble.println(bufferValue[i]); 
      //ble.println(testString);
    // delay(10);
  // } 
  
  
  //---------------------------
  ble.waitForOK();
}
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
 
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}
 
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}



/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (Serial.peek() < 0) && !timeout.expired() ) {}

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.peek() < 0) );

  return true;
}

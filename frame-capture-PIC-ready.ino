/*
  PIC-Ready Frame Capture
 */

#include <Arduino.h>
#include <SPI.h>

// SPI pins:
#define PIN_RESET     9        
#define PIN_NCS       10
#define LED           8

// Signal delay time:
#define ADNS3080_T_IN_RST             500
#define ADNS3080_T_PW_RESET           10
#define ADNS3080_T_SRAD_MOT           75
#define ADNS3080_T_SWW                50
#define ADNS3080_T_SRAD               50
#define ADNS3080_T_LOAD               10
#define ADNS3080_T_BEXIT              4

// Pixel dimensions:
#define ADNS3080_PIXELS_X             30
#define ADNS3080_PIXELS_Y             30

// Registers: 
#define ADNS3080_PRODUCT_ID           0x00
#define ADNS3080_CONFIGURATION_BITS   0x0a
#define ADNS3080_MOTION_CLEAR         0x12
#define ADNS3080_FRAME_CAPTURE        0x13
#define ADNS3080_PIXEL_BURST          0x40
#define ADNS3080_MOTION_BURST         0x50
#define ADNS3080_PRODUCT_ID_VALUE     0x17

// Convert a pixel into a character
char pixelSymbol(int k) {
  constexpr char scale[] = "0123456789abcdef";
  return scale[k >> 4];                               // Divide uint8_t by 4
}

void setup() {
  sensorSetup( false, false );
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);  
  Serial.begin(9600);
}

void loop() {
  // Array to store frame
  uint8_t frame[ADNS3080_PIXELS_X][ADNS3080_PIXELS_Y];    // The number of pixels are included in the header file. Nx = Ny = 30

  // Retrieve frame
  frameCapture( frame );

  // Scan axes and display pixel
  for(int x = 0; x < ADNS3080_PIXELS_X; x += 1 ) {
    for(int y = 0; y < ADNS3080_PIXELS_Y; y += 1 ) {
      Serial.print( pixelSymbol( frame[x][y] ) );
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
}



void reset()
{
  digitalWrite( PIN_RESET, HIGH );
  delayMicroseconds( ADNS3080_T_PW_RESET );                  
  digitalWrite( PIN_RESET, LOW );
  delayMicroseconds( ADNS3080_T_IN_RST );              
}

void writeRegister( const uint8_t reg, uint8_t output ) {
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( reg | B10000000 );
  
  // Send value
  SPI.transfer( output );
  
  // Disable communcation
  digitalWrite( PIN_NCS, HIGH );
  delayMicroseconds( ADNS3080_T_SWW );
}


uint8_t readRegister( const uint8_t reg ) {
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( reg );
  delayMicroseconds( ADNS3080_T_SRAD_MOT );
  
  // Receive value
  uint8_t output = SPI.transfer(0x00);
  
  // Dissable communication
  digitalWrite( PIN_NCS, HIGH ); 
  return output;
}




// Initialize and configure sensor
bool sensorSetup( const bool led_mode, const bool resolution )
{
  // Configure SPI
  SPI.begin();
  SPI.setClockDivider( SPI_CLOCK_DIV32 );
  SPI.setDataMode( SPI_MODE3 );
  SPI.setBitOrder( MSBFIRST );  
  
  // Set sensor pins
  pinMode( PIN_RESET, OUTPUT );
  pinMode( PIN_NCS,   OUTPUT );

  // Disable communication and reset 
  digitalWrite( PIN_NCS, HIGH );
  reset();
 
  // Configure sensor:
  //                          LED Shutter    High resolution
  uint8_t mask = B00000000 | led_mode << 6 | resolution << 4;      
  writeRegister( ADNS3080_CONFIGURATION_BITS, mask );

  // Check Connection
  if( readRegister(ADNS3080_PRODUCT_ID) == ADNS3080_PRODUCT_ID_VALUE ) {
    return true;
  } else {
    return false;
  } 
}

void frameCapture( uint8_t output[ADNS3080_PIXELS_X][ADNS3080_PIXELS_Y] )
{  
  // Store pixel values
  writeRegister( ADNS3080_FRAME_CAPTURE, 0x83 );
  
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( ADNS3080_PIXEL_BURST );
  delayMicroseconds( ADNS3080_T_SRAD );

  //-- First pixel:
  uint8_t pixel = 0;

  // Recieve pixels until first is found 
  while( (pixel & B01000000) == 0 ) {
      pixel = SPI.transfer(0x00);
      delayMicroseconds( ADNS3080_T_LOAD );  
  }

  //-- Scan first frame:
  for( int y = 0; y < ADNS3080_PIXELS_Y; y += 1 ) {
    for( int x = 0; x < ADNS3080_PIXELS_X; x += 1 ) {  
      
      // Store and scale past pixel
      output[x][y] = pixel << 2; 
      
      // Receive new pixel
      pixel = SPI.transfer(0x00);
      delayMicroseconds( ADNS3080_T_LOAD );  
    }
  }
  // Disable communication
  digitalWrite( PIN_NCS, HIGH ); 
  delayMicroseconds( ADNS3080_T_LOAD + ADNS3080_T_BEXIT );
}    

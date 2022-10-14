#include <Adafruit_NeoPixel.h>

// Bluetooth Goggles Sketch -- shows the Adafruit Bluefruit LE UART Friend
// can be used even with Trinket or Gemma!

// https://www.adafruit.com/products/2479

// Works in conjunction with Bluefruit LE Connect app on iOS or Android --
// pick colors 
// or 
// '1' =  Pinwheel mode
// '2' =  Sparkle mode
// '3' =  Color Wheel
// '4' =  Follow the Leader
// down_arrow = turn goggles off  
// up_arrow = Pinwheel mode multicolor
// All of the above take 93% of available space on the Trinket.
// You can try adding more, but space is VERY tight...helps to use Arduino IDE
// 1.6.4 or later; produces slightly smaller code than the 1.0.X releases.

// BLUEFRUIT LE UART FRIEND MUST BE SWITCHED TO 'UART' MODE

#include <Adafruit_NeoPixel.h>
#include <Adafruit_DotStar.h>
#ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.
 #include <avr/power.h>
#endif

#define RX_PIN    3 // Connect this Trinket pin to BLE 'TXO' pin
#define CTS_PIN   2 // Connect this Trinket pin to BLE 'CTS' pin
#define LED_PIN   0 // Connect NeoPixels to this Trinket pin
#define NUM_LEDS 32 // Two 16-LED NeoPixel rings
#define FPS      30 // Animation frames/second (ish)
#define SWITCH    1

  // DotStar Code
  #define NUMPIXELS 1 // Number of LEDs in strip

  // Here's how to control the LEDs from any two pins:
  #define DATAPIN    7
  #define CLOCKPIN   8
  Adafruit_DotStar dot(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN);

void setup() {
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  // MUST do this on 16 MHz Trinket for serial & NeoPixels!
  clock_prescale_set(clock_div_1);
#endif
  Serial1.begin(9600);

  // Set CTS to 1
    pinMode(CTS_PIN, OUTPUT); digitalWrite(CTS_PIN, HIGH);

  dot.begin();
  dot.setBrightness(80);
  dot.setPixelColor(0, 0,0,255);
  dot.show(); // Blue

  // Setup Digital#1 as input with pull down.
  // this uses the battery +V as input to the switch 
  // and a SPST OFF (ON) switch so the USB interface is
  // not impacted.
  pinMode(SWITCH, INPUT_PULLDOWN);
  // Red LED
  pinMode(13, OUTPUT);

  pixels.begin(); // NeoPixel init
  // Flash space is tight on Trinket/Gemma, so setBrightness() is avoided --
  // it adds ~200 bytes.  Instead the color picker input is 'manually' scaled.
}

uint8_t  buf[3],              // Enough for RGB parse; expand if using sensors
         animMode = 0,        // Current animation mode
         animPos  = 0;        // Current animation position
uint32_t color_array[2] = {0x400000, 0xFF0000},  // Current animation color (red by default), Color wheel
         prevTime = 0L;       // For animation timing
uint8_t last = 0;         
uint8_t  i=0;

void loop(void) {
  int      c;
  uint32_t t;
  uint8_t  j=0;  

  //dot.setPixelColor(0, 0,255,0);
  //dot.show();

  digitalWrite(13, LOW); // Turn off the red led 
  //digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
  //delay(1000); // wait for a second
  //digitalWrite(13, LOW); // turn the LED off by making the voltage LOW
  //delay(1000); // wait for a second


  
  // Animation happens at about 30 frames/sec.  Rendering frames takes less
  // than that, so the idle time is used to monitor incoming serial data.
  digitalWrite(CTS_PIN, LOW); // Signal to BLE, OK to send data!
  for(;;) {
    t = micros();                            // Current time
    if((t - prevTime) >= (1000000L / FPS)) { // 1/30 sec elapsed?
      prevTime = t;
      break;                                 // Yes, go update LEDs
    }
      if((c = Serial1.read()) == '!') {            // Received UART app input?
        digitalWrite(13, HIGH);  // Turn on red led if we get a button press
      while((c = Serial1.read()) < 0);           // Yes, wait for command byte
      switch(c) {
       case 'B':       // Button (Control Pad)
        if(readAndCheckCRC(255-'!'-'B', buf, 2) & (buf[1] == '1')) {
          buttonPress(buf[0]); // Handle button-press message
        }
        break;
       case 'C':       // Color Picker
        if(readAndCheckCRC(255-'!'-'C', buf, 3)) {
          // As mentioned earlier, setBrightness() was avoided to save space.
          // Instead, results from the color picker (in buf[]) are divided
          // by 4; essentially equivalent to setBrightness(64).  This is to
          // improve battery run time (NeoPixels are still plenty bright).
          color_array[0] = pixels.Color(buf[0]/4, buf[1]/4, buf[2]/4);
        }
        break;
       case 'Q':       // Quaternion
        skipBytes(17); // 4 floats + CRC (see note below re: parsing)
        break;
       case 'A':       // Accelerometer
#if 0
        // The phone sensors are NOT used by this sketch, but this shows how
        // they might be read.  First, buf[] must be delared large enough for
        // the expected data packet (minus header & CRC) -- that's 16 bytes
        // for quaternions (above), or 12 bytes for most of the others.
        // Second, the first arg to readAndCheckCRC() must be modified to
        // match the data type (e.g. 'A' here for accelerometer).  Finally,
        // values can be directly type-converted to float by using a suitable
        // offset into buf[] (e.g. 0, 4, 8, 12) ... it's not used in this
        // example because floating-point math uses lots of RAM and code
        // space, not suitable for the space-constrained Trinket/Gemma, but
        // maybe you're using a Pro Trinket, Teensy, etc.
        if(readAndCheckCRC(255-'!'-'A', buf, 12)) {
          float x = *(float *)(&buf[0]),
                y = *(float *)(&buf[4]),
                z = *(float *)(&buf[8]);
        }
        // In all likelihood, updates from the buttons and color picker
        // alone are infrequent enough that you could do without any mention
        // of the CTS pin in this code.  It's the extra sensors that really
        // start the firehose of data.
        break;
#endif
       case 'G':       // Gyroscope
       case 'M':       // Magnetometer
       case 'L':       // Location
        skipBytes(13); // 3 floats + CRC
      }
    }
  }
  digitalWrite(CTS_PIN, HIGH); // BLE STOP!
  
  // Check switch if ON state and wait for OFF..
  if(digitalRead(SWITCH))
  {

    // tried a while(xx); loop here but it failed?
    for (;;)
    {
      if(! digitalRead(SWITCH))
      {
        break;
      }
    }
    
    // Now we can switch states... 
    pixels.clear(); // Clear pixel data when switching modes (else residue)
    animMode += 1;    
    if ( (animMode == 3) || (animMode ==2) )
    {
      i = 0;
    }
      
    if (animMode == 5)
    {
      animMode = 0;
    }
      
  }

  // Show pixels calculated on *prior* pass; this ensures more uniform timing
  pixels.show();

  // Then calculate pixels for *next* frame...
  switch(animMode) 
  {
    case 0: // Pinwheel mode
    case 1: // pinwheel multcolor       
      for(i=0; i<NUM_LEDS/2; i++)
      {
        uint32_t c = 0;
        if(((animPos + i) & 7) < 2) c = color_array[animMode]; // 4 pixels on...
        pixels.setPixelColor(   i, c);         // First eye
        pixels.setPixelColor(NUM_LEDS-1-i, c); // Second eye (flipped)
        if(animMode == 1)
        {
          color_array[1] >>= 8;
          if (color_array[1] == 0x00) color_array[1] = 0xFF0000;
        }
      }
      animPos++;
      break;
    case 4: // Sparkle mode
      pixels.setPixelColor(animPos, 0);     // Erase old dot
      animPos = random(NUM_LEDS);           // Pick a new one
      pixels.setPixelColor(animPos, color_array[0]); // and light it
      break;
    case 2: // Color Wheel
    case 3: // Follow the leader    
      {
        pixels.setPixelColor(i, color_array[1]);
        pixels.setPixelColor(i+16, color_array[1]);
        if(animMode == 3)
        {
          if (i>=2)
          {
            pixels.setPixelColor(i-3, 0);
            pixels.setPixelColor(i+13, 0);              
          }
          if(last)
          {
            pixels.setPixelColor(16-last, 0);
            pixels.setPixelColor(32-last, 0);
            last--;
          }
        } // mode == 3                       
        color_array[1] >>= 8;
        if (color_array[1] == 0x00) color_array[1] = 0xFF0000;
//        delay(100);
      }
      i++;
      if(i == 16)
      {
        i = 0;
        if(animMode == 2)
        {
          for(j=0; j<32; j++) pixels.setPixelColor(j, 0);
        }
        if(animMode == 3)
        {
          last = 3;
        }
      }
      break;
    case 5: // Do nothing and show nothing turn off leds
        break;                                 
  }
}

boolean readAndCheckCRC(uint8_t sum, uint8_t *buf, uint8_t n) {
  for(int c;;) {
    while((c = Serial1.read()) < 0); // Wait for next byte
    if(!n--) return (c == sum);  // If CRC byte, we're done
    *buf++ = c;                  // Else store in buffer
    sum   -= c;                  // and accumulate sum
  }
}

void skipBytes(uint8_t n) {
  while(n--) {
    while(Serial1.read() < 0);
  }
}

void buttonPress(char c) {
  pixels.clear(); // Clear pixel data when switching modes (else residue)
  switch(c) {
   case '1':
    animMode = 0; // Switch to pinwheel mode
    break;
   case '2':
    animMode = 4; // Switch to sparkle mode
    break;
   case '3':
    i = 0;
    animMode = 2; // Color wheel
    break;
   case '4':
    i = 0;
    animMode = 3; // follow the leader
    break;
   case '5': // Up  pinwheel multicolor
    animMode = 1;
    break;
   case '6': // Down off 
    animMode = 5;
    break;
   case '7': // Left
    break;
   case '8': // Right
    break;
  }
}

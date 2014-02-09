#include "ControlPanel.h"


//********************************************
// manual control panel
//********************************************
//
// A push button (PB) along with two status leds (LED0 and LED1)
// can be used to select between several configuration sets stored in eeprom.
// 
// LED0 and LED1 show the current configuration set
//
//    LED1  LED0
//    ----------------------------------
//     off   on   configuration set #1
//      on  off   configuration set #2
//      on   on   configuration set #3
//
// how to select another configuration set
//   (1.) press PB at least 2 seconds
//        enters configuration mode, LEDs flashing
//   (2.) press PB sveral times until desired config set is shown
//        LEDs show selected config set. LEDs still flashing
//   (3.) press PB at least 2 seconds
//        exits configuration mode and makes selection permanent
//


// initialize the pin modes for the control panel
void initControlPanelPins() {
#ifdef USE_CONTROL_PANEL
  pinMode (ST_LED0, OUTPUT);
  pinMode (ST_LED1, OUTPUT);
  pinMode (PB_PIN, INPUT_PULLUP);
#endif
}

// function is called every 20ms 
void handleControlPanel() {
#ifdef USE_CONTROL_PANEL

  uint8_t pbIn = 0;
  static uint8_t pbInStatePrev = 0;
  static uint8_t pbDbCount = 0;
  static bool pbInDb = false;
   
  static uint8_t pbStateCount = 0;   
  static pbState_t pbStateVar = PB_IDLE;
   
  static uint8_t blinkCount = 0;
  uint8_t ledBlink;
  uint8_t vLedOn;
  uint8_t vLed0;
  uint8_t vLed1;
   
  // read push button
  pbIn = digitalRead(PB_PIN);
  
  // debounce
  if (pbIn == pbInStatePrev) {
    if (pbDbCount < 3) {
      pbDbCount++;
    } else {
      pbInDb = (pbIn == 0)? true : false;  
    }
  } else {
    pbDbCount = 0;
  }
  pbInStatePrev = pbIn;
 
  // press long: enter Paramter Set Mode
  // press short: increment Parameter Set
  //   leds show state and blink
  // press long: save change
  //   leds show Paramter Set 
  
  // Push state machine
  switch (pbStateVar) {
  case PB_IDLE:
    if (pbInDb == true) {  // pb pressed
      if (pbStateCount < 100) { // wait 2 sec
      pbStateCount++; 
      } else {
      pbStateVar = PB_ENTER_IN;
      pbStateCount = 0;
      }
    } else {
      pbStateCount = 0;
    }
    break;
  case PB_ENTER_IN:
    if (pbInDb == false) {  // pb released
      pbStateVar = PB_ENTER_WAIT;
    }
    break; 
  case PB_ENTER_WAIT:
    if (pbInDb == true) {  // pb pressed
      pbStateVar = PB_ENTER_PRESSED;
    }
    break; 
  case PB_ENTER_PRESSED:
    if (pbInDb == false) {  // pb release
      pbStateVar = PB_ENTER_WAIT;
      pbStateCount = 0;
      // increment parameter set
      config.configSet++;
      if (config.configSet >= NUM_EEPROM_CONFIG_SETS) {
        config.configSet = 0;
      }
    } else {  // pb pressed
      if (pbStateCount < 100) { // wait 2 sec
        pbStateCount++; 
      } else {
        pbStateVar = PB_IDLE_IN;
        pbStateCount = 0;
        // confirm selected parameterset
        readEEPROM();
      }
    }
    break;
  case PB_IDLE_IN:
    if (pbInDb == true) {  // pb pressed
      pbStateVar = PB_IDLE;
    }
    break; 
  }

  switch (pbStateVar) {
    case PB_IDLE:
      ledBlink = 0; break;
    case PB_ENTER_IN:
      ledBlink = 1; break;
    case PB_ENTER_WAIT:
      ledBlink = 1; break;
    case PB_ENTER_PRESSED:
      ledBlink = 1; break;
    case PB_IDLE_IN:
      ledBlink = 0; break;    
    default:
      ledBlink = 0; break;  
  }

  // handle blinking
  blinkCount++;
  if (blinkCount >10) {
    blinkCount = 0;
  }
  vLedOn = (blinkCount < 7) ? 1 : 0;
  vLedOn = ledBlink == 0 ? 1 : vLedOn;

  // put status to leds display
  vLed0 = (config.configSet + 1) & 1;
  vLed1 = ((config.configSet + 1) >> 1) & 1;
  vLed0 &= vLedOn;
  vLed1 &= vLedOn;
  
  SET_LED( ST_LED0, vLed0 );
  SET_LED( ST_LED1, vLed1 );

#endif  
}



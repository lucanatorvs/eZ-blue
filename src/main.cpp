#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// I would love to port this to a real time os, but than i would need to port freeRTOS to the arduino nano every
// TODO: port freeRTOS to the arduino nano every

/*
    * nano every (ATMega4809) pin configuration
    * https://docs.arduino.cc/static/a9b7676afe52b3943b02b3033e51b831/ABX00028-datasheet.pdf

    Pin name  |  Function           |  use                        |  ATMega port and pin
    ------------------------------------------------------------------------------------------------
    D2        |  Blower             |  digital output             |  PA0 (port A, pin 0)
    D3        |  rpm gauge          |  frequency output           |  PF5 (port F, pin 5)
    D4        |  CAN interrupt      |  digital input (interrupt)  |  PC6 (port C, pin 6)
    D5        |  fuel gauge         |  pwm output                 |  PB2 (port B, pin 2)
    D6        |  temperature gauge  |  pwm output                 |  PF4 (port F, pin 4)
    D7        |  temp sensor CS     |  digital output             |  PA1 (port A, pin 1)
    D8        |  CAN CS             |  digital output             |  PE3 (port E, pin 3)
    D9        |  battery light      |  digital output             |  PB0 (port B, pin 0)
    D10       |  oil light          |  digital output             |  PB1 (port B, pin 1)
    D11       |  MOSI               |  SPI                        |  PB3 (port B, pin 3)
    D12       |  MISO               |  SPI                        |  PB4 (port B, pin 4)
    D13       |  SCK                |  SPI                        |  PB5 (port B, pin 5)
    AREF      |  fault HVIL         |  digital output             |  PD7 (port D, pin 7)
    D14       |  heater switch      |  digital input              |  PD3 (port D, pin 3)
    D15       |  Bi metal switch    |  digital input              |  PD2 (port D, pin 2)
    D16       |  heater contactor   |  digital output             |  PD1 (port D, pin 1)
    D17       |  Drive mode 1       |  digital input              |  PD0 (port D, pin 0)
    D18       |  Drive mode 3       |  digital input              |  PA2 (port A, pin 2)
    D19       |  hydrolic pump      |  digital output             |  PA3 (port A, pin 3)
    D20       |  vacuum pump        |  digital output             |  PD4 (port D, pin 4)
    D21       |  running input      |  digital input              |  PD5 (port D, pin 5)
*/

/* SPI info
 * the 2 spi devices are connected to the same spi bus
 * the CAN Module has a CS on pin D8
 * the CAN module has a interrupt pin on D4
 * the temperature sensor has a CS on pin D7
*/

/****************************************************************************************/
/******************************  Define pins and constants ******************************/
/****************************************************************************************/

// define a debug mode to tern off the serial output when not needed
#define DEBUG 1 // probably just leave this on even in production so any errors can be debugged
#define CAN_DEBUG 1 // set to 1 to print the can messages to the serial monitor

// define the pins with human readable names
#define BLOWER_PIN 2 // PA0
#define RPM_GAUGE_PIN 3 // PF5
#define FUEL_GAUGE_PIN 5 // PC6
#define TEMP_GAUGE_PIN 6 // PF4
#define TEMP_SENSOR_CS_PIN 7 // PA1
#define CAN_CS_PIN 8 // PE3
#define BATTERY_LIGHT_PIN 9 // PB0
#define OIL_LIGHT_PIN 10 // PB1
#define FAULT_HVIL_PIN AREF // aref is PD7
#define HEATER_SWITCH_PIN 14 // PD3
#define BI_METAL_SWITCH_PIN 15 // PD2
#define HEATER_CONTACTOR_PIN 16 // PD1
#define DRIVE_MODE_1_PIN 17 // PD0
#define DRIVE_MODE_3_PIN 18 // PA2
#define HYDRO_PUMP_PIN 19 // PA3
#define VACUUM_PUMP_PIN 20 // PD4
#define RUNNING_INPUT_PIN 21 // PD5
#define CAN_INTERRUPT_PIN 4 // PC6
#define MOSI_PIN 11 // PB3
#define MISO_PIN 12 // PB4
#define SCK_PIN 13 // PB5

// define bit masks for the drive mode pins
#define bit0 0x01 // 0000 0001
#define bit1 0x02 // 0000 0010
#define bit2 0x04 // 0000 0100
#define bit3 0x08 // 0000 1000
#define bit4 0x10 // 0001 0000
#define bit5 0x20 // 0010 0000
#define bit6 0x40 // 0100 0000
#define bit7 0x80 // 1000 0000

// define constants
#define SetHeaterTemp 65 // set the heater temperature to 50 degrees

/****************************************************************************************/
/******************************  Function prototypes  ***********************************/
/****************************************************************************************/

double read_temperature();
// void irqHandler();

/****************************************************************************************/
/******************************  Global Variables  **************************************/
/****************************************************************************************/

// gauge variables
uint8_t fuel_gauge = 0; // value between 0 and the max of a uint8_t (255)
uint8_t temperature_gauge = 0; // value between 0 and the max of a uint8_t (255)
uint8_t rpm_gauge = 0; // value between 0 and the max of a uint8_t (255)

// volatile bool interrupt = false;
struct can_frame frame;
struct can_frame canMsg;
MCP2515 mcp2515(8);

int last_time = 0; // used to keep track to the loop frequency
static int loop_delay = 10; // used to keep track to the loop frequency

double temperature = 0; // used to store the temperature
int last_temp_read = 0;

int temperature_sensor_loop_count = 0; // used to keep track of the temperature sensor loop count

int16_t rpm = 0; // used to store the rpm

/****************************************************************************************/
/******************************  Setup  *************************************************/
/****************************************************************************************/

void setup() {
  /******************************************/
  /****** start serial if in debug mode *****/
  /******************************************/

  #if DEBUG
    Serial.begin(115200);
    Serial.println("Serial started\n");
    // print clock speed
    Serial.print("Clock speed: ");
    Serial.print(F_CPU/1000000);
    Serial.println("MHz\n");
    Serial.println("eZ blue - firmware version: 0.1.0 - By: Luca van Straaten\n");
  #endif

  /************************************************************/
  /****** Setup the pins to In or Output and High or Low ******/
  /************************************************************/

  #if DEBUG
    Serial.println("Setting up pin(mode)s...\n");
  #endif

  // set the output pins to output mode
  PORTB_DIRSET = bit1; // OIL_LIGHT_PIN
  PORTB_DIRSET = bit0; // BATTERY_LIGHT_PIN
  PORTD_DIRSET = bit7; // FAULT_HVIL_PIN
  PORTA_DIRSET = bit3; // HYDRO_PUMP_PIN
  PORTD_DIRSET = bit4; // VACUUM_PUMP_PIN
  PORTD_DIRSET = bit1; // HEATER_CONTACTOR_PIN
  PORTA_DIRSET = bit0; // BLOWER_PIN

  // set the gauge pins to output mode
  PORTF_DIRSET = bit4; // TEMP_GAUGE_PIN pwm output
  PORTF_DIRSET = bit5; // RPM_GAUGE_PIN frequency output (not pwm) with Timer Counter B (TCB)
  PORTC_DIRSET = bit6; // FUEL_GAUGE_PIN pwm output

  // set the vac and blower pins to high
  PORTD_OUTSET = bit4; // VACUUM_PUMP_PIN
  PORTA_OUTSET = bit0; // BLOWER_PIN

  // set the rest of the output pins to low
  PORTB_OUTCLR = bit1; // OIL_LIGHT_PIN
  PORTB_OUTCLR = bit0; // BATTERY_LIGHT_PIN
  PORTD_OUTCLR = bit7; // FAULT_HVIL_PIN
  PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN
  PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN

  // set the input pins to input mode
  PORTD_DIRCLR = bit5; // RUNNING_INPUT_PIN
  PORTD_DIRCLR = bit0; // DRIVE_MODE_1_PIN
  PORTA_DIRCLR = bit2; // DRIVE_MODE_3_PIN
  PORTD_DIRCLR = bit2; // BI_METAL_SWITCH_PIN 
  PORTD_DIRCLR = bit3; // HEATER_SWITCH_PIN

  /**************************/
  /****** Setup the SPI *****/
  /**************************/

  #if DEBUG
    Serial.println("Setting up SPI...\n");
  #endif

  // begin the SPI bus
  SPI.begin();

  // set the SPI pins modes
  pinMode(TEMP_SENSOR_CS_PIN, OUTPUT);
  pinMode(CAN_CS_PIN, OUTPUT);
  pinMode(CAN_INTERRUPT_PIN, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);

  // set the CS pins high
  digitalWrite(TEMP_SENSOR_CS_PIN, HIGH);
  digitalWrite(CAN_CS_PIN, HIGH);

  /*******************************/
  /****** Start the CAN bus ******/
  /*******************************/

  #if DEBUG
    Serial.println("CAN setup - CAN_250KBPS - MCP_8MHZ\n");
  #endif

  // attachInterrupt(4, irqHandler, FALLING);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  /*******************************/
  /****** End of setup ***********/
  /*******************************/

  #if DEBUG
    Serial.println("Setup complete\n");
  #endif
}

/****************************************************************************************/
/******************************  Main Loop  *********************************************/
/****************************************************************************************/

void loop() {
  // main loop code

  /**********************************************************/
  /*************  Read all the sensors  *********************/
  /**********************************************************/

  // read the drive mode pins
  uint8_t drive_mode = 1;
  if (!(PORTD_IN & bit0)) {
    drive_mode = 0;
  }
  if (!(PORTA_IN & bit2)) {
    drive_mode = 2;
  }
  #if DEBUG
    Serial.print("Drive mode: ");
    Serial.print(drive_mode);
    Serial.print(" - ");
  #endif

  // read the running input pin
  uint8_t running = 0;
  if (!(PORTD_IN & bit5)) {
    running = 1;
  }
  #if DEBUG
    Serial.print("Running: ");
    Serial.print(running);
    Serial.print(" - ");
  #endif

  // read the bi-metal switch pin
  uint8_t bi_metal_switch = 0;
  if (!(PORTD_IN & bit2)) {
    bi_metal_switch = 1;
  }
  #if DEBUG
    Serial.print("Bi-metal switch: ");
    if (bi_metal_switch) {
      Serial.print("Closed");
    } else {
      Serial.print("Open");
    }
    Serial.print(" - ");
  #endif

  // read the heater switch pin
  uint8_t heater_switch = 1;
  if (!(PORTD_IN & bit3)) {
    heater_switch = 0;
  }
  #if DEBUG
    Serial.print("Heater switch: ");
    Serial.print(heater_switch);
    Serial.println(" - ");
  #endif


  // only read the tem sensor every 800ms, we know the loop_delay so we calculate every whitch loop we should read the sensor and count the loops
  if (temperature_sensor_loop_count >= 800 / loop_delay) {
    temperature_sensor_loop_count = 0;
    last_temp_read = millis();

    // #if DEBUG
    //   Serial.print("Reading temp sensor - ");
    // #endif

    // read the temperature sensor
    temperature = read_temperature();
  }
  temperature_sensor_loop_count++;

  #if DEBUG
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" - ");
  #endif

  /**********************************************************/
  /*************  Read the can bus  *************************/
  /**********************************************************/

  // #if DEBUG
  //   Serial.print("Interupt: ");
  //   Serial.print(interrupt);
  //   Serial.print(" - ");
  // #endif

  // interrupt = false;

  // // read the can bus
  // if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
  //   Serial.print("CAN ID: ");
  //   Serial.print(canMsg.can_id, HEX); // print ID
  //   Serial.print(" - ");
  // }

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    #if CAN_DEBUG
      Serial.print(canMsg.can_id, HEX); // print ID
      Serial.print(" "); 
      Serial.print(canMsg.can_dlc, HEX); // print DLC
      Serial.print(" ");
    #endif
    if (canMsg.can_id == 6) {
      // take the 3d and 4th byte and convert it to a int
      rpm = (canMsg.data[3] << 8) | canMsg.data[2];
      #if DEBUG
        Serial.print("RPM: ");
        Serial.print(rpm);
        Serial.print(" - ");
      #endif
      int8_t temp1 = canMsg.data[0];
      int8_t temp2 = canMsg.data[1];
      #if DEBUG
        Serial.print("Temp1: ");
        Serial.print(temp1);
        Serial.print(" - ");
        Serial.print("Temp2: ");
        Serial.print(temp2);
        Serial.print(" - ");
      #endif
    }
  }

  /**********************************************************/
  /*************  Move the gauges  **************************/
  /**********************************************************/

  // temperature_gauge = map(temperature, 0, 80, 0, 255);

  // fuel gauge
  analogWrite(FUEL_GAUGE_PIN, fuel_gauge);

  // temperature gauge
  analogWrite(TEMP_GAUGE_PIN, temperature_gauge);

  // rpm gauge
  rpm_gauge = map(rpm, 0, 7000, 0, 230);
  // mappedValue is never lower than 10
  if (rpm_gauge < 10) {
    rpm_gauge = 10;
  } else if (rpm_gauge > 230) {
    rpm_gauge = 230;
  }
  #if DEBUG
    Serial.print("RPM 2:");
    Serial.print(rpm);
    Serial.print(" - ");
    Serial.print("RPM gauge: ");
    Serial.print(rpm_gauge);
    Serial.print(" - ");
  #endif
  tone(RPM_GAUGE_PIN, rpm_gauge);

  /**********************************************************/
  /*************  Set the outputs  **************************/
  /**********************************************************/

  // depending on the drive mode set the vacuum pump and hydro pump
  if (drive_mode == 0) {
    PORTD_OUTCLR = bit4; // VACUUM_PUMP_PIN high
    PORTA_OUTSET = bit3; // HYDRO_PUMP_PIN high
  } else if (drive_mode == 1) {
    PORTD_OUTSET = bit4; // VACUUM_PUMP_PIN low
    PORTA_OUTSET = bit3; // HYDRO_PUMP_PIN high
  } else if (drive_mode == 2) {
    PORTD_OUTSET = bit4; // VACUUM_PUMP_PIN low
    PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN low
  }

  /**********************************************************/
  /*************  Do the hrater stuff ***********************/
  /**********************************************************/

  // if the heater switch is on (HEATER_SWITCH_PIN is high) and the bi-metal switch closed (BI_METAL_SWITCH_PIN is Low)
  // then we can check if we can turn on the heater contacter

  if (heater_switch && bi_metal_switch && running) {
    // now we check if the temperature is below the SetHeaterTemp
    // we need some hystereses
    int hysteresis = 2;
    if (temperature < (SetHeaterTemp - hysteresis)) {
      // turn on the heater contacter
      PORTD_OUTSET = bit1; // HEATER_CONTACTOR_PIN
      PORTB_OUTSET = bit0; // BATTERY_LIGHT_PIN
      Serial.print("Heater on - ");
    } else if (temperature > (SetHeaterTemp + hysteresis)) {
      // turn off the heater contacter
      PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN
      PORTB_OUTCLR = bit0; // BATTERY_LIGHT_PIN
      Serial.print("Heater off - ");
    }
  } else {
    PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN
    PORTB_OUTCLR = bit0; // BATTERY_LIGHT_PIN
    Serial.print("Heater off - ");
  }

  // tern the blower on if the heater is on or still warm
  if ((heater_switch && running) || temperature > 40) {
    PORTA_OUTCLR = bit0; // BLOWER_PIN
    Serial.print("Blower on - ");
  } else {
    PORTA_OUTSET = bit0; // BLOWER_PIN
    Serial.print("Blower off - ");
  }

  /**********************************************************/
  /*************  End the loop  *****************************/
  /**********************************************************/

  #if DEBUG
    int time = millis();
    int loop_time = time - last_time;
    last_time = time;
    Serial.print("Loop frequency: ");
    Serial.print(1000/loop_time);
    Serial.println("Hz");
    Serial.println("\n");
  #endif
  delay(loop_delay);
}

/****************************************************************************************/
/******************************  Functions  *********************************************/
/****************************************************************************************/

// void irqHandler() {
//   interrupt = true;
//   uint8_t irq = mcp2515.getInterrupts();

//   if (irq & MCP2515::CANINTF_RX0IF) {
//     if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
//       // frame contains received from RXB0 message
//       Serial.print("RXB0 CAN ID: ");
//       Serial.print(canMsg.can_id, HEX); // print ID
//       Serial.print(" - ");
//     } else {
//       // error reading message
//       Serial.println("Error reading RXB0 message");
//     }
//   }

//   if (irq & MCP2515::CANINTF_RX1IF) {
//     if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
//       // frame contains received from RXB1 message
//       Serial.print("RXB1 CAN ID: ");
//       Serial.print(canMsg.can_id, HEX); // print ID
//       Serial.print(" - ");
//     } else {
//       // error reading message
//       Serial.println("Error reading RXB1 message");
//     }
//   }
// }

double read_temperature() { // see: https://gist.github.com/sleemanj/059fce7f1b8087edfe7d7ef845a5d881

  SPI.end();
  delay(1);

  uint16_t v; // 16 bit value to hold the raw temperature value
  digitalWrite(TEMP_SENSOR_CS_PIN, LOW); // enable the temperature sensor
  delay(2); // wait for the sensor to wake up

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MISO_PIN, SCK_PIN, MSBFIRST); // MSB
  v <<= 8; // shift MSB to MSB position
  v |= shiftIn(MISO_PIN, SCK_PIN, MSBFIRST); // LSB
  
  digitalWrite(TEMP_SENSOR_CS_PIN, HIGH); // disable chip

  SPI.begin();

  if (v & bit2) {
    #if DEBUG
      Serial.print("Thermocouple open circuit - ");
    #endif
    return NAN; // thermocouple is open circuit
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}

/****************************************************************************************/
/******************************  Ene of code  *******************************************/
/****************************************************************************************/


// #include <SPI.h>
// #include <mcp2515.h>

// struct can_frame canMsg;
// MCP2515 mcp2515(8);


// void setup() {
//   Serial.begin(115200);
  
//   mcp2515.reset();
//   mcp2515.setBitrate(CAN_500KBPS);
//   mcp2515.setNormalMode();
  
//   Serial.println("------- CAN Read ----------");
//   Serial.println("ID  DLC   DATA");
// }

// void loop() {
//   if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
//     Serial.print(canMsg.can_id, HEX); // print ID
//     Serial.print(" "); 
//     Serial.print(canMsg.can_dlc, HEX); // print DLC
//     Serial.print(" ");
    
//     for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
//       Serial.print(canMsg.data[i],HEX);
//       Serial.print(" ");
//     }

//     Serial.println();      
//   }
// }
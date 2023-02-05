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
#define CAN_DEBUG 0 // set to 1 to print the can messages to the serial monitor

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
int16_t rpm_gauge = 0; // value between 0 and the max of a uint8_t (255)

// volatile bool interrupt = false;
struct can_frame frame;
struct can_frame canMsg;
MCP2515 mcp2515(8);

int last_time = 0; // used to keep track to the loop frequency
static int loop_delay = 4; // used to keep track to the loop frequency

double temperature = 0; // used to store the temperature
int last_temp_read = 0;

int temperature_sensor_loop_count = 0; // used to keep track of the temperature sensor loop count

int16_t rpm = 0; // used to store the rpm
int16_t temp_motor = 0; // used to store the temperature for the motor
int8_t temp_inverter = 0; // used to store the temperature for the inverter
int8_t temp_bat = 0; // used to store the temperature for the battery
int16_t CURRENT = 0; // used to store the current
int16_t CHARGE = 0; // used to store the charge in 0.1 Ah
int8_t SOC = 0; // used to store the state of charge in %
uint8_t chargingstatus = 0; // used to store the charging status
uint8_t last_chargingstatus = 0; // used to store the last charging status
unsigned long lastToggleTime = 0; // used to store the last time the charging status changed
bool Batlight = false; // used to store the state of the battery light

int8_t maxCelModTemp = 0; // used to store the max cell module temperature
int8_t maxCelTemp = 0; // used to store the max cell temperature
int8_t minCelTemp = 0; // used to store the min cell temperature

const float SOC_LUT[] = {110, 100, 90, 87, 83, 80, 77, 73, 70, 69, 68, 67, 66, 64, 63, 62, 61, 60, 57, 53, 50, 45, 40, 38, 35, 32, 30, 29, 29, 28, 27, 26, 26, 25, 24, 24, 23, 22, 21, 21, 20, 20, 19, 19, 18, 18, 18, 17, 17, 16, 16, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7}; // lookup table for the state of charge

/****************************************************************************************/
/******************************  Setup  *************************************************/
/****************************************************************************************/

void setup() {
  /******************************************/
  /****** start serial if in debug mode *****/
  /******************************************/

  // if debug is enabled or can debug is enabled start the serial monitor
  #if DEBUG || CAN_DEBUG
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
    Serial.print(", ");
  #endif

  // read the running input pin
  uint8_t running = 0;
  if (!(PORTD_IN & bit5)) {
    running = 1;
  }
  #if DEBUG
    Serial.print("Running: ");
    Serial.print(running);
    Serial.print(", ");
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
    Serial.print(", ");
  #endif

  // read the heater switch pin
  uint8_t heater_switch = 1;
  if (!(PORTD_IN & bit3)) {
    heater_switch = 0;
  }
  #if DEBUG
    Serial.print("Heater switch: ");
    Serial.print(heater_switch);
    Serial.print(", ");
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
    Serial.print("Heater temperature: ");
    Serial.print(temperature);
    Serial.print(", ");
  #endif

  /**********************************************************/
  /*************  Read the can bus  *************************/
  /**********************************************************/

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    #if CAN_DEBUG
      if (1){
        Serial.print(canMsg.can_id, HEX); // print ID
        Serial.print(" "); 
        Serial.print(canMsg.can_dlc, HEX); // print DLC
        Serial.print(" ");
        for (uint8_t i = 0; i < canMsg.can_dlc; i++) {
          Serial.print(canMsg.data[i], DEC); // print data
          Serial.print(" ");
        }
        Serial.println("");
      }
    #endif
    if (canMsg.can_id == 6) {
      #if CAN_DEBUG
        Serial.print("Motor Message - ");
      #endif
      // take the 3d and 4th byte and convert it to a int
      rpm = (canMsg.data[3] << 8) | canMsg.data[2];
      temp_motor = canMsg.data[0];
      temp_inverter = canMsg.data[1];
      // convert the temperature to celsius from fahrenheit
      temp_motor = (temp_motor - 32) * 5 / 9;
      temp_inverter = (temp_inverter - 32) * 5 / 9;
    }
    if (canMsg.can_id == 2578777344) {
      #if CAN_DEBUG
        Serial.print("BMS Message - ");
      #endif
      CURRENT = (canMsg.data[0] << 8) | canMsg.data[1];
      CHARGE = (canMsg.data[2] << 8) | canMsg.data[3];
      SOC = canMsg.data[6];
    }
    if (canMsg.can_id == 0x99B50000) {
      #if CAN_DEBUG
        Serial.print("Charging Message - ");
      #endif
      // take the 3d and 4th byte and convert it to a int
      chargingstatus = canMsg.data[3];
    }
    if (canMsg.can_id == 0x99B50002) {
      #if CAN_DEBUG
        Serial.print("Cel Module Temp Message - ");
      #endif
      // byte 1 is Highest cell module temperature in the battery pack, encoded in 1 degree Celsius with basis of -100 C.
      maxCelModTemp = canMsg.data[1];
      maxCelModTemp = maxCelModTemp - 100;
    }
    if (canMsg.can_id == 0x99B50008) {
      #if CAN_DEBUG
        Serial.print("Cel Temp Message - ");
      #endif
      // byte 0 Lowest cell temperature in the battery pack, encoded in 1 degree Celsius with basis of -100 C
      // byte 1 is Highest cell temperature in the battery pack, encoded in 1 degree Celsius with basis of -100 C.
      minCelTemp = canMsg.data[0];
      minCelTemp = minCelTemp - 100;
      maxCelTemp = canMsg.data[1];
      maxCelTemp = maxCelTemp - 100;
    }
  }

  #if DEBUG
    // Serial.print("\n");
    Serial.print("Motor temp: ");
    Serial.print(temp_motor);
    Serial.print(", ");
    Serial.print("Inverter temp: ");
    Serial.print(temp_inverter);
    Serial.print(", ");
  #endif

  #if DEBUG
    Serial.print("Max Cel Mod Temp: ");
    Serial.print(maxCelModTemp);
    Serial.print(", ");
    Serial.print("Min Cel Temp: ");
    Serial.print(minCelTemp);
    Serial.print(", ");
    Serial.print("Max Cel Temp: ");
    Serial.print(maxCelTemp);
    Serial.print(", ");
    // Serial.print("\n");
  #endif

  #if DEBUG
    Serial.print("Current: ");
    Serial.print(CURRENT);
    Serial.print(", ");
    Serial.print("Charge: ");
    Serial.print(CHARGE);
    Serial.print(", ");
    Serial.print("SOC: ");
    Serial.print(SOC);
    Serial.print(", ");
  #endif

  #if DEBUG
    Serial.print("charging status: ");
    Serial.print(chargingstatus);
    Serial.print(", ");
  #endif

  #if DEBUG
    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(", ");
    // Serial.print("\n");
  #endif

  /**********************************************************/
  /*************  Move the gauges  **************************/
  /**********************************************************/

  // temperature_gauge = map(temperature, 0, 80, 0, 255);

  // fuel gauge
  /*
  pwm   sog
  0 = Not Defined
  7 = 100
  8 = 99
  9 = 90
  10 = 77
  12 = 67
  15 = 52
  20 = 40
  30 = 26
  40 = 22
  45 = 21
  50 = 20
  60 = 17
  70 = 8
  80 = 5
  90 = 2
  100 = 1
  110 = 0
   */

  // q: what function do we need to map the fuel level to the fuel gauge


  // set it to half
  fuel_gauge = SOC_LUT[SOC];
  // fuel_gauge = 110; // 0 - 255
  analogWrite(FUEL_GAUGE_PIN, fuel_gauge);

  // mapping the individual temperature to a rainge of 0 - 255
  // temp_motor -> -40 C tot 135 C, thermal shutdown at 165 C
  // temp_inverter -> -40 C tot 80 C, thermal shutdown at 95 C
  // maxCelModTemp -> -40 C tot 80 C, temperature might increase during cell balancing, so we mighit not want to use this
  // maxCelTemp -> discharging: -25 C tot 55 C, charging: 0 C tot 55 C

  // we map the values so that the opearting range is between 64 and 192
  // 64 = 1/4 of 255
  // 192 = 3/4 of 255
  // 128 = 1/2 of 255

  #define blue 0
  #define red 160

  maxCelTemp = 0;
  maxCelModTemp = 0;
  temp_inverter = 0;
  temp_motor = 130;

  // temp_motor = temp_motor / 3;
  int16_t temp_motor_16 = map(temp_motor, -5, 135, blue, red);

  // temp_inverter = temp_inverter / 2;
  int16_t temp_inverte_16 = map(temp_inverter, -5, 80, blue, red);

  // maxCelModTemp = maxCelModTemp / 2;
  int16_t maxCelModTemp_16 = map(maxCelModTemp, -5, 80, blue, red);

  // maxCelTemp = (maxCelTemp * 3) - 10;
  int16_t maxCelTemp_16 = map(maxCelTemp, -5, 55, blue, red);

  // get the max of all the temperatures
  int16_t maxofalltemps = max(temp_motor_16, temp_inverte_16);
  maxofalltemps = max(maxofalltemps, maxCelModTemp_16);
  maxofalltemps = max(maxofalltemps, maxCelTemp_16);
  

  // tem gauge mapping
  /*
  pwm = temp
  20 = 10c = 16%
  40 = 50%
  192 = 90%
  */

 // 255 = 0E

//  maxofalltemps = maxofalltemps;

  uint8_t temp_gauge = maxofalltemps;

  // temperature gauge
  analogWrite(TEMP_GAUGE_PIN, temp_gauge);

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
    Serial.print(", ");
    Serial.print("RPM gauge: ");
    Serial.print(rpm_gauge);
    Serial.print(", ");
  #endif
  tone(RPM_GAUGE_PIN, rpm_gauge);

  /**********************************************************/
  /*************  Set the outputs  **************************/
  /**********************************************************/

  if (running) {
    // depending on the drive mode set the vacuum pump and hydro pump
    if (drive_mode == 0) {
      PORTD_OUTCLR = bit4; // VACUUM_PUMP_PIN high
      PORTA_OUTSET = bit3; // HYDRO_PUMP_PIN high
    } else if (drive_mode == 1) {
      PORTD_OUTCLR = bit4; // VACUUM_PUMP_PIN high
      PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN low
    } else if (drive_mode == 2) {
      PORTD_OUTSET = bit4; // VACUUM_PUMP_PIN low
      PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN low
    }
  } else {
    PORTD_OUTSET = bit4; // VACUUM_PUMP_PIN low
    PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN low
  }

  /**********************************************************/
  /*************  Do the hrater stuff  **********************/
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
      PORTB_OUTSET = bit1; // OIL_LIGHT_PIN
      #if DEBUG
        Serial.print("Heater on - ");
      #endif
    } else if (temperature > (SetHeaterTemp + hysteresis)) {
      // turn off the heater contacter
      PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN
      PORTB_OUTCLR = bit1; // OIL_LIGHT_PIN
      #if DEBUG
        Serial.print("Heater off - ");
      #endif
    }
  } else {
    PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN
    PORTB_OUTCLR = bit1; // OIL_LIGHT_PIN
    #if DEBUG
      Serial.print("Heater off - ");
    #endif
  }

  // tern the blower on if the heater is on or still warm
  if ((heater_switch && running) || temperature > 35) {
    PORTA_OUTCLR = bit0; // BLOWER_PIN
    #if DEBUG
      Serial.print("Blower on - ");
    #endif
  } else {
    PORTA_OUTSET = bit0; // BLOWER_PIN
    #if DEBUG
      Serial.print("Blower off - ");
    #endif
  }

  /**********************************************************/
  /*************  Charging status light  ********************/
  /**********************************************************/

  // charging status
  // 0 = Disconnected - light off
  // 1 = Pre-heating - blinking fast 1Hz
  // 2 = Pre-charging - light on
  // 3 = Main Charging - light on
  // 4 = Balancing - blinking slow 0.5Hz
  // 5 = Charging Finished - blinking slow 0.25Hz
  // 6 = Charging Error - blinking fast 1Hz

  // variable to store the current time
  unsigned long currentMillis = millis();

  // check if the charging status has changed
  if (chargingstatus != last_chargingstatus) {
    // reset the last time the light was toggled
    lastToggleTime = currentMillis;
    // set the last charging status
    last_chargingstatus = chargingstatus;
  }

  if (chargingstatus == 0) { // Disconnected
    Batlight = false;
  } else if (chargingstatus == 1) { // Pre-heating
    // check if the current time is greater than the last time the light was toggled
    if (currentMillis - lastToggleTime >= 500) {
      // toggle the light
      Batlight = !Batlight;
      // reset the last time the light was toggled
      lastToggleTime = currentMillis;
    }
  } else if (chargingstatus == 2 || chargingstatus == 3) { // Pre-charging or Main Charging
    Batlight = true;
  } else if (chargingstatus == 4) { // Balancing
    // check if the current time is greater than the last time the light was toggled
    if (currentMillis - lastToggleTime >= 1000) {
      // toggle the light
      Batlight = !Batlight;
      // reset the last time the light was toggled
      lastToggleTime = currentMillis;
    }
  } else if (chargingstatus == 5) { // Charging Finished
    // check if the current time is greater than the last time the light was toggled
    if (currentMillis - lastToggleTime >= 2000) {
      // toggle the light
      Batlight = !Batlight;
      // reset the last time the light was toggled
      lastToggleTime = currentMillis;
    }
  } else if (chargingstatus == 6) { // Charging Error
    // check if the current time is greater than the last time the light was toggled
    if (currentMillis - lastToggleTime >= 500) {
      // toggle the light
      Batlight = !Batlight;
      // reset the last time the light was toggled
      lastToggleTime = currentMillis;
    }
  }

  if (Batlight){
    PORTB_OUTSET = bit0;
  } else {
    PORTB_OUTCLR = bit0;
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
    // Serial.println("\n");
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
//       Serial.print(", ");
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
//       Serial.print(", ");
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
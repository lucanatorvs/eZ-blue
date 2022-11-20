#include <Arduino.h>
#include <SPI.h>

// define a debug mode to tern off the serial output when not needed
#define DEBUG 1

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
    D11       |  MOSI               |  SPI                        |  P
    D12       |  MISO               |  SPI                        |  P
    D13       |  SCK                |  SPI                        |  P
    AREF      |  fault HVIL         |  digital output             |  PD7 (port D, pin 7)
    D14       |  heater switch      |  digital input              |  PA
    D15       |  Bi metal switch    |  digital input              |  PA
    D16       |  heater contactor   |  digital output             |  PA
    D17       |  Drive mode 1       |  digital input              |  PA
    D18       |  Drive mode 3       |  digital input              |  PA
    D19       |  hydrolic pump      |  digital output
    D20       |  vacuum pump        |  digital output
    D21       |  running input      |  digital input

*/

/* SPI info
 * the 2 spi devices are connected to the same spi bus
 * the CAN Module has a CS on pin D8
 * the CAN module has a interrupt pin on D4
 * the temperature sensor has a CS on pin D7
*/

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
#define bit0 0x01
#define bit1 0x02
#define bit2 0x04
#define bit3 0x08
#define bit4 0x10
#define bit5 0x20
#define bit6 0x40
#define bit7 0x80

//function prototypes
double read_temperature();
// void set_gauges();
// void set_lights();
// void HVI_fault();

void setup() {
  // setup code

  // set the clock speed to 20MHz
  // this is the maximum speed for the nano every


  // initialize the spi bus
  SPI.begin();
  // SPI.setBitOrder(MSBFIRST);
  // SPI.setDataMode(SPI_MODE0);
  // SPI.setClockDivider(SPI_CLOCK_DIV4);

  // start serial if in debug mode
  #if DEBUG
    Serial.begin(115200);
    Serial.println("Serial started");
    // print clock speed
    Serial.print("Clock speed: ");
    Serial.print(F_CPU/1000000);
    Serial.println("MHz\n");
  #endif

  // set the pins to output mode
  PORTB_DIRSET = bit1; // OIL_LIGHT_PIN
  PORTB_DIRSET = bit0; // BATTERY_LIGHT_PIN
  PORTD_DIRSET = bit7; // FAULT_HVIL_PIN
  PORTA_DIRSET = bit3; // HYDRO_PUMP_PIN
  PORTD_DIRSET = bit4; // VACUUM_PUMP_PIN
  PORTD_DIRSET = bit1; // HEATER_CONTACTOR_PIN
  PORTA_DIRSET = bit0; // BLOWER_PIN

  // set the output pins to low
  PORTB_OUTCLR = bit1; // OIL_LIGHT_PIN
  PORTB_OUTCLR = bit0; // BATTERY_LIGHT_PIN
  PORTD_OUTCLR = bit7; // FAULT_HVIL_PIN
  PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN
  PORTD_OUTCLR = bit4; // VACUUM_PUMP_PIN
  PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN
  PORTA_OUTCLR = bit0; // BLOWER_PIN

  // set the pins to input mode
  PORTD_DIRCLR = bit5; // RUNNING_INPUT_PIN
  PORTD_DIRCLR = bit0; // DRIVE_MODE_1_PIN
  PORTA_DIRCLR = bit2; // DRIVE_MODE_3_PIN
  PORTD_DIRCLR = bit2; // BI_METAL_SWITCH_PIN 
  PORTD_DIRCLR = bit3; // HEATER_SWITCH_PIN

  // SPI setup
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

  #if DEBUG
    Serial.println("Setup complete\n");
  #endif
}

void loop() {
  // main loop code
  // PORTB_OUTSET = bit1; // OIL_LIGHT_PIN
  // delay(100);
  // PORTB_OUTSET = bit0; // BATTERY_LIGHT_PIN
  // delay(100);
  // PORTD_OUTSET = bit7; // FAULT_HVIL_PIN
  // delay(100);
  // PORTA_OUTSET = bit3; // HYDRO_PUMP_PIN
  // delay(100);
  // PORTD_OUTSET = bit4; // VACUUM_PUMP_PIN
  // delay(100);
  // PORTD_OUTSET = bit1; // HEATER_CONTACTOR_PIN
  // delay(100);
  // PORTA_OUTSET = bit0; // BLOWER_PIN
  // delay(1000);
  // PORTB_OUTCLR = bit1; // OIL_LIGHT_PIN
  // delay(100);
  // PORTB_OUTCLR = bit0; // BATTERY_LIGHT_PIN
  // delay(100);
  // PORTD_OUTCLR = bit7; // FAULT_HVIL_PIN
  // delay(100);
  // PORTA_OUTCLR = bit3; // HYDRO_PUMP_PIN
  // delay(100);
  // PORTD_OUTCLR = bit4; // VACUUM_PUMP_PIN
  // delay(100);
  // PORTD_OUTCLR = bit1; // HEATER_CONTACTOR_PIN
  // delay(1000);
  // PORTA_OUTCLR = bit0; // BLOWER_PIN

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
    Serial.print(bi_metal_switch);
    Serial.print(" - ");
  #endif

  // read the heater switch pin
  uint8_t heater_switch = 0;
  if (!(PORTD_IN & bit3)) {
    heater_switch = 1;
  }
  #if DEBUG
    Serial.print("Heater switch: ");
    Serial.print(heater_switch);
    Serial.print(" - ");
  #endif

  // read the temperature sensor
  double temperature = read_temperature();
  #if DEBUG
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" - ");
  #endif

  // open the HVIL relay (active high) if the bi-metal switch is open and the temperature is NAN
  if (!bi_metal_switch && isnan(temperature)) {
    PORTD_OUTCLR = bit7; // FAULT_HVIL_PIN
    #if DEBUG
      Serial.print("HVIL fault - ");
    #endif
  } else {
    PORTD_OUTSET = bit7; // FAULT_HVIL_PIN
    #if DEBUG
      Serial.print("HVIL OK - ");
    #endif
  }


  #if DEBUG
    Serial.println("\n");
  #endif
  delay(1000);
}

double read_temperature() {

  SPI.end();

  uint16_t v; // 16 bit value to hold the raw temperature value
  digitalWrite(TEMP_SENSOR_CS_PIN, LOW); // enable the temperature sensor
  delay(1); // wait for the sensor to wake up

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

  #if DEBUG
    Serial.print("Raw temperature: ");
    Serial.print(v);
    Serial.print(" - ");
  #endif

  if (v & bit2) {
    #if DEBUG
      Serial.print("Thermocouple open circuit - ");
    #endif
    return NAN; // check for open circuit
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}
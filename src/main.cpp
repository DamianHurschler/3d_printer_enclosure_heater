#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <U8g2lib.h>
#include <math.h>
#include "pid.h"

// SH1106 LILYGO 1.3" T-Beam OLED display using I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 23);
const uint8_t * display_font = u8g2_font_logisoso16_tf; // Set display font - https://github.com/olikraus/u8g2/wiki/fntlist12

// T-Beam 1.3 Inch OLED SH1106 Display Buttons
#define BUTTON_1_PIN 14 //      IO25
#define BUTTON_2_PIN 27 // (A0) IO39
#define BUTTON_3_PIN 26 //      IO36
bool button1_pressed = false;
bool button_enable_pressed = false;
bool button3_pressed = false;
int button_debounce = 300;
volatile double last_interrupt_time = 0;

// Create an instance of the BME680 sensor
Adafruit_BME680 bme; // I2C
float temp_offset = -3; // Offset to apply to sensor reading to correct temperature.

// Definitions for PID controller
#define PID_KP  40.0f
#define PID_KI  0.0002f //0.2f
#define PID_KD  0.0001f //0.1f
#define PID_INTEGRAL_MAX 70.0f
#define PID_INTEGRAL_MIN  0.0f
#define PID_OUTPUT_MAX 100.0f
#define PID_OUTPUT_MIN  0.0f
#define PID_INTERVAL_MS  1000UL
bool pid_initialised = false;
float measurement = 0.0f;
float set_point = 26.0f;
float pid_out = 0.0f;
unsigned long last_run = 0UL;

// Pass definitions to pid_data struct
pid_data pid = { PID_KP, PID_KI, PID_KD,
                PID_OUTPUT_MAX, PID_OUTPUT_MIN,
                PID_INTEGRAL_MAX, PID_INTEGRAL_MIN, 
                PID_INTERVAL_MS };

// Define PWM parameters
bool pwm_enable = false;
char output_state [4] = "OFF";  // String with 3 characters (plus one null character) to display ON or OFF
const int pwmPin = 15;          // PWM output pin number
const int pwmChannel = 0;       // PWM channel (0-15)
const int pwmFrequency = 4;     // Frequency of PWM signal in Hz, minimum is 4
const int pwmResolution = 8;    // PWM resolution in bits (8 bits gives 256 levels)
float pwm_scaling_factor = 0;   // Scaling factor to get from percent to binary
int dutyCycle_bin = 0;          // Resulting binary value for pwm




float read_sensor(){
    // Perform a measurement and confirm it's available
    if (!bme.performReading()) {
      Serial.println("Failed to perform reading :(");
    }
    bme.temperature = bme.temperature + temp_offset;
    return bme.temperature;
}




int set_output(float pid_out, bool pwm_enable){
  // Set PWM duty cycle
  // Calculate resolution based on 'pwmResolution' and scale it to 100% duty cycle
  // Example: 8bit res has 2pow8-1, i.e. 255 steps of resolution to cover 0-100% duty cycle
  pwm_scaling_factor = (pow(2, pwmResolution) -1) / 100;
  dutyCycle_bin = pwm_scaling_factor * pid_out;
  ledcWrite(pwmChannel, dutyCycle_bin); // Write to PWM pin
  return 0;
}




void serial_print(pid_data *pid){
  // Print temperature
  // Serial.print("Temperature = ");
  // Serial.print(bme.temperature);
  // Serial.println(" *C");

  // Print humidity
  // Serial.print("Humidity = ");
  // Serial.print(bme.humidity);
  // Serial.println(" %");

  // Print pressure
  // Serial.print("Pressure = ");
  // Serial.print(bme.pressure / 100.0); // Convert to hPa
  // Serial.println(" hPa");

  // Print gas resistance
  // Serial.print("Gas Resistance = ");
  // Serial.print(bme.gas_resistance / 1000.0); // Convert to KOhms
  // Serial.println(" KOhms");

  // Print PID variables
  Serial.printf("measurement: %.1f\n", measurement);
  Serial.printf("set_point: %.0f\n", set_point);
  Serial.printf("pid->interval: %u\n", pid->interval);
  Serial.printf("pid->proportional: %.1f\n", pid->proportional);
  Serial.printf("pid->integral: %.1f\n", pid->integral);
  Serial.printf("pid->derivative: %.1f\n", pid->derivative);
  Serial.printf("pid->pid_out: %.0f\n", pid->pid_out);

  // Print PWM output variables
  // Serial.printf("pwm_scaling_factor: %.2f\n", pwm_scaling_factor);
  Serial.printf("pwm_enable: %s\n", pwm_enable ? "true" : "false");
  // Serial.printf("output_state: %s\n", output_state);
  // Serial.printf("dutyCycle_bin: %u\n", dutyCycle_bin);
}




void update_display(float set_point, float pid_out){
  // Initialise variables for display test lines and parts of lines
  char line1 [16];
  char line2 [16];
  char line3_1 [16];
  char line3_2 [16];
  char line3_3 [16];

  // Convert data and text and store it in variables
  sprintf(line1, "%.1f°C  %.0f %RH", bme.temperature, bme.humidity);
  sprintf(line2, "SET %.0f°C", set_point);
  sprintf(line3_1, "PWM %.0f", pid_out);
  sprintf(line3_2, "%%");
  sprintf(line3_3, "%s", output_state);

  // Send text lines to display
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.drawUTF8(0,18, line1);
  u8g2.drawUTF8(0,41, line2);
  u8g2.drawUTF8(0,64, line3_1);
  u8g2.drawUTF8(75,64, line3_2);
  u8g2.drawUTF8(100,64, line3_3);
  u8g2.sendBuffer(); // transfer internal memory to the display
}




// Interrupt functions to handle button presses
void IRAM_ATTR ISR_button1_pressed() {
	button1_pressed = true;
}

void IRAM_ATTR ISR_button_enable_pressed() {
	button_enable_pressed = true;
}

void IRAM_ATTR ISR_button3_pressed() {
	button3_pressed = true;
}




void setup() {

  // Initialise serial debug interface
  Wire.begin();
  Serial.begin(115200);

  // Set the button pins as input with internal pull-up resistors
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);

  // Interrupts for button presses to call Interrupt Service Routines
  attachInterrupt(BUTTON_1_PIN, ISR_button1_pressed, FALLING);
  attachInterrupt(BUTTON_2_PIN, ISR_button_enable_pressed, FALLING);
  attachInterrupt(BUTTON_3_PIN, ISR_button3_pressed, FALLING);

  // Initialize the BME680 sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(200, 50); // 200*C for 50 ms

 // Initialize the SH1106 OLED display
  u8g2.begin();

  // Configure display and send start message to the OLED
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(display_font); // choose a suitable font
  u8g2.drawStr(0, 14, "Starting...");  // write something to the internal memory
  u8g2.setContrast(200);  // Maximum contrast
  u8g2.sendBuffer();                // transfer internal memory to the display

  // Configure PWM pin
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);

  // Initialise PID controller
  pid_init(&pid);
}




void loop() {
	if (button1_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      Serial.printf("Button 1 has been pressed\n");
      set_point = set_point + 1;
      last_interrupt_time = millis();
    }
    button1_pressed = false;
	}

  if (button_enable_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      Serial.printf("Enable button has been pressed\n");
      if (pwm_enable){
        pwm_enable = false;
        sprintf(output_state, "OFF");
      } else {
        pwm_enable = true;
        sprintf(output_state, "ON");
        pid_init(&pid); // Reset PID variables on reactivation of output
      }
      last_interrupt_time = millis();
    }
    button_enable_pressed = false;
	}

  if (button3_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      Serial.printf("Button 3 has been pressed\n");
      set_point = set_point - 1;
      last_interrupt_time = millis();
    }
    button3_pressed = false;
	}

  measurement = read_sensor();

  if (( (millis() - last_run ) > PID_INTERVAL_MS) & ( pwm_enable )) {
    pid_out = pid_update(&pid, set_point, measurement);
    last_run = millis();
  }

  if (!pwm_enable) {
    pid_out = 0;
  } 

  set_output(pid_out, pwm_enable);

  serial_print(&pid);

  update_display(set_point, pid_out);
}
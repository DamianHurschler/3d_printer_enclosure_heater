#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <U8g2lib.h>
#include <math.h>

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
signed int set_temp = 25;
float Kp = 40.0f;
float Ki = 0.2f;
float Kd = 0.1f;
float measurement = 0.0f;
float measurement_prev = 0.0f;
float error = 0.0f;
float error_prev = 0.0f;
float proportional = 0.0f;
float integral = 0.0f;
float integral_max = 70.0f;
float integral_min = 0.0f;
float derivative = 0.0f;
int interval = 1; //s
int pwm_output_max = 100;
int pwm_output_min = 0;
int pwm_output = 0;

// Define PWM parameters
bool pwm_enable = false;
char output_state [4] = "OFF";  // String with 3 characters (plus one null character) to display ON or OFF
const int pwmPin = 15;          // PWM output pin number
const int pwmChannel = 0;       // PWM channel (0-15)
const int pwmFrequency = 4;     // Frequency of PWM signal in Hz, minimum is 4
const int pwmResolution = 8;    // PWM resolution in bits (8 bits gives 256 levels)
float pwm_scaling_factor = 0;   // Scaling factor to get from percent to binary
int dutyCycle_bin = 0;          // Resulting binary value for pwm




// Read data from sensor, and make it available via 'bme' constructor
void read_sensor(){
    // Perform a measurement and confirm it's available
    if (!bme.performReading()) {
      Serial.println("Failed to perform reading :(");
    }
    bme.temperature = bme.temperature + temp_offset;
}




// Update PID controller numbers
void pid_control(){
    // Implement check to only start producing output if sensor reading was successful
    measurement = bme.temperature;
    error = set_temp - measurement;
    proportional = Kp * error;
    integral = integral + 0.5f * Ki * interval * (error + error_prev);
    if (integral > integral_max) {
      integral = integral_max;
    } else if (integral < integral_min) {
      integral =integral_min;
    }
    derivative = - (Kd * (measurement - measurement_prev) / interval);
    pwm_output = proportional + integral + derivative;
    if (pwm_output > pwm_output_max){ // Clamp to max output
      pwm_output = pwm_output_max;
    }
    if (pwm_output < pwm_output_min){ // Clamp to min output
      pwm_output = pwm_output_min;
    }
    error_prev = error;
    measurement_prev = measurement;
}




void set_output(){
  // Set PWM duty cycle
  // Calculate resolution based on 'pwmResolution' and scale it to 100% duty cycle
  // Example: 8bit res has 2pow8-1, i.e. 255 steps of resolution to cover 0-100% duty cycle
  if (!pwm_enable) {
    pwm_output = 0;
  } 
  pwm_scaling_factor = (pow(2, pwmResolution) -1) / 100;
  dutyCycle_bin = pwm_scaling_factor * pwm_output;
  ledcWrite(pwmChannel, dutyCycle_bin); // Write to PWM pin

}




void serial_print(){
  // Print temperature
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  // Print humidity
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  // Print pressure
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0); // Convert to hPa
  Serial.println(" hPa");

  // Print gas resistance
  Serial.print("Gas Resistance = ");
  Serial.print(bme.gas_resistance / 1000.0); // Convert to KOhms
  Serial.println(" KOhms");

  // Print PID variables
  Serial.printf("proportional: %.1f\n", proportional);
  Serial.printf("integral: %.1f\n", integral);
  Serial.printf("derivative: %.1f\n", derivative);
  Serial.printf("pwm_output: %u\n", pwm_output);

  // Print PWM output variables
  Serial.printf("pwm_scaling_factor: %.2f\n", pwm_scaling_factor);
  Serial.printf("pwm_enable: %s\n", pwm_enable ? "true" : "false");
  Serial.printf("output_state: %s\n", output_state);
  Serial.printf("dutyCycle_bin: %u\n", dutyCycle_bin);
}




void once_per_second(void * arg){
  while(1){
    read_sensor();

    pid_control();

    set_output();

    serial_print();

    delay(1000);
  }
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

  // Display a welcome message on the OLED
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(display_font); // choose a suitable font
  u8g2.drawStr(0, 14, "Starting...");  // write something to the internal memory
  u8g2.setContrast(200);  // Maximum contrast
  u8g2.sendBuffer();                // transfer internal memory to the display
 
  // Create independent task which will run continuously 'in the background'
  xTaskCreate(once_per_second, "once_per_second", 4096, NULL, 2, NULL);

  // Configure PWM pin
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
}




void loop() {

	if (button1_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      Serial.printf("Button 1 has been pressed\n");
      set_temp = set_temp + 1;
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
      }
      last_interrupt_time = millis();
    }
    button_enable_pressed = false;
	}

  if (button3_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      Serial.printf("Button 3 has been pressed\n");
      set_temp = set_temp - 1;
      last_interrupt_time = millis();
    }
    button3_pressed = false;
	}

  // Display a message on the OLED
  char line1 [16];
  char line2 [16];
  char line3_1 [16];
  char line3_2 [16];
  char line3_3 [16];
  sprintf(line1, "%.1f°C  %.0f %RH", bme.temperature, bme.humidity);
  sprintf(line2, "SET %d°C", set_temp);
  sprintf(line3_1, "PWM %u", pwm_output);
  sprintf(line3_2, "%%");
  sprintf(line3_3, "%s", output_state);
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.drawUTF8(0,18, line1);
  u8g2.drawUTF8(0,41, line2);
  u8g2.drawUTF8(0,64, line3_1);
  u8g2.drawUTF8(70,64, line3_2);
  u8g2.drawUTF8(100,64, line3_3);
  u8g2.sendBuffer(); // transfer internal memory to the display
}

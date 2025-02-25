#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <U8g2lib.h>

// Enable serial debugging
bool enable_serial = true;

// SH1106 LILYGO 1.3" T-Beam OLED display using I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 23);
const uint8_t * display_font = u8g2_font_bytesize_tf; // Set display font - https://github.com/olikraus/u8g2/wiki/fntlist12

// T-Beam 1.3 Inch OLED SH1106 Display Buttons
#define BUTTON_1_PIN 14 //      IO25
#define BUTTON_2_PIN 27 // (A0) IO39
#define BUTTON_3_PIN 26 //      IO36
bool button1_pressed = false;
bool button2_pressed = false;
bool button3_pressed = false;
int button_debounce = 300;
volatile double last_interrupt_time = 0;

// Create an instance of the BME680 sensor
Adafruit_BME680 bme; // I2C
float temp_offset = -3; // Offset to apply to sensor reading to correct temperature.

// Definitions for PID controller
signed int set_temp = 40;
float Kp = 2;
float Ki = 0.1;
float Kd = 0;
float measurement = 0;
float error = 0;
float error_prev = 0;
float integral = 0;
float derivative = 0;
int interval = 1; //s
int pwm_output_max = 100;
float pwm_output_min = 0;
int pwm_output = 0;

// Define the GPIO pin to use for PWM output
const int pwmPin = 15; 

// Define PWM parameters
const int pwmChannel = 0;       // PWM channel (0-15)
const int pwmFrequency = 1000;  // Frequency of PWM signal in Hz
const int pwmResolution = 8;    // PWM resolution in bits (8 bits gives 256 levels)




// Read data from sensor, and make it available via 'bme' constructor
void read_sensor(void * arg){
  while(1){
    // Perform a measurement and confirm it's available
    if (!bme.performReading()) {
      if (enable_serial){
      Serial.println("Failed to perform reading :(");
      }
    }
    bme.temperature = bme.temperature + temp_offset;
    delay(1000);
  }
}




// Update PID controller numbers and apply change to output
void pid_control(void * arg){
  while(1){
    // Implement check to only start producing output if sensor reading was successful
    measurement = bme.temperature;
    error = set_temp - measurement;
    if (pwm_output < pwm_output_max && pwm_output > pwm_output_min){ // Only keep integrating as long as we are in a sane range - anti-windup
      integral = integral + (error * interval);
    }
    derivative = (error - error_prev) / interval;
    pwm_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    if (pwm_output > pwm_output_max){ // Clamp to max output
      pwm_output = pwm_output_max;
    }
    if (pwm_output < pwm_output_min){ // Clamp to min output
      pwm_output = pwm_output_min;
    }
    error_prev = error;
    Serial.printf("proportional: %.1f\n", (Kp * error));
    Serial.printf("integral: %.1f\n", (Ki * integral));
    Serial.printf("derivative: %.1f\n", (Kd * derivative));
    Serial.printf("pwm_output: %u\n", pwm_output);
    delay((interval * 1000));
  }
}




void data_to_serial() {
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
}




// Interrupt functions to handle button presses
void IRAM_ATTR ISR_button1_pressed() {
	button1_pressed = true;
}

void IRAM_ATTR ISR_button2_pressed() {
	button2_pressed = true;
}

void IRAM_ATTR ISR_button3_pressed() {
	button3_pressed = true;
}




void setup() {

  if (enable_serial){
    // Initialise serial debug interface
    Wire.begin();
    Serial.begin(115200);
  }

  // Set the button pins as input with internal pull-up resistors
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);

  // Interrupts for button presses to call Interrupt Service Routines
  attachInterrupt(BUTTON_1_PIN, ISR_button1_pressed, FALLING);
  attachInterrupt(BUTTON_2_PIN, ISR_button2_pressed, FALLING);
  attachInterrupt(BUTTON_3_PIN, ISR_button3_pressed, FALLING);

  // Initialize the BME680 sensor
  if (!bme.begin()) {
    if (enable_serial){
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
    }
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
  xTaskCreate(read_sensor, "read_sensor", 4096, NULL, 2, NULL);

  // Create independent task which will run continuously 'in the background'
  xTaskCreate(pid_control, "pid_control", 4096, NULL, 2, NULL);

  // Configure the PWM functionalitiy on the specified pin
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);

  // Attach the PWM channel to the specified GPIO pin
  ledcAttachPin(pwmPin, pwmChannel);

}




void loop() {

	if (button1_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      if (enable_serial){
        Serial.printf("Button 1 has been pressed\n");
      }
      set_temp = set_temp + 1;
      last_interrupt_time = millis();
    }
    button1_pressed = false;
	}

  if (button2_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      if (enable_serial){
        Serial.printf("Button 2 has been pressed\n");
      }
      last_interrupt_time = millis();
    }
    button2_pressed = false;
	}

  if (button3_pressed) {
    if ((millis() - last_interrupt_time) > button_debounce){
      if (enable_serial){
        Serial.printf("Button 3 has been pressed\n");
      }
      set_temp = set_temp - 1;
      last_interrupt_time = millis();
    }
    button3_pressed = false;
	}

  if (enable_serial){
    data_to_serial();
  }

  // Display a message on the OLED
  char line1 [16];
  char line2 [16];
  char line3 [16];
  sprintf(line1, "%.1f°C   %.0f %RH", bme.temperature, bme.humidity);
  sprintf(line2, "set: %d°C", set_temp);
  sprintf(line3, "PWM: %u / 100", pwm_output);
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.drawUTF8(0,12, line1);
  u8g2.drawUTF8(0,30, line2);
  u8g2.drawUTF8(0,48, line3);
  u8g2.sendBuffer(); // transfer internal memory to the display

  // Set PWM duty cycle to 70%
  int dutyCycle = 178; // 178 (70%) of 255 (8-bit resolution)

  // Write the PWM duty cycle to the pin
  ledcWrite(pwmChannel, dutyCycle);
}

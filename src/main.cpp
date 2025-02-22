#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <U8g2lib.h>

// SH1106 LILYGO 1.3" T-Beam OLED display using I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

// T-Beam 1.3 Inch OLED SH1106 Display Buttons
#define BUTTON_1_PIN 14 //      IO25
#define BUTTON_2_PIN 27 // (A0) IO39
#define BUTTON_3_PIN 26 //      IO36

// Create an instance of the BME680 sensor
Adafruit_BME680 bme; // I2C

// Define the GPIO pin to use for PWM output
const int pwmPin = 15; 

// Define PWM parameters
const int pwmChannel = 0;       // PWM channel (0-15)
const int pwmFrequency = 1000;  // Frequency of PWM signal in Hz
const int pwmResolution = 8;    // PWM resolution in bits (8 bits gives 256 levels)

void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");

  // Set the button pins as input with internal pull-up resistors
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);

  // Initialize the BME680 sensor
  if (!bme.begin()) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
      while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Give some time for the serial connection to establish
  delay(1000);

 // Initialize the SH1106 OLED display
  u8g2.begin();

  // Display a welcome message on the OLED
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(0, 10, "Hello, World!");  // write something to the internal memory
  u8g2.setContrast(200);  // Maximum contrast
  u8g2.sendBuffer();                // transfer internal memory to the display

}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  // Read the button states
  bool button1Pressed = !digitalRead(BUTTON_1_PIN);
  bool button2Pressed = !digitalRead(BUTTON_2_PIN);
  bool button3Pressed = !digitalRead(BUTTON_3_PIN);

    // Perform a measurement and check if it's available
  if (!bme.performReading()) {
      Serial.println("Failed to perform reading :(");
  }

  // Read and print temperature
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  // Read and print humidity
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  // Read and print pressure
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0); // Convert to hPa
  Serial.println(" hPa");

  // Read and print gas resistance
  Serial.print("Gas Resistance = ");
  Serial.print(bme.gas_resistance / 1000.0); // Convert to KOhms
  Serial.println(" KOhms");

  // If Button 1 is pressed, display a different message
  if (button1Pressed) {
    Serial.println("Button 1 Pressed!");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "Button 1 Pressed!");
    u8g2.sendBuffer();
  }

  // If Button 2 is pressed, display a different message
  if (button2Pressed) {
    Serial.println("Button 2 Pressed!");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "Button 2 Pressed!");
    u8g2.sendBuffer();
  }

  // If Button 3 is pressed, display a different message
  if (button3Pressed) {
    Serial.println("Button 3 Pressed!");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "Button 3 Pressed!");
    u8g2.sendBuffer();
  }

  // Configure the PWM functionalitiy on the specified pin
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);

  // Attach the PWM channel to the specified GPIO pin
  ledcAttachPin(pwmPin, pwmChannel);

  // Set PWM duty cycle to 70%
  int dutyCycle = 178; // 178 (70%) of 255 (8-bit resolution)

  // Write the PWM duty cycle to the pin
  ledcWrite(pwmChannel, dutyCycle);

  delay(1000);  // wait 1 seconds for next scan
}

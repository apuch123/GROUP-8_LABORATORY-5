#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// STM32 I2C Pins (adjust according to your board)
#define I2C_SDA PB7
#define I2C_SCL PB6

// I2C Configuration
#define LCD_ADDR 0x27     // Verify with I2C scanner
#define TSL2561_ADDR 0x39 // Common TSL2561 address

// Light range calibration
#define MIN_LUX 0.0       // 0% light level
#define MAX_LUX 40000.0   // 100% light level (direct sunlight)

// Initialize components
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR);

// Sensor data processing
const uint8_t SAMPLE_SIZE = 5;
float luxReadings[SAMPLE_SIZE];
uint8_t currentReading = 0;
float luxTotal = 0;

// Timing control
unsigned long previousUpdate = 0;
const uint16_t UPDATE_INTERVAL = 500;  // 0.5 seconds

void configureSensor() {
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
}

float mapPercentage(float lux) {
  lux = constrain(lux, MIN_LUX, MAX_LUX);
  return (lux - MIN_LUX) * 100.0 / (MAX_LUX - MIN_LUX);
}

void setup() {
  Serial.begin(115200);
  Wire.setSDA(I2C_SDA); // STM32 I2C pin assignment
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Light Level:");
  
  // Initialize sensor
  if (!tsl.begin()) {
    lcd.clear();
    lcd.print("Sensor Error!");
    while(1);
  }
  configureSensor();
  
  // Initialize readings
  for(uint8_t i=0; i<SAMPLE_SIZE; i++) {
    sensors_event_t event;
    tsl.getEvent(&event);
    luxReadings[i] = event.light;
    luxTotal += luxReadings[i];
    delay(100);
  }
}

void loop() {
  sensors_event_t event;
  tsl.getEvent(&event);
  
  // Update moving average
  luxTotal -= luxReadings[currentReading];
  luxReadings[currentReading] = event.light;
  luxTotal += luxReadings[currentReading];
  currentReading = (currentReading + 1) % SAMPLE_SIZE;

  if(millis() - previousUpdate >= UPDATE_INTERVAL) {
    previousUpdate = millis();
    
    float avgLux = luxTotal / SAMPLE_SIZE;
    float percentage = mapPercentage(avgLux);
    
    // LCD Update
    lcd.setCursor(0, 1);
    lcd.print("           ");
    lcd.setCursor(0, 1);
    
    if(avgLux < 1000) {
      lcd.print(avgLux, 0);
      lcd.print(" lx ");
    } else {
      lcd.print(avgLux/1000, 1);
      lcd.print("klx ");
    }
    lcd.print("(");
    lcd.print(percentage, 0);
    lcd.print("%)");

    // Serial Output
    Serial.print("Light Level: ");
    Serial.print(avgLux, 1);
    Serial.print(" lx (");
    Serial.print(percentage, 0);
    Serial.println("%)");
  }
}
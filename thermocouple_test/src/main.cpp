#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <math.h>

#define TC_SCK 7
#define TC_CS 9
#define TC_SO 8

#define OLED_SDA 5
#define OLED_SCL 6
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    OLED_RESET
);

MAX6675 thermocouple(TC_SCK, TC_CS, TC_SO);

void showReading(double temperature)
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("MAX6675 TEST");

    display.setTextSize(2);
    display.setCursor(0, 20);

    if (isnan(temperature))
    {
        display.println("ERROR");
    }
    else
    {
        display.print(temperature, 1);
        display.println(" C");
    }

    display.setTextSize(1);
    display.setCursor(0, 52);

    if (isnan(temperature))
    {
        display.println("Check sensor wiring");
    }
    else
    {
        display.println("Sensor OK");
    }

    display.display();
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Wire.begin(OLED_SDA, OLED_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
    {
        Serial.println("OLED initialization failed");
        while (true)
        {
            delay(1000);
        }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("MAX6675 TEST");
    display.println();
    display.println("Waiting for sensor...");
    display.display();

    Serial.println("MAX6675 thermocouple test");
    Serial.println("SCK=7, CS=9, SO=8");
    delay(500);
}

void loop()
{
    double temperature = thermocouple.readCelsius();

    Serial.print("Temperature: ");

    if (isnan(temperature))
    {
        Serial.println("ERROR / NAN - check thermocouple wiring");
    }
    else
    {
        Serial.print(temperature, 2);
        Serial.println(" C");
    }

    showReading(temperature);
    delay(1000);
}

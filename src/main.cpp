#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <math.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define TC_SCK       7
#define TC_CS        9
#define TC_SO        8

#define OLED_SDA     5
#define OLED_SCL     6

#define ENCODER_CLK  3
#define ENCODER_DT   2
#define ENCODER_SW   1

#define HEATER_PIN   4

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    OLED_RESET
);

// ============================================================
// MAX6675
// ============================================================

MAX6675 thermocouple(
    TC_SCK,
    TC_CS,
    TC_SO
);

// ============================================================
// TEMPERATURE LIMITS
// ============================================================

const int MIN_TEMP = 0;
const int MAX_TEMP = 200;

const double MAX_SAFE_TEMP = 205.0;

// ============================================================
// PWM
// ============================================================

const int PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;

// ============================================================
// SYSTEM SAFETY FLAGS
// ============================================================

bool systemReady = false;
bool oledReady = false;
bool temperatureValid = false;

bool controlActive = false;
bool targetConfirmed = false;

// ============================================================
// ENCODER
// ============================================================

volatile int encoderValue = 0;
volatile bool encoderMoved = false;

int lastCLKState = HIGH;

bool lastButtonState = HIGH;

unsigned long lastButtonTime = 0;

const unsigned long BUTTON_DEBOUNCE_TIME = 200;

// ============================================================
// TEMPERATURE
// ============================================================

double currentTemperature = NAN;

double celsiusToFahrenheit(double celsius)
{
    return (celsius * 9.0 / 5.0) + 32.0;
}

// ============================================================
// TARGET
// ============================================================

int targetValue = 0;
double setpoint = 0.0;

// ============================================================
// PID
// ============================================================

double Kp = 5.0;
double Ki = 0.2;
double Kd = 20.0;

double integral = 0.0;
double previousError = 0.0;
double pidOutput = 0.0;

unsigned long previousPIDCalculation = 0;

// ============================================================
// TIMING
// ============================================================

unsigned long lastPIDRun = 0;
const unsigned long PID_INTERVAL = 500;

unsigned long lastTemperatureRead = 0;
const unsigned long TEMPERATURE_INTERVAL = 250;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 100;

unsigned long lastOLEDAttempt = 0;
const unsigned long OLED_RETRY_INTERVAL = 1000;

// ============================================================
// FORCE HEATER OFF
// ============================================================

void heaterOff()
{
    // Set PWM to zero first
    ledcWrite(
        PWM_CHANNEL,
        0
    );

    // Also force GPIO LOW
    digitalWrite(
        HEATER_PIN,
        LOW
    );

    pidOutput = 0.0;
}

// ============================================================
// ENCODER INTERRUPT
// ============================================================

void IRAM_ATTR encoderISR()
{
    int currentDT = digitalRead(ENCODER_DT);

    if (currentDT == HIGH)
    {
        encoderValue++;

        if (encoderValue > MAX_TEMP)
        {
            encoderValue = MAX_TEMP;
        }
    }
    else
    {
        encoderValue--;

        if (encoderValue < MIN_TEMP)
        {
            encoderValue = MIN_TEMP;
        }
    }

    encoderMoved = true;
}

// ============================================================
// READ TEMPERATURE
// ============================================================

void readTemperature()
{
    double temperature =
        thermocouple.readCelsius();

    // MAX6675 disconnected / invalid
    if (isnan(temperature))
    {
        currentTemperature = NAN;
        temperatureValid = false;

        heaterOff();

        return;
    }

    // MAX6675 fault
    if (temperature < -20.0 || temperature > 1000.0)
    {
        currentTemperature = NAN;
        temperatureValid = false;

        heaterOff();

        return;
    }

    currentTemperature = temperature;
    temperatureValid = true;
}

// ============================================================
// RESET PID
// ============================================================

void resetPID()
{
    integral = 0.0;
    previousError = 0.0;
    pidOutput = 0.0;

    previousPIDCalculation =
        millis();

    heaterOff();
}

// ============================================================
// CALCULATE PID
// ============================================================

double calculatePID(
    double target,
    double temperature
)
{
    unsigned long currentTime =
        millis();

    double dt =
        (currentTime - previousPIDCalculation)
        / 1000.0;

    if (dt <= 0.0)
    {
        dt = 0.001;
    }

    previousPIDCalculation =
        currentTime;

    // --------------------------------------------------------
    // ERROR
    // --------------------------------------------------------

    double error =
        target - temperature;

    // --------------------------------------------------------
    // PROPORTIONAL
    // --------------------------------------------------------

    double P =
        Kp * error;

    // --------------------------------------------------------
    // INTEGRAL
    // --------------------------------------------------------

    integral +=
        error * dt;

    // Anti-windup
    if (Ki > 0.0)
    {
        double integralLimit =
            100.0 / Ki;

        if (integral > integralLimit)
        {
            integral =
                integralLimit;
        }

        if (integral < -integralLimit)
        {
            integral =
                -integralLimit;
        }
    }

    double I =
        Ki * integral;

    // --------------------------------------------------------
    // DERIVATIVE
    // --------------------------------------------------------

    double derivative =
        (error - previousError)
        / dt;

    double D =
        Kd * derivative;

    previousError =
        error;

    // --------------------------------------------------------
    // PID OUTPUT
    // --------------------------------------------------------

    double output =
        P + I + D;

    // Limit 0-100%
    if (output > 100.0)
    {
        output = 100.0;
    }

    if (output < 0.0)
    {
        output = 0.0;
    }

    return output;
}

// ============================================================
// UPDATE HEATER
// ============================================================

void updateHeater()
{
    // ========================================================
    // SAFETY CHECK 1
    // System must be fully initialized
    // ========================================================

    if (!systemReady)
    {
        heaterOff();
        return;
    }

    // ========================================================
    // SAFETY CHECK 2
    // OLED must be working
    // ========================================================

    if (!oledReady)
    {
        heaterOff();
        return;
    }

    // ========================================================
    // SAFETY CHECK 3
    // Temperature must be valid
    // ========================================================

    if (!temperatureValid ||
        isnan(currentTemperature))
    {
        heaterOff();
        return;
    }

    // ========================================================
    // SAFETY CHECK 4
    // Controller must be activated by button
    // ========================================================

    if (!controlActive)
    {
        heaterOff();
        return;
    }

    // ========================================================
    // SAFETY CHECK 5
    // Target must be valid
    // ========================================================

    if (setpoint <= 0.0)
    {
        heaterOff();
        return;
    }

    // ========================================================
    // SAFETY CHECK 6
    // Maximum temperature
    // ========================================================

    if (currentTemperature >= MAX_SAFE_TEMP)
    {
        heaterOff();

        controlActive = false;

        return;
    }

    // ========================================================
    // PID
    // ========================================================

    pidOutput =
        calculatePID(
            setpoint,
            currentTemperature
        );

    // ========================================================
    // CONVERT PID % TO PWM
    // ========================================================

    int pwmValue =
        (int)(
            (pidOutput / 100.0)
            * PWM_MAX
        );

    pwmValue =
        constrain(
            pwmValue,
            0,
            PWM_MAX
        );

    // ========================================================
    // HEATER OUTPUT
    // ========================================================

    ledcWrite(
        PWM_CHANNEL,
        pwmValue
    );
}

// ============================================================
// HANDLE BUTTON
// ============================================================

void handleButton()
{
    bool currentButtonState =
        digitalRead(ENCODER_SW);

    // Detect falling edge
    if (
        lastButtonState == HIGH &&
        currentButtonState == LOW
    )
    {
        unsigned long currentTime =
            millis();

        if (
            currentTime - lastButtonTime
            >= BUTTON_DEBOUNCE_TIME
        )
        {
            lastButtonTime =
                currentTime;

            int newTarget;

            noInterrupts();

            newTarget =
                encoderValue;

            interrupts();

            // ------------------------------------------------
            // Validate target
            // ------------------------------------------------

            if (newTarget < MIN_TEMP)
            {
                newTarget =
                    MIN_TEMP;
            }

            if (newTarget > MAX_TEMP)
            {
                newTarget =
                    MAX_TEMP;
            }

            targetValue =
                newTarget;

            setpoint =
                (double)targetValue;

            // ------------------------------------------------
            // If target is 0, keep heater OFF
            // ------------------------------------------------

            if (targetValue <= 0)
            {
                controlActive =
                    false;

                targetConfirmed =
                    true;

                resetPID();

            }
            else
            {
                // ------------------------------------------------
                // Only activate if temperature is valid
                // ------------------------------------------------

                if (temperatureValid)
                {
                    targetConfirmed =
                        true;

                    controlActive =
                        true;

                    resetPID();

                }
                else
                {
                    controlActive =
                        false;

                    targetConfirmed =
                        false;

                    heaterOff();

                }
            }
        }
    }

    lastButtonState =
        currentButtonState;
}

// ============================================================
// HANDLE ENCODER
// ============================================================

void handleEncoderMovement()
{
    bool moved;

    noInterrupts();

    moved =
        encoderMoved;

    encoderMoved =
        false;

    interrupts();

    if (moved)
    {
        targetConfirmed =
            false;

        int value;

        noInterrupts();

        value =
            encoderValue;

        interrupts();

    }
}

// ============================================================
// INITIALIZE OLED
// ============================================================

bool initializeOLED()
{
    if (
        display.begin(
            SSD1306_SWITCHCAPVCC,
            0x3C
        )
    )
    {
        oledReady =
            true;

        display.clearDisplay();

        display.setTextColor(
            SSD1306_WHITE
        );

        display.setTextSize(1);

        display.setCursor(
            20,
            5
        );

        display.println(
            "PTC CONTROLLER"
        );

        display.setCursor(
            25,
            22
        );

        display.println(
            "ESP32-S3"
        );

        display.setCursor(
            20,
            39
        );

        display.println(
            "Initializing..."
        );

        display.display();

        delay(1000);

        return true;
    }

    oledReady =
        false;

    // IMPORTANT:
    // OLED failure always means heater OFF.
    heaterOff();

    return false;
}

// ============================================================
// OLED RETRY
// ============================================================

void checkOLED()
{
    if (oledReady)
    {
        return;
    }

    unsigned long currentTime =
        millis();

    if (
        currentTime - lastOLEDAttempt
        >= OLED_RETRY_INTERVAL
    )
    {
        lastOLEDAttempt =
            currentTime;

        initializeOLED();
    }
}

// ============================================================
// UPDATE DISPLAY
// ============================================================

void updateDisplay()
{
    // If OLED isn't ready, don't try to use it
    if (!oledReady)
    {
        return;
    }

    int displayTarget;

    noInterrupts();

    displayTarget =
        (int)round(
            celsiusToFahrenheit(
                (double)encoderValue
            )
        );

    interrupts();

    display.clearDisplay();

    // ========================================================
    // DIVIDER
    // ========================================================

    display.drawLine(
        63,
        0,
        63,
        63,
        SSD1306_WHITE
    );

    // ========================================================
    // LEFT SIDE - CURRENT TEMPERATURE
    // ========================================================

    display.setTextColor(
        SSD1306_WHITE
    );

    display.setTextSize(1);

    display.setCursor(
        5,
        2
    );

    display.println(
        "TEMP(F)"
    );

    display.setTextSize(2);

    display.setCursor(
        3,
        20
    );

    if (
        isnan(currentTemperature)
    )
    {
        display.println(
            "--.-"
        );
    }
    else
    {
        display.print(
            celsiusToFahrenheit(
                currentTemperature
            ),
            1
        );
    }

    display.setTextSize(1);

    display.setCursor(
        3,
        40
    );

    display.print(
        "SET "
    );

    display.print(
        (int)round(
            celsiusToFahrenheit(
                (double)targetValue
            )
        )
    );

    display.print(
        "F"
    );

    // ========================================================
    // RIGHT SIDE - TARGET
    // ========================================================

    display.setTextSize(1);

    display.setCursor(
        70,
        2
    );

    if (targetConfirmed)
    {
        display.println(
            "SET POINT"
        );
    }
    else
    {
        display.println(
            "EDITTING"
        );
    }

    // ========================================================
    // TARGET HIGHLIGHT
    // ========================================================

    if (targetConfirmed)
    {
        display.fillRect(
            66,
            18,
            60,
            25,
            SSD1306_WHITE
        );

        display.setTextColor(
            SSD1306_BLACK
        );
    }
    else
    {
        display.setTextColor(
            SSD1306_WHITE
        );
    }

    display.setTextSize(2);

    display.setCursor(
        70,
        22
    );

    display.print(
        displayTarget
    );

    // STATUS
    // ========================================================

    display.setTextSize(1);

    if (!targetConfirmed)
    {
        display.setCursor(
            70,
            51
        );

        display.print(
            "SET"
        );
    }
    else if (controlActive)
    {
        display.setCursor(
            3,
            51
        );

        display.print(
            "ACTIVE"
        );

        display.setCursor(
            70,
            51
        );

        display.print(
            (int)pidOutput
        );

        display.print(
            "%"
        );
    }
    else
    {
        display.setCursor(
            3,
            51
        );

        display.print(
            "OFF"
        );

        display.setCursor(
            70,
            51
        );

        if (targetConfirmed)
        {
            display.print(
                (int)pidOutput
            );

            display.print(
                "%"
            );
        }
        else
        {
            display.print(
                "SET"
            );
        }
    }

    display.display();
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    // ========================================================
    // VERY FIRST ACTION:
    // MAKE SURE HEATER PIN IS LOW
    // ========================================================

    pinMode(
        HEATER_PIN,
        OUTPUT
    );

    digitalWrite(
        HEATER_PIN,
        LOW
    );

    // ========================================================
    // FORCE ALL SAFETY STATES
    // ========================================================

    systemReady =
        false;

    oledReady =
        false;

    temperatureValid =
        false;

    controlActive =
        false;

    targetConfirmed =
        false;

    heaterOff();

    // ========================================================
    // ENCODER
    // ========================================================

    pinMode(
        ENCODER_CLK,
        INPUT_PULLUP
    );

    pinMode(
        ENCODER_DT,
        INPUT_PULLUP
    );

    pinMode(
        ENCODER_SW,
        INPUT_PULLUP
    );

    lastCLKState =
        digitalRead(
            ENCODER_CLK
        );

    attachInterrupt(
        digitalPinToInterrupt(
            ENCODER_CLK
        ),
        encoderISR,
        FALLING
    );

    // ========================================================
    // PWM
    // ========================================================

    ledcSetup(
        PWM_CHANNEL,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        HEATER_PIN,
        PWM_CHANNEL
    );

    // CRITICAL:
    // PWM starts at 0
    ledcWrite(
        PWM_CHANNEL,
        0
    );

    digitalWrite(
        HEATER_PIN,
        LOW
    );

    // ========================================================
    // I2C
    // ========================================================

    Wire.begin(
        OLED_SDA,
        OLED_SCL
    );

    delay(100);

    // ========================================================
    // OLED
    // ========================================================

    initializeOLED();

    // ========================================================
    // MAX6675
    // ========================================================

    delay(500);

    readTemperature();

    // ========================================================
    // PID
    // ========================================================

    resetPID();

    // ========================================================
    // SYSTEM READY
    // ========================================================

    // IMPORTANT:
    // Heater will only be allowed after this point.
    //
    // OLED MUST be working.
    // Temperature MUST be valid.
    //
    // User still has to press the encoder button.
    // ========================================================

    if (
        oledReady &&
        temperatureValid
    )
    {
        systemReady =
            true;

    }
    else
    {
        systemReady =
            false;

        heaterOff();

    }

    // ========================================================
    // TIMERS
    // ========================================================

    unsigned long now =
        millis();

    lastPIDRun =
        now;

    lastTemperatureRead =
        now;

    lastDisplayUpdate =
        now;

    lastOLEDAttempt =
        now;
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    unsigned long currentTime =
        millis();

    // ========================================================
    // OLED CHECK
    // ========================================================

    checkOLED();

    // ========================================================
    // ENCODER
    // ========================================================

    handleEncoderMovement();

    handleButton();

    // ========================================================
    // TEMPERATURE
    // ========================================================

    if (
        currentTime -
        lastTemperatureRead
        >= TEMPERATURE_INTERVAL
    )
    {
        lastTemperatureRead =
            currentTime;

        readTemperature();
    }

    // ========================================================
    // PID / HEATER
    // ========================================================

    if (
        currentTime -
        lastPIDRun
        >= PID_INTERVAL
    )
    {
        lastPIDRun =
            currentTime;

        updateHeater();
    }

    // ========================================================
    // OLED
    // ========================================================

    if (
        currentTime -
        lastDisplayUpdate
        >= DISPLAY_INTERVAL
    )
    {
        lastDisplayUpdate =
            currentTime;

        updateDisplay();
    }

    // ========================================================
    // LOOP DELAY
    // ========================================================

    delay(5);
}
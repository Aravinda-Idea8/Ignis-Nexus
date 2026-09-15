#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <math.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// ---------------- MAX6675 ----------------
#define TC_SCK 7
#define TC_CS  9
#define TC_SO  8

// ---------------- OLED --------------------
#define OLED_SDA 5
#define OLED_SCL 6

// ---------------- ROTARY ENCODER ----------
#define ENCODER_CLK 4
#define ENCODER_DT  2
#define ENCODER_SW  3

// ---------------- HEATER MOSFET -----------
#define HEATER_PIN 1


// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

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

// Independent safety cutoff.
// Heater is switched OFF if this temperature is reached.
const double MAX_SAFE_TEMP = 205.0;


// ============================================================
// PWM CONFIGURATION
// ============================================================

const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;


// ============================================================
// ENCODER
// ============================================================

// Value currently being edited by the user.
volatile int encoderValue = 0;

// Previous CLK state.
volatile int lastCLKState = HIGH;

// This flag becomes true whenever the encoder is rotated.
volatile bool encoderMoved = false;


// ============================================================
// TARGET / SETPOINT
// ============================================================

// Temperature currently being edited.
int targetValue = 0;

// Temperature that has actually been confirmed
// by pressing the encoder button.
double setpoint = 0.0;

// True after a target has been confirmed.
bool controlActive = false;

// True when the currently displayed encoder value
// is confirmed/selected.
bool targetConfirmed = false;


// ============================================================
// BUTTON
// ============================================================

bool lastButtonState = HIGH;

unsigned long lastButtonTime = 0;

const unsigned long BUTTON_DEBOUNCE_TIME = 200;


// ============================================================
// TEMPERATURE
// ============================================================

double currentTemperature = NAN;


// ============================================================
// PID PARAMETERS
// ============================================================
//
// STARTING VALUES.
//
// These values should be experimentally tuned for
// your actual PTC heater, thermal mass, insulation,
// thermocouple position and cooling conditions.
//

double Kp = 5.0;
double Ki = 0.2;
double Kd = 20.0;


// ============================================================
// PID INTERNAL VARIABLES
// ============================================================

// Integral accumulator
double integral = 0.0;

// Previous error
double previousError = 0.0;

// PID output in percentage
//
// 0   = heater OFF
// 100 = maximum commanded heating
//
double pidOutput = 0.0;


// ============================================================
// PID TIMING
// ============================================================

unsigned long previousPIDCalculation = 0;

unsigned long lastPIDRun = 0;

const unsigned long PID_INTERVAL = 500;


// ============================================================
// TEMPERATURE READING TIMING
// ============================================================

unsigned long lastTemperatureRead = 0;

const unsigned long TEMPERATURE_INTERVAL = 250;


// ============================================================
// OLED UPDATE TIMING
// ============================================================

unsigned long lastDisplayUpdate = 0;

const unsigned long DISPLAY_INTERVAL = 100;


// ============================================================
// SERIAL DATA TIMING
// ============================================================

unsigned long lastSerialOutput = 0;

const unsigned long SERIAL_INTERVAL = 500;


// ============================================================
// ENCODER INTERRUPT
// ============================================================

void IRAM_ATTR encoderISR()
{
  int currentDT = digitalRead(ENCODER_DT);

  // ----------------------------------------------------------
  // Direction detection
  // ----------------------------------------------------------
  //
  // If the direction is reversed on your encoder,
  // swap ++ and --.
  //

  if (currentDT == HIGH)
  {
    // Clockwise
    encoderValue++;

    if (encoderValue > MAX_TEMP)
    {
      encoderValue = MAX_TEMP;
    }
  }
  else
  {
    // Anti-clockwise
    encoderValue--;

    if (encoderValue < MIN_TEMP)
    {
      encoderValue = MIN_TEMP;
    }
  }

  // ----------------------------------------------------------
  // Tell the main program that the encoder was rotated.
  //
  // We do NOT directly modify targetConfirmed here because
  // this function is running inside an interrupt.
  // ----------------------------------------------------------

  encoderMoved = true;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ==========================================================
  // SERIAL
  // ==========================================================

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" XIAO ESP32-S3 PTC PID CONTROLLER");
  Serial.println("========================================");


  // ==========================================================
  // ENCODER
  // ==========================================================

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
    digitalRead(ENCODER_CLK);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_CLK),
    encoderISR,
    FALLING
  );


  // ==========================================================
  // HEATER PWM
  // ==========================================================

  pinMode(
    HEATER_PIN,
    OUTPUT
  );

  // Make absolutely sure heater starts OFF.
  digitalWrite(
    HEATER_PIN,
    LOW
  );

  // ESP32 Arduino Core 3.x
  ledcAttach(
    HEATER_PIN,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );

  // Heater OFF
  ledcWrite(
    HEATER_PIN,
    0
  );


  // ==========================================================
  // OLED
  // ==========================================================

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        0x3C
      ))
  {
    Serial.println(
      "ERROR: OLED NOT FOUND"
    );

    // Heater remains OFF
    ledcWrite(
      HEATER_PIN,
      0
    );

    while (true)
    {
      delay(1000);
    }
  }


  // ==========================================================
  // STARTUP SCREEN
  // ==========================================================

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    20,
    20
  );

  display.println(
    "PTC CONTROLLER"
  );

  display.setCursor(
    20,
    35
  );

  display.println(
    "Starting..."
  );

  display.display();

  delay(1500);


  // ==========================================================
  // INITIAL TIMERS
  // ==========================================================

  previousPIDCalculation =
    millis();

  lastPIDRun =
    millis();

  lastTemperatureRead =
    millis();

  lastDisplayUpdate =
    millis();

  lastSerialOutput =
    millis();


  // ==========================================================
  // INITIAL TEMPERATURE READING
  // ==========================================================

  currentTemperature =
    thermocouple.readCelsius();


  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();
  Serial.println("SYSTEM READY");

  Serial.println(
    "Rotate encoder to select temperature."
  );

  Serial.println(
    "Press encoder button to confirm target."
  );

  Serial.println();
}


// ============================================================
// READ TEMPERATURE
// ============================================================

void readTemperature()
{
  double newTemperature =
    thermocouple.readCelsius();

  if (isnan(newTemperature))
  {
    currentTemperature = NAN;

    return;
  }

  currentTemperature =
    newTemperature;
}


// ============================================================
// RESET PID
// ============================================================

void resetPID()
{
  // Reset integral
  integral = 0.0;

  // Reset previous error
  previousError = 0.0;

  // Reset PID output
  pidOutput = 0.0;

  // Reset timing
  previousPIDCalculation =
    millis();

  // Heater OFF
  ledcWrite(
    HEATER_PIN,
    0
  );
}


// ============================================================
// PID CALCULATION
// ============================================================
//
// Returns:
//
// 0   -> 0% heating
// 100 -> 100% heating
//
// ============================================================

double calculatePID(
  double target,
  double temperature
)
{
  unsigned long currentTime =
    millis();


  // ----------------------------------------------------------
  // Calculate elapsed time
  // ----------------------------------------------------------

  double dt =
    (
      currentTime -
      previousPIDCalculation
    ) / 1000.0;

  previousPIDCalculation =
    currentTime;


  if (dt <= 0.0)
  {
    return pidOutput;
  }


  // ----------------------------------------------------------
  // ERROR
  // ----------------------------------------------------------

  double error =
    target -
    temperature;


  // ----------------------------------------------------------
  // PROPORTIONAL
  // ----------------------------------------------------------

  double P =
    Kp * error;


  // ----------------------------------------------------------
  // INTEGRAL
  // ----------------------------------------------------------

  integral +=
    error * dt;


  // ----------------------------------------------------------
  // INTEGRAL ANTI-WINDUP
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // DERIVATIVE
  // ----------------------------------------------------------

  double derivative =
    (
      error -
      previousError
    ) / dt;


  double D =
    Kd * derivative;


  // Store error
  previousError =
    error;


  // ----------------------------------------------------------
  // TOTAL PID
  // ----------------------------------------------------------

  double output =
    P +
    I +
    D;


  // ----------------------------------------------------------
  // LIMIT OUTPUT
  // ----------------------------------------------------------

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
  // ----------------------------------------------------------
  // No target confirmed yet
  // ----------------------------------------------------------

  if (!controlActive)
  {
    pidOutput = 0.0;

    ledcWrite(
      HEATER_PIN,
      0
    );

    return;
  }


  // ----------------------------------------------------------
  // THERMOCOUPLE ERROR
  // ----------------------------------------------------------

  if (isnan(currentTemperature))
  {
    Serial.println(
      "THERMOCOUPLE ERROR - HEATER OFF"
    );

    pidOutput = 0.0;

    ledcWrite(
      HEATER_PIN,
      0
    );

    return;
  }


  // ----------------------------------------------------------
  // OVER TEMPERATURE
  // ----------------------------------------------------------

  if (
    currentTemperature >=
    MAX_SAFE_TEMP
  )
  {
    Serial.println(
      "OVER TEMPERATURE - HEATER OFF"
    );

    pidOutput = 0.0;

    ledcWrite(
      HEATER_PIN,
      0
    );

    return;
  }


  // ----------------------------------------------------------
  // PID
  // ----------------------------------------------------------

  pidOutput =
    calculatePID(
      setpoint,
      currentTemperature
    );


  // ----------------------------------------------------------
  // Convert 0-100% to 0-255
  // ----------------------------------------------------------

  int pwmValue =
    (int)(
      (
        pidOutput /
        100.0
      ) * PWM_MAX
    );


  // ----------------------------------------------------------
  // Safety limit
  // ----------------------------------------------------------

  pwmValue =
    constrain(
      pwmValue,
      0,
      PWM_MAX
    );


  // ----------------------------------------------------------
  // OUTPUT PWM
  // ----------------------------------------------------------

  ledcWrite(
    HEATER_PIN,
    pwmValue
  );
}


// ============================================================
// BUTTON HANDLING
// ============================================================

void handleButton()
{
  bool buttonState =
    digitalRead(
      ENCODER_SW
    );


  // ----------------------------------------------------------
  // Detect button press
  // ----------------------------------------------------------

  if (
    buttonState == LOW &&
    lastButtonState == HIGH
  )
  {
    unsigned long currentTime =
      millis();


    // --------------------------------------------------------
    // Debounce
    // --------------------------------------------------------

    if (
      currentTime -
      lastButtonTime
      >=
      BUTTON_DEBOUNCE_TIME
    )
    {
      lastButtonTime =
        currentTime;


      // ------------------------------------------------------
      // Safely read encoder value
      // ------------------------------------------------------

      noInterrupts();

      int newTarget =
        encoderValue;

      interrupts();


      // ------------------------------------------------------
      // Confirm target
      // ------------------------------------------------------

      targetValue =
        newTarget;

      setpoint =
        targetValue;


      // ------------------------------------------------------
      // Target is now confirmed
      // ------------------------------------------------------

      targetConfirmed =
        true;

      controlActive =
        true;


      // ------------------------------------------------------
      // Reset PID
      // ------------------------------------------------------

      resetPID();


      // ------------------------------------------------------
      // Serial message
      // ------------------------------------------------------

      Serial.println();
      Serial.println(
        "================================"
      );

      Serial.print(
        "TARGET CONFIRMED: "
      );

      Serial.print(
        targetValue
      );

      Serial.println(
        " C"
      );

      Serial.println(
        "PID CONTROL ACTIVE"
      );

      Serial.println(
        "================================"
      );

      Serial.println();
    }
  }


  lastButtonState =
    buttonState;
}


// ============================================================
// HANDLE ENCODER MOVEMENT
// ============================================================
//
// Whenever the encoder is rotated after a target has been
// confirmed, the highlight is removed.
//
// IMPORTANT:
// The PID setpoint is NOT changed here.
//
// The PID continues controlling the previously confirmed
// temperature until the user presses the button again.
//

void handleEncoderMovement()
{
  bool movementDetected = false;


  // ----------------------------------------------------------
  // Safely check encoder movement flag
  // ----------------------------------------------------------

  noInterrupts();

  if (encoderMoved)
  {
    encoderMoved = false;

    movementDetected = true;
  }

  interrupts();


  // ----------------------------------------------------------
  // Encoder was rotated
  // ----------------------------------------------------------

  if (movementDetected)
  {
    // Remove confirmation/highlight
    targetConfirmed = false;


    // --------------------------------------------------------
    // Show what is happening on Serial Monitor
    // --------------------------------------------------------

    int currentTarget;

    noInterrupts();

    currentTarget =
      encoderValue;

    interrupts();


    Serial.print(
      "EDITING TARGET: "
    );

    Serial.print(
      currentTarget
    );

    Serial.println(
      " C"
    );
  }
}


// ============================================================
// OLED DISPLAY
// ============================================================

void updateDisplay()
{
  // ----------------------------------------------------------
  // Safely copy encoder value
  // ----------------------------------------------------------

  noInterrupts();

  int displayTarget =
    encoderValue;

  interrupts();


  // ----------------------------------------------------------
  // Clear OLED
  // ----------------------------------------------------------

  display.clearDisplay();


  // ==========================================================
  // VERTICAL DIVIDER
  // ==========================================================

  display.drawLine(
    63,
    0,
    63,
    63,
    SSD1306_WHITE
  );


  // ==========================================================
  // LEFT COLUMN
  // ACTUAL TEMPERATURE
  // ==========================================================

  display.setTextSize(1);

  display.setCursor(
    6,
    5
  );

  display.println(
    "TEMP"
  );


  // ----------------------------------------------------------
  // Actual temperature
  // ----------------------------------------------------------

  display.setTextSize(2);

  display.setCursor(
    2,
    25
  );


  if (isnan(currentTemperature))
  {
    display.println(
      "ERROR"
    );
  }
  else
  {
    display.print(
      currentTemperature,
      1
    );

    display.print(
      (char)247
    );

    display.print(
      "C"
    );
  }


  // ==========================================================
  // RIGHT COLUMN
  // TARGET
  // ==========================================================

  display.setTextSize(1);

  display.setCursor(
    80,
    5
  );


  if (targetConfirmed)
  {
    display.println(
      "SET"
    );
  }
  else
  {
    display.println(
      "TARGET"
    );
  }


  // ==========================================================
  // TARGET VALUE
  // ==========================================================

  display.setTextSize(2);


  // ----------------------------------------------------------
  // Calculate text width
  // ----------------------------------------------------------

  int textWidth;


  if (displayTarget < 10)
  {
    textWidth = 12;
  }
  else if (displayTarget < 100)
  {
    textWidth = 24;
  }
  else
  {
    textWidth = 36;
  }


  // ----------------------------------------------------------
  // Center target in right column
  // ----------------------------------------------------------

  int x =
    95 -
    (textWidth / 2);


  // ==========================================================
  // CONFIRMED TARGET
  // ==========================================================

  if (targetConfirmed)
  {
    // --------------------------------------------------------
    // Highlight rectangle
    // --------------------------------------------------------

    display.fillRect(
      x - 3,
      24,
      textWidth + 6,
      18,
      SSD1306_WHITE
    );


    // --------------------------------------------------------
    // Black text on white background
    // --------------------------------------------------------

    display.setTextColor(
      SSD1306_BLACK
    );


    display.setCursor(
      x,
      25
    );


    display.print(
      displayTarget
    );


    // --------------------------------------------------------
    // Return to normal white text
    // --------------------------------------------------------

    display.setTextColor(
      SSD1306_WHITE
    );
  }


  // ==========================================================
  // UNCONFIRMED / EDITING TARGET
  // ==========================================================

  else
  {
    // --------------------------------------------------------
    // Normal white text
    // --------------------------------------------------------

    display.setTextColor(
      SSD1306_WHITE
    );


    display.setCursor(
      x,
      25
    );


    display.print(
      displayTarget
    );
  }


  // ==========================================================
  // STATUS
  // ==========================================================

  display.setTextSize(1);


  if (targetConfirmed)
  {
    display.setCursor(
      68,
      51
    );

    display.print(
      "ACTIVE"
    );
  }
  else
  {
    display.setCursor(
      68,
      51
    );

    if (controlActive)
    {
      display.print(
        "EDITING"
      );
    }
    else
    {
      display.print(
        "PRESS SET"
      );
    }
  }


  // ----------------------------------------------------------
  // Update OLED
  // ----------------------------------------------------------

  display.display();
}


// ============================================================
// SERIAL PID MONITOR
// ============================================================

void sendSerialData()
{
  if (
    millis() -
    lastSerialOutput
    <
    SERIAL_INTERVAL
  )
  {
    return;
  }


  lastSerialOutput =
    millis();


  /*
    CSV FORMAT:

    time,
    temperature,
    setpoint,
    error,
    pidOutput,
    Kp,
    Ki,
    Kd
  */


  Serial.print(
    millis()
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Temperature
  // ----------------------------------------------------------

  if (isnan(currentTemperature))
  {
    Serial.print(
      "NaN"
    );
  }
  else
  {
    Serial.print(
      currentTemperature,
      2
    );
  }


  Serial.print(",");


  // ----------------------------------------------------------
  // Setpoint
  // ----------------------------------------------------------

  Serial.print(
    setpoint,
    2
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Error
  // ----------------------------------------------------------

  if (isnan(currentTemperature))
  {
    Serial.print(
      "NaN"
    );
  }
  else
  {
    Serial.print(
      setpoint -
      currentTemperature,
      2
    );
  }


  Serial.print(",");


  // ----------------------------------------------------------
  // PID output
  // ----------------------------------------------------------

  Serial.print(
    pidOutput,
    2
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Kp
  // ----------------------------------------------------------

  Serial.print(
    Kp,
    3
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Ki
  // ----------------------------------------------------------

  Serial.print(
    Ki,
    3
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Kd
  // ----------------------------------------------------------

  Serial.println(
    Kd,
    3
  );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  unsigned long currentTime =
    millis();


  // ==========================================================
  // ENCODER MOVEMENT
  // ==========================================================

  handleEncoderMovement();


  // ==========================================================
  // BUTTON
  // ==========================================================

  handleButton();


  // ==========================================================
  // TEMPERATURE READING
  // ==========================================================

  if (
    currentTime -
    lastTemperatureRead
    >=
    TEMPERATURE_INTERVAL
  )
  {
    lastTemperatureRead =
      currentTime;

    readTemperature();
  }


  // ==========================================================
  // PID
  // ==========================================================

  if (
    currentTime -
    lastPIDRun
    >=
    PID_INTERVAL
  )
  {
    lastPIDRun =
      currentTime;

    updateHeater();
  }


  // ==========================================================
  // OLED
  // ==========================================================

  if (
    currentTime -
    lastDisplayUpdate
    >=
    DISPLAY_INTERVAL
  )
  {
    lastDisplayUpdate =
      currentTime;

    updateDisplay();
  }


  // ==========================================================
  // SERIAL MONITOR / PC
  // ==========================================================

  sendSerialData();


  // ==========================================================
  // SMALL DELAY
  // ==========================================================

  delay(5);
}

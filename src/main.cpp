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
#define ENCODER_CLK 3
#define ENCODER_DT  2
#define ENCODER_SW  1

// ---------------- HEATER MOSFET -----------
#define HEATER_PIN 4


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
const int PWM_CHANNEL = 0;


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
// by pressing the encoder button. This is the user's
// *final* target.
double setpoint = 0.0;

// True after a target has been confirmed.
bool controlActive = false;

// True when the currently displayed encoder value
// is confirmed/selected.
bool targetConfirmed = false;


// ============================================================
// SETPOINT RAMP (SOFT-START)
// ============================================================
//
// The PID loop does not chase `setpoint` directly. Instead it
// chases `activeSetpoint`, which ramps smoothly from the
// temperature you were at when the target was confirmed, up
// (or down) to `setpoint`, at a limited rate.
//
// This is the single biggest lever against overshoot on a PTC
// heater: the heater can only add heat, it cannot actively
// cool, so any overshoot has to be waited out passively. A
// ramped setpoint prevents the large initial error (and the
// proportional/integral "kick" that error would otherwise
// cause) that drives most of the overshoot.
//
// Tune this to your heater's thermal mass - start conservative
// and speed it up once you've confirmed there's no overshoot.
//
const double SETPOINT_RAMP_RATE = 1.5; // deg C per second

// The setpoint the PID loop is actually chasing right now.
double activeSetpoint = 0.0;


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
// These will likely need re-tuning after this update: the
// derivative term now acts on the measured temperature
// instead of on the error (see calculatePID), so it no longer
// spikes on setpoint changes and generally wants a similar or
// slightly lower Kd than before. Retune with the setpoint ramp
// active, not against a step change.
//

double Kp = 5.0;
double Ki = 0.2;
double Kd = 20.0;


// ============================================================
// PID INTERNAL VARIABLES
// ============================================================

// Integral accumulator
double integral = 0.0;

// PID output in percentage
//
// 0   = heater OFF
// 100 = maximum commanded heating
//
double pidOutput = 0.0;

// Previous temperature reading, used to compute the derivative
// term on measurement instead of on error (avoids "derivative
// kick" whenever the setpoint changes).
double previousTemperature = NAN;

// Low-pass filtered derivative estimate. Raw d(temp)/dt is
// noisy (thermocouple jitter, EMI near the heater); filtering
// it keeps the D term from throwing sudden, spurious kicks at
// the heater output.
double filteredDerivative = 0.0;

// Filter strength for the derivative term: 0 = fully filtered
// (D term barely moves), 1 = no filtering (raw derivative).
const double D_FILTER_ALPHA = 0.2;

// Anti-windup state: true when the *unclamped* PID output was
// pinned above 100 / below 0 on the previous cycle. Used to
// stop the integral from accumulating further in that
// direction (conditional integration), on top of the hard
// integral clamp below.
bool pidSaturatedHigh = false;
bool pidSaturatedLow = false;


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
// STARTUP TEMPERATURE READ (AVERAGED)
// ============================================================
//
// The MAX6675 takes ~220ms per conversion, and the very first
// reading right after power-up is often unreliable. This takes
// several spaced samples, discards any NaN (fault) readings,
// and averages the rest so control starts from a real,
// stable temperature instead of a single noisy sample.
//
// Returns NAN if every sample failed (e.g. thermocouple not
// connected).
//
double readStartupTemperature(uint8_t samples, uint16_t sampleDelayMs)
{
  double sum = 0.0;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < samples; i++)
  {
    double reading = thermocouple.readCelsius();

    if (!isnan(reading))
    {
      sum += reading;
      validSamples++;
    }

    delay(sampleDelayMs);
  }

  if (validSamples == 0)
  {
    return NAN;
  }

  return sum / validSamples;
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

  // ESP32 Arduino Core 2.x
  ledcSetup(
    PWM_CHANNEL,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );
  ledcAttachPin(
    HEATER_PIN,
    PWM_CHANNEL
  );

  // Heater OFF
  ledcWrite(
    PWM_CHANNEL,
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
      PWM_CHANNEL,
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


  // ==========================================================
  // INITIAL TEMPERATURE READING
  // ==========================================================
  //
  // Take several averaged samples instead of one single read.
  // This also naturally covers the old fixed 1500ms startup
  // delay, since 5 samples x 250ms ~= 1.25s.
  //

  Serial.println();
  Serial.println("Reading initial temperature...");

  currentTemperature =
    readStartupTemperature(5, 250);

  if (isnan(currentTemperature))
  {
    Serial.println(
      "WARNING: No valid startup reading - check thermocouple wiring."
    );
  }
  else
  {
    Serial.print("Startup temperature: ");
    Serial.print(currentTemperature, 1);
    Serial.println(" C");
  }

  // Seed the derivative filter and the setpoint ramp with the
  // real starting temperature so the first PID cycle doesn't
  // see an artificial jump.
  previousTemperature = currentTemperature;

  activeSetpoint =
    isnan(currentTemperature) ? 0.0 : currentTemperature;


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

  // Reset PID output
  pidOutput = 0.0;

  // Reset anti-windup saturation flags
  pidSaturatedHigh = false;
  pidSaturatedLow = false;

  // Reset timing
  previousPIDCalculation =
    millis();

  // Start the setpoint ramp from wherever the temperature
  // actually is right now, not from 0 or the previous target.
  // This is what makes the ramp a smooth trajectory from
  // "where we are" to "where we want to be", every time a new
  // target is confirmed.
  activeSetpoint =
    isnan(currentTemperature) ? 0.0 : currentTemperature;

  // Reset the derivative reference too, so the first PID cycle
  // of the new run doesn't see a spurious jump.
  previousTemperature = currentTemperature;
  filteredDerivative = 0.0;

  // Heater OFF
  ledcWrite(
    PWM_CHANNEL,
    0
  );
}


// ============================================================
// UPDATE RAMPED SETPOINT
// ============================================================
//
// Moves activeSetpoint toward setpoint by at most
// SETPOINT_RAMP_RATE degrees per second.
//

void updateRampedSetpoint(double dt)
{
  double maxStep = SETPOINT_RAMP_RATE * dt;

  if (activeSetpoint < setpoint)
  {
    activeSetpoint += maxStep;

    if (activeSetpoint > setpoint)
    {
      activeSetpoint = setpoint;
    }
  }
  else if (activeSetpoint > setpoint)
  {
    activeSetpoint -= maxStep;

    if (activeSetpoint < setpoint)
    {
      activeSetpoint = setpoint;
    }
  }
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

double calculatePID(double temperature)
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
  // SETPOINT RAMP (SOFT-START)
  // ----------------------------------------------------------

  updateRampedSetpoint(dt);


  // ----------------------------------------------------------
  // ERROR (against the ramped setpoint, not the raw target)
  // ----------------------------------------------------------

  double error =
    activeSetpoint -
    temperature;


  // ----------------------------------------------------------
  // PROPORTIONAL
  // ----------------------------------------------------------

  double P =
    Kp * error;


  // ----------------------------------------------------------
  // INTEGRAL, with conditional integration (anti-windup)
  // ----------------------------------------------------------
  //
  // Skip accumulating the integral if the output was already
  // pinned at a limit last cycle AND the error would keep
  // pushing it further into that same limit. This stops the
  // integral from ballooning during the ramp-up phase, which
  // is one of the main causes of overshoot once the ramp
  // finally lets the error close.
  //

  bool wouldWindUpHigh =
    pidSaturatedHigh && (error > 0.0);

  bool wouldWindUpLow =
    pidSaturatedLow && (error < 0.0);

  if (!wouldWindUpHigh && !wouldWindUpLow)
  {
    integral +=
      error * dt;
  }


  // ----------------------------------------------------------
  // INTEGRAL HARD CLAMP (secondary safety net)
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
  // DERIVATIVE ON MEASUREMENT, LOW-PASS FILTERED
  // ----------------------------------------------------------
  //
  // Computed from the change in temperature rather than the
  // change in error, so it does NOT spike whenever the
  // setpoint moves (only the ramp moves it now anyway, but
  // this also protects against a user re-confirming a new
  // target while control is active). The low-pass filter
  // smooths out thermocouple/EMI noise so the D term doesn't
  // throw spurious kicks at the heater.
  //

  double rawDerivative = 0.0;

  if (!isnan(previousTemperature))
  {
    rawDerivative =
      -(temperature - previousTemperature) / dt;
  }

  previousTemperature =
    temperature;

  filteredDerivative =
    (D_FILTER_ALPHA * rawDerivative) +
    ((1.0 - D_FILTER_ALPHA) * filteredDerivative);

  double D =
    Kd * filteredDerivative;


  // ----------------------------------------------------------
  // TOTAL PID
  // ----------------------------------------------------------

  double output =
    P +
    I +
    D;


  // ----------------------------------------------------------
  // LIMIT OUTPUT, and remember saturation for next cycle's
  // anti-windup check
  // ----------------------------------------------------------

  pidSaturatedHigh = (output > 100.0);
  pidSaturatedLow  = (output < 0.0);

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
      PWM_CHANNEL,
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
      PWM_CHANNEL,
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
    calculatePID(currentTemperature);


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
    PWM_CHANNEL,
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
      // Reset PID (also restarts the setpoint ramp from the
      // current temperature toward the new setpoint)
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
    activeSetpoint,
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
  // Setpoint (final target)
  // ----------------------------------------------------------

  Serial.print(
    setpoint,
    2
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Active (ramped) setpoint - useful for watching the ramp
  // on the Serial Plotter
  // ----------------------------------------------------------

  Serial.print(
    activeSetpoint,
    2
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // Error (against the ramped setpoint)
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
      activeSetpoint -
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
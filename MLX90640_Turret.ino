// ESP32 Thermal Tracking Turret  
// - Uses MLX90640 thermal camera to detect heat signatures  
// - Controls two stepper motors (pan/tilt) with PID for smooth aiming  
// - Separates camera processing and motor control onto separate cores for efficiency  
// - Fires when a valid heat target is detected within a defined area


#include <Adafruit_MLX90640.h>
#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


// --- EDITABLE PARAMETERS --- //

// ESP32 Pinout:
#define I2C_SDA 21
#define I2C_SCL 22

#define ENA_PIN_X 17
#define STEP_PIN_X 4
#define DIR_PIN_X 16
#define ENA_PIN_Y 19
#define STEP_PIN_Y 5
#define DIR_PIN_Y 18

#define FIRE_PIN 13
#define LED_PIN 2

// MLX90640 settings:
const int cameraPixelsX = 32, cameraPixelsY = 24;
const float cameraFOV = 55.0;
const int startupGracePeriod = 1000; // In milliseconds

const float minDetectionTemp = 30.0; // In degrees celcius
const float maxDetectionTemp = 200.0; // In degrees celcius
const int continueTrackingLostTarget = 100; // How many frames the turret will keep searching for a lost target, 0 to disable
const bool turretWandering = true; // When there is no valid target, the turret will wander clockwise looking for a target
const int wanderSpeed = 10;

// Stepper motor settings:
const float stepAngle = 1.8;
const int microstep = 32;

const bool enablePan = true;
const bool enableTilt = true;

const float maxSpeedX = 2000;
const float maxAccelX = 20000;
const int ratioX = -3;
const float maxSpeedY = 2000;
const float maxAccelY = 20000;
const int ratioY = 1;

// PID controller parameters:
const float constX = 100.0;
const float constY = 100.0;
const float Kp_X = 1.0, Ki_X = 0.0, Kd_X = 2000.0;
const float Kp_Y = 1.0, Ki_Y = 0.0, Kd_Y = 4000.0;
const float intConstraintX = 20.0;
const float intConstraintY = 20.0;
const float deadzoneX = 1.8;
const float deadzoneY = 1.8;

// Firing settings:
const bool enableFiring = true;
const int firingBoxSize = 2;
const float minFiringTemp = 30.0; // In degrees celcius
const float maxFiringTemp = 200.0; // In degrees celcius

// Debug options:
const bool sendCameraFeedback = false;
const bool sendMotorFeedback = false;
const bool sendGunFeedback = false;


// --- BEGIN CODE --- //

struct Target { // Create Target struct: x and y position, w and h dimensions, and validity of target
  float x;
  float y;
  int w;
  int h;
  bool valid;
};
Target target = {0, 0, 0, 0, false};


void mlxTask(void *param);
void motorTask(void *param);
Target getAveragePos();
void fireTurret(float* frame);

// Mutex ensures safe access to the 'target' data structure between tasks on seperate cores
SemaphoreHandle_t targetMutex;
TaskHandle_t mlxTaskHandle;
TaskHandle_t motorTaskHandle;

// Initiate MLX90640
TwoWire I2CBus = TwoWire(0);
Adafruit_MLX90640 mlx;
float frame[cameraPixelsX * cameraPixelsY];

// Initiate stepper motors
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;


float errorX = 0, integralX = 0, derivativeX = 0;
float errorY = 0, integralY = 0, derivativeY = 0;

const float centerX = (cameraPixelsX - 1.0) / 2.0;
const float centerY = (cameraPixelsY - 1.0) / 2.0;
const float maxOutputX = Kp_X * centerX + Ki_X * intConstraintX + Kd_X * cameraPixelsX;
const float maxOutputY = Kp_Y * centerY + Ki_Y * intConstraintY + Kd_Y * cameraPixelsY;


void setup() {
  Serial.begin(921600);
  I2CBus.begin(I2C_SDA, I2C_SCL, 400000);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &I2CBus)) { // Find MLX90640 on I2C Bus
    Serial.println("MLX90640 not found!");
    while (1) delay(10);
  }
  Serial.println("MLX90640 detected.");

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_16_HZ); // Set MLX90640's refresh rate to 16HZ (maximum possible refresh rate on the given hardware)

  engine.init();

  stepperX = engine.stepperConnectToPin(STEP_PIN_X);
  if (stepperX) {
    stepperX->setDirectionPin(DIR_PIN_X);
    stepperX->setEnablePin(ENA_PIN_X);
    stepperX->setAutoEnable(false);
    stepperX->setSpeedInHz(fabs(maxSpeedX * microstep * ratioX));
    stepperX->setAcceleration(maxAccelX);
  }
  stepperY = engine.stepperConnectToPin(STEP_PIN_Y);
  if (stepperY) {
    stepperY->setDirectionPin(DIR_PIN_Y);
    stepperY->setEnablePin(ENA_PIN_Y);
    stepperY->setAutoEnable(false);
    stepperY->setSpeedInHz(fabs(maxSpeedY * microstep * ratioY));
    stepperY->setAcceleration(maxAccelY);
  }

  pinMode(ENA_PIN_X, OUTPUT);
  pinMode(ENA_PIN_Y, OUTPUT);
  if (enablePan) {digitalWrite(ENA_PIN_X, LOW);}
  else {digitalWrite(ENA_PIN_X, HIGH);}
  if (enableTilt) {digitalWrite(ENA_PIN_Y, LOW);}
  else {digitalWrite(ENA_PIN_Y, HIGH);}

  pinMode(FIRE_PIN, OUTPUT);
  pinMode(2, OUTPUT);

  // Seperate MLX and motor tasks on different cores
  targetMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    mlxTask, "MLX90640Task", 8192, NULL, 2, &mlxTaskHandle, 1
  );
  xTaskCreatePinnedToCore(
    motorTask, "StepperMotorTask", 4096, NULL, 1, &motorTaskHandle, 0
  );
}


void loop() {vTaskDelay(portMAX_DELAY);}


// Handles MLX90640's thermal camera data capture
// Runs on Core 1 at defined refresh rate to avoid bottlenecking the motor task's speed
void mlxTask(void *param) {
  unsigned long startTime = millis();

  do { // Do not find target during grace period
    digitalWrite(FIRE_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    if (xSemaphoreTake(targetMutex, portMAX_DELAY)) {
      target = {0, 0, 0, 0, false};
      xSemaphoreGive(targetMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  } while (millis() - startTime < startupGracePeriod);

  for (;;) { // Loop indefinitely on core 1 (when grace period ends)
    Target newTarget = getAveragePos(); // Find a target

    if (newTarget.valid && enableFiring) { // Check validity of target
      fireTurret(frame);
    }
    else {
      digitalWrite(FIRE_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
    }

    if (xSemaphoreTake(targetMutex, portMAX_DELAY)) {
      target = newTarget;
      xSemaphoreGive(targetMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


// Handles stepper motor control and PID tracking
// Runs on Core 0 to loop as fast as possible on the given hardware
void motorTask(void *param) {
  for (;;) { // Loop indefinitely
    if (xSemaphoreTake(targetMutex, portMAX_DELAY)) {
      if (target.valid) { // Check validity of target
        if (enableTilt) {digitalWrite(ENA_PIN_Y, LOW);}
        PIDController(target.x, target.y, target.w, target.h); // Move turret to target's position
      }
      else { // If target is not valid, try to find a new target
        if (turretWandering) {
          digitalWrite(ENA_PIN_Y, HIGH);
          stepperX->setSpeedInHz(fabs(wanderSpeed * microstep * ratioX));
          if (ratioX > 0) {
            stepperX->runForward();
          }
          else {
            stepperX->runBackward();
          }
        }
        else {
          stepperX->stopMove();
          stepperY->stopMove();
        }
      }
      xSemaphoreGive(targetMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


// Loops through all pixels in the MLX90640's image array
// Calculates weighted centroid of hot spots to determine target position and dimensions
Target getAveragePos() {
  Target pos = {0, 0, 0, 0, false};
  static Target lastValidPos = {0, 0, 0, 0, false};
  static int missedFrameCount = 0;

  if (mlx.getFrame(frame) != 0) {
    Serial.println("Failed to get frame");
    return pos;
  }

  float maxTemp = 0;
  float weightedX = 0;
  float weightedY = 0;
  float totalWeight = 0;
  int hotPixelCount = 0;

  int minX = cameraPixelsX, maxX = 0;
  int minY = cameraPixelsY, maxY = 0;

  // Loop through MLX90640's pixel array
  for (int y = 0; y < cameraPixelsY; ++y) {
    for (int x = 0; x < cameraPixelsX; ++x) {
      int index = y * cameraPixelsX + x;
      float temp = frame[index];

      if (temp > maxTemp) { // Record the maximum detected temperature on the array
        maxTemp = temp;
      }

      if ((temp >= minDetectionTemp) && (temp <= maxDetectionTemp)) { // Cheeck if pixel's temperature is within the detection range
        // Find the average position of the target's heat accross all pixels
        weightedX += x * temp;
        weightedY += y * temp;
        totalWeight += temp;
        hotPixelCount++;

        // Estimate target's dimensions
        if (x < minX) {minX = x;}
        if (x > maxX) {maxX = x;}
        if (y < minY) {minY = y;}
        if (y > maxY) {maxY = y;}
      }
    }
  }

  if (hotPixelCount > 0) {
    pos.x = weightedX / totalWeight;
    pos.y = weightedY / totalWeight;
    pos.w = maxX - minX + 1;
    pos.h = maxY - minY + 1;
    pos.valid = true; // Target is only valid if there is at least 1 pixel within the temperature detection range

    lastValidPos = pos;
    missedFrameCount = 0;

    if (sendCameraFeedback) {
      Serial.printf("Heat centroid: (%.2f, %.2f), Heat dimensions: (%d, %d), Max Temp: %.2f°C\n", pos.x, pos.y, pos.w, pos.h, maxTemp);
    }

    return pos;
  }
  else if (continueTrackingLostTarget > 0 && missedFrameCount != continueTrackingLostTarget && lastValidPos.valid) { // If current target is invalid and last target was valid, search for the lost target
    if (missedFrameCount < continueTrackingLostTarget) {
      missedFrameCount++;

      if (sendCameraFeedback) {
        Serial.printf("Target lost, (%d/%d)\n", missedFrameCount, continueTrackingLostTarget);
      }

      lastValidPos.y = centerY;
      return lastValidPos;
    }
    else {
      missedFrameCount = continueTrackingLostTarget;
    }
  }
  else if (sendCameraFeedback) {
    Serial.println("No hot pixels in detection range.");
  }

  return pos;
}


// PID control logic to smoothly move turret towards detected target position
// Uses proportional, integral, and derivative terms to minimize overshooting / undershooting
void PIDController(float x, float y, float w, float h) {
  static float prevErrorX = 0;
  static float prevErrorY = 0;

  errorX = centerX - x;
  errorY = centerY - y;

  integralX += errorX;
  integralY += errorY;

  derivativeX = errorX - prevErrorX;
  derivativeY = errorY - prevErrorY;

  integralX = constrain(integralX, 0, intConstraintX);
  integralY = constrain(integralY, 0, intConstraintY);

  // PID controller algorithm
  float outputX = (Kp_X * errorX + Ki_X * integralX + Kd_X * derivativeX) * ratioX * constX;
  float outputY = (Kp_Y * errorY + Ki_Y * integralY + Kd_Y * derivativeY) * ratioY * constY;

  prevErrorX = errorX;
  prevErrorY = errorY;

  float speedX = mapFloat(fabs(outputX), 0, maxOutputX, 0, maxSpeedX * microstep);
  float speedY = mapFloat(fabs(outputY), 0, maxOutputY, 0, maxSpeedY * microstep);
  // Estimate target's distance based on the 2D dimensions on MLX array
  float widthFactor = mapFloat(w, 0, cameraPixelsX, 0, 1);
  float heightFactor = mapFloat(h, 0, cameraPixelsY, 0, 1);

  if (stepperX) {
    if (fabs(errorX) * (cameraFOV / cameraPixelsX) * 2 < deadzoneX * fabs(ratioX) * widthFactor) { // If turret is already aimed at target given a certain deadzone, stop moving
      stepperX->stopMove();
      if (sendMotorFeedback) {Serial.printf("Moving Stopped\n");}
    }
    else { // Otherwise, move in accordance to the PID controller to track the target
      stepperX->setSpeedInHz(speedX);
      if (outputX < 0) {
        stepperX->runForward();
        if (sendMotorFeedback) {Serial.printf("Moving Left,  %.2lf\n", speedX);}
      }
      else if (outputX > 0) {
        stepperX->runBackward();
        if (sendMotorFeedback) {Serial.printf("Moving Right, %.2lf\n", speedX);}
      }
      else {
        stepperX->stopMove();
        if (sendMotorFeedback) {Serial.printf("Moving Stopped\n");}
      }
    }
  }
  if (stepperY) {
    if (fabs(errorY) * (0.75 * cameraFOV / cameraPixelsY) * 2 < deadzoneY * fabs(ratioY) * heightFactor) {
      stepperY->stopMove();
    }
    else {
      stepperY->setSpeedInHz(speedY);
      if (outputY < 0) {stepperY->runForward();}
      else if (outputY > 0) {stepperY->runBackward();}
      else {stepperY->stopMove();}
    }
  }
}


// Activates firing mechanism if target's heat signature is centered on the predefined firing area
void fireTurret(float* frame) {
  int halfSize = firingBoxSize / 2;

  int centerX = cameraPixelsX / 2;
  int centerY = cameraPixelsY / 2;

  int startX = centerX - halfSize;
  int startY = centerY - halfSize;

  for (int y = startY; y < startY + firingBoxSize; y++) {
    for (int x = startX; x < startX + firingBoxSize; x++) {
      if (x < 0 || x >= cameraPixelsX || y < 0 || y >= cameraPixelsY) {continue;} // If target is within the firing box, fire the turret
      float temp = frame[y * cameraPixelsX + x];
      if (temp >= minFiringTemp && temp <= maxFiringTemp) { // Verify target is within the temperature detection range
        digitalWrite(FIRE_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        if (sendGunFeedback) {Serial.printf("Turret Firing\n");}
        return;
      }
    }
  }

  digitalWrite(FIRE_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  if (sendGunFeedback) {Serial.printf("Turret Stopping\n");}
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Custom float version of Arduino's map() function; scales values proportionally to a given range, works with float values
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


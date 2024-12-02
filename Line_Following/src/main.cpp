// #include <Arduino.h>
// #include <WiFi.h>
// #include <WiFiClient.h>
// #include <Adafruit_MCP3008.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// // WiFi credentials
// const char* ssid = "Eli's iPhone";
// const char* password = "myPassword";

// // Server IP and Port (replace with your computer's IP address)
// const char* server_ip = "172.20.10.2";  // IP address of the Python script
// const uint16_t server_port = 10000;       // Port number to connect to

// WiFiClient client;

// // Motor Pins (if you still need them)
// const unsigned int M1_IN_1 = 13;
// const unsigned int M1_IN_2 = 12;
// const unsigned int M2_IN_1 = 25;
// const unsigned int M2_IN_2 = 14;

// // PWM Channels
// const unsigned int M1_IN_1_CHANNEL = 8;
// const unsigned int M1_IN_2_CHANNEL = 9;
// const unsigned int M2_IN_1_CHANNEL = 10;
// const unsigned int M2_IN_2_CHANNEL = 11;

// const int freq = 5000;
// const int resolution = 8;

// // Initialize the MPU6050
// Adafruit_MPU6050 mpu;

// // Variables for Gyroscope Calibration and Angle Tracking
// float gyroBias = 0;

// //  global variables to keep track of color counts
// int redCount = 0;
// int blueCount = 0;
// int greenCount = 0;
// int yellowCount = 0;

// bool detectionInProgress = false;

// //  variable to track how many times to run color detection
// int colorDetectionRuns = 3; 
// int runsCompleted = 0;


// // to track color detection phase
// enum DetectionPhase {
//   NONE,
//   FIRST_DETECTION,
//   SECOND_DETECTION,
//   AUDIO_DETECTION
// };

// DetectionPhase currentDetectionPhase = NONE;

// // Function to calibrate the gyroscope and determine its bias
// void calibrateGyro() {
//   sensors_event_t accel, gyro, temp;
//   float totalZ = 0;
//   const int numSamples = 100;

//   Serial.println("Calibrating gyroscope...");
//   for (int i = 0; i < numSamples; i++) {
//     mpu.getEvent(&accel, &gyro, &temp);
//     totalZ += gyro.gyro.z;
//     delay(10);
//   }
//   gyroBias = totalZ / numSamples; // Calculate average bias
//   Serial.print("Gyro Z-axis bias: ");
//   Serial.println(gyroBias);
// }

// void setLeftSpeed(int speed) {
//   int pwm = constrain(abs(speed), 0, 255);
//   if (speed > 0) {
//     // Forward
//     ledcWrite(M1_IN_1_CHANNEL, 0);
//     ledcWrite(M1_IN_2_CHANNEL, pwm);
//   } else if (speed < 0) {
//     // Reverse
//     ledcWrite(M1_IN_1_CHANNEL, pwm);
//     ledcWrite(M1_IN_2_CHANNEL, 0);
//   } else {
//     // Stop
//     ledcWrite(M1_IN_1_CHANNEL, 0);
//     ledcWrite(M1_IN_2_CHANNEL, 0);
//   }
// }

// void setRightSpeed(int speed) {
//   int pwm = constrain(abs(speed), 0, 255);
//   if (speed > 0) {
//     // Forward
//     ledcWrite(M2_IN_1_CHANNEL, 0);
//     ledcWrite(M2_IN_2_CHANNEL, pwm);
//   } else if (speed < 0) {
//     // Reverse
//     ledcWrite(M2_IN_1_CHANNEL, pwm);
//     ledcWrite(M2_IN_2_CHANNEL, 0);
//   } else {
//     // Stop
//     ledcWrite(M2_IN_1_CHANNEL, 0);
//     ledcWrite(M2_IN_2_CHANNEL, 0);
//   }
// }

// void stopMotors() {
//   setLeftSpeed(0);
//   setRightSpeed(0);
// }

// void stopMotorsForTime(int duration) {
//   setLeftSpeed(0);
//   setRightSpeed(0);
//   delay(duration);
//   stopMotors();
// }

// void moveForwardForTime(int speed, int duration) {
//   setLeftSpeed(speed);
//   setRightSpeed(speed + 10);
//   delay(duration);
//   stopMotors();
// }

// void moveBackwardForTime(int speed, int duration) {
//   setLeftSpeed(-speed);
//   setRightSpeed(-speed - 15); // Adjust speed for balance
//   delay(duration);
//   stopMotors();
// }

// // ADC (line sensor)
// Adafruit_MCP3008 adc1;
// Adafruit_MCP3008 adc2;

// const unsigned int ADC_1_CS = 2;
// const unsigned int ADC_2_CS = 17;

// int adc1_buf[8];
// int adc2_buf[8];
// uint8_t lineArray[13];  // 13 sensors

// int maxValues[13] = {
//   /* Sensor 0 */ 691,
//   /* Sensor 1 */ 695,
//   /* Sensor 2 */ 683,
//   /* Sensor 3 */ 694,
//   /* Sensor 4 */ 691,
//   /* Sensor 5 */ 690,
//   /* Sensor 6 */ 702,
//   /* Sensor 7 */ 693,
//   /* Sensor 8 */ 695,
//   /* Sensor 9 */ 703,
//   /* Sensor 10 */ 703,
//   /* Sensor 11 */ 702,
//   /* Sensor 12 */ 710
// };

// int minValues[13] = {
//   /* Sensor 0 */ 631,
//   /* Sensor 1 */ 646,
//   /* Sensor 2 */ 605,
//   /* Sensor 3 */ 635,
//   /* Sensor 4 */ 619,
//   /* Sensor 5 */ 607,
//   /* Sensor 6 */ 653,
//   /* Sensor 7 */ 607,
//   /* Sensor 8 */ 608,
//   /* Sensor 9 */ 641,
//   /* Sensor 10 */ 657,
//   /* Sensor 11 */ 652,
//   /* Sensor 12 */ 673
// };

// int thresholds[13];  // Thresholds calculated from min and max values

// void readADC() {
//   for (int i = 0; i < 8; i++) {
//     adc1_buf[i] = adc1.readADC(i);
//     adc2_buf[i] = adc2.readADC(i);
//   }
// }

// void digitalConvert() {
//   // Calculate thresholds if not already calculated
//   static bool thresholdsCalculated = false;
//   if (!thresholdsCalculated) {
//     for (int i = 0; i < 13; i++) {
//       thresholds[i] = (maxValues[i] + minValues[i]) / 2;
//     }
//     thresholdsCalculated = true;
//   }

//   // Convert analog readings to digital based on individual thresholds
//   for (int i = 0; i < 7; i++) {
//     int index = 2 * i;
//     lineArray[index] = adc1_buf[i] < thresholds[index] ? 1 : 0;  // Inverted comparison
//     if (i < 6) {
//       lineArray[index + 1] = adc2_buf[i] < thresholds[index + 1] ? 1 : 0;
//     }
//   }
// }

// void printLineArray() {
//   Serial.print("Line Array: ");
//   for (int i = 0; i < 13; i++) {
//     Serial.print(lineArray[i]);
//     if (i < 12) {
//       Serial.print(", ");
//     }
//   }
//   Serial.println();
// }

// int calculateLinePosition() {
//   long numerator = 0;
//   int denominator = 0;

//   // Define the weights for each sensor, excluding sensors 8 and 12
//   int weights[13] = {-6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6};

//   for (int i = 0; i < 13; i++) {
//     // if (i == 12) {
//     //   continue;  // Skip sensor 12
//     // }

//     int sensorValue = lineArray[i];
//     numerator += (long)sensorValue * weights[i];
//     denominator += sensorValue;
//   }

//   if (denominator != 0) {
//     int position = numerator / denominator;
//     return position;
//   } else {
//     // Line lost; consider implementing a recovery strategy
//     return 0;
//   }
// }

// bool allWhite() {
//   for (int i = 0; i < 13; i++) {
//     if (lineArray[i] != 1) {
//       return false;
//     }
//   }
//   return true;
// }

// bool allBlack() {
//   for (int i = 0; i < 13; i++) {
//     if (lineArray[i] != 0) {
//       return false;
//     }
//   }
//   return true;
// }

// // PID control variables
// float Kp = 7.0;     // Proportional gain
// float Ki = 0.0;     // Integral gain
// float Kd = 50.0;     // Derivative gain

// float previousError = 0;
// float integral = 0;

// float pidCompute(int error) {
//   float P = error;
//   integral += error;
//   float D = error - previousError;
//   previousError = error;

//   float correction = Kp * P + Ki * integral + Kd * D;
//   return correction;
// }

// int isCorner() {
//   // If all sensors are white, it's an all-white area, not a corner
//   if (allWhite()) {
//     return 0;
//   }

//   // Check for left corner (right turn)
//   if (lineArray[6] == 1 && lineArray[7] == 1 && lineArray[8] == 1 && lineArray[9] == 1 && lineArray[10] == 1 && lineArray[11] == 1 && lineArray[12] == 1) {
//     // Ensure other sensors are not also white (to avoid false positives)
//     for (int i = 0; i <= 5; i++) {
//       if (lineArray[i] == 1) {
//         // Other sensors are also white; it's not a corner
//         return 0;
//       }
//     }
//     Serial.print(", left corner, ");
//     return 1; // left corner
//   }
//   // Check for right corner (left turn)
//   else if (lineArray[6] == 1 && lineArray[5] == 1 && lineArray[4] == 1 && lineArray[3] == 1 && lineArray[2] == 1 && lineArray[1] == 1 && lineArray[0] == 1) {
//     // Ensure other sensors are not also white (to avoid false positives)
//     for (int i = 7; i <= 12; i++) {
//       if (lineArray[i] == 1) {
//         // Other sensors are also white; it's not a corner
//         return 0;
//       }
//     }
//     Serial.print(", right corner, ");
//     return 2; // right corner
//   } else {
//     Serial.print(", no corner, ");
//     return 0; // no corner
//   }
// }

// void followLine() {
//   // Calculate line position
//   int linePosition = calculateLinePosition();

//   // Define desired position (centered on the line)
//   int desiredPosition = 0;

//   // Calculate error
//   int error = desiredPosition - linePosition;

//   // Compute PID correction
//   float correction = pidCompute(error);

//   // Set base speeds for the motors
//   int baseSpeedLeft = 75;   // Base speed for the left motor
//   int baseSpeedRight = 85;  // Base speed for the right motor (adjusted)

//   // Calculate motor speeds
//   int leftSpeed = baseSpeedLeft + correction;
//   int rightSpeed = baseSpeedRight - correction;

//   // Constrain motor speeds to valid range
//   leftSpeed = constrain(leftSpeed, 0, 100);
//   rightSpeed = constrain(rightSpeed, 0, 100);

//   // Set motor speeds
//   setLeftSpeed(leftSpeed);
//   setRightSpeed(rightSpeed);

//   // Optional: Print debugging information
//   Serial.print("Line Position: ");
//   Serial.print(linePosition);
//   Serial.print(" | Error: ");
//   Serial.print(error);
//   Serial.print(" | Correction: ");
//   Serial.print(correction);
//   Serial.print(" | Left Speed: ");
//   Serial.print(leftSpeed);
//   Serial.print(" | Right Speed: ");
//   Serial.println(rightSpeed);
// }

// void turnFunction(int degrees, int motor) {
//   float angleZ = 0; // Tracks the total angle rotated
//   unsigned long previousTime = millis();

//   Serial.print("Starting ");
//   Serial.print(degrees);
//   Serial.print("-degree turn using ");
//   Serial.print((motor > 0) ? "Motor M" : "Motor M");
//   Serial.println(abs(motor));

//   // Determine which motor to control and direction
//   int speed = (motor > 0) ? 80 : -80; // Adjusted speed to 80 for both directions

//   if (motor == 1) { // Motor M1
//     setLeftSpeed(-speed);
//     setRightSpeed(speed);
//   } else if (motor == 2) { // Motor M2
//     setRightSpeed(-speed);
//     setLeftSpeed(speed);
//   } else {
//     Serial.println("Invalid motor selection!");
//     return;
//   }

//   sensors_event_t accel, gyro, temp;
//   while (abs(angleZ) < degrees) {
//     mpu.getEvent(&accel, &gyro, &temp);

//     // Calculate elapsed time in seconds
//     unsigned long currentTime = millis();
//     float elapsedTime = (currentTime - previousTime) / 1000.0;

//     // Correct gyroscope data for bias and integrate angular velocity
//     float correctedGyroZ = gyro.gyro.z - gyroBias;
//     angleZ += correctedGyroZ * elapsedTime * (180.0 / PI);

//     // Debugging output
//     Serial.print("Elapsed Time (s): ");
//     Serial.print(elapsedTime, 6);
//     Serial.print(" | Corrected Gyro Z (rad/s): ");
//     Serial.print(correctedGyroZ);
//     Serial.print(" | Current Angle (deg): ");
//     Serial.println(angleZ);

//     previousTime = currentTime; // Update the previous time
//     delay(10);
//   }

//   // Stop all motors after completing the turn
//   stopMotors();
//   Serial.println("Turn complete! All motors stopped.");
// }

// String mostFrequentColor = "";

// void traverseLargeWhiteArea() {
//   moveForwardForTime(75, 1000);
//   stopMotorsForTime(100);
//   turnFunction(60, 2);
//   stopMotorsForTime(100);
//   moveForwardForTime(75, 600);
//   stopMotors();

//   readADC();
//   digitalConvert();
//   stopMotorsForTime(1000);

//   while(allBlack()) {
//     turnFunction(155, 1);
//     stopMotorsForTime(100);
//     moveForwardForTime(75, 600);
//     stopMotorsForTime(100);
//     turnFunction(70, 2);
//     stopMotorsForTime(100);
//     moveForwardForTime(75, 600);
//     stopMotors();

//     readADC();
//     digitalConvert();
//   }
// }

// void allWhiteDetected() {
//   stopMotors();
//   Serial.println("Large white area detected. Driving forward to verify...");

//   // Drive forward to verify the white area
//   moveForwardForTime(75, 200); // Adjust duration as needed
//   stopMotors();

//   // Re-read sensors
//   readADC();
//   digitalConvert();
//   Serial.println("Rechecking surface...");

//   if (allBlack()) {
//     Serial.println("Black detected after moving forward. Initiating audio detection.");

//     // Ensure the robot is stopped
//     stopMotors();

//     // Drive backward slightly
//     moveBackwardForTime(75, 300);
//     stopMotorsForTime(100);

//   } else if (allWhite()) {
//     Serial.println("Still in white area.");
//     moveBackwardForTime(75, 400);
//     stopMotorsForTime(100);
//     traverseLargeWhiteArea();
//   }
// }

// void setup() {
//   Serial.begin(115200);

//   // Stop the right motor by setting pin 14 low
//   pinMode(14, OUTPUT);
//   digitalWrite(14, LOW);

//   // Initialize MPU6050
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) delay(10);
//   }
//   Serial.println("MPU6050 Found!");

//   // Configure MPU6050 settings
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   // Calibrate gyroscope
//   calibrateGyro();

//   adc1.begin(ADC_1_CS);
//   adc2.begin(ADC_2_CS);

//   // Setup PWM channels
//   ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
//   ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
//   ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
//   ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

//   // Attach PWM channels to motor pins
//   ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
//   ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
//   ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
//   ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);
// }

// void loop() {
  
//   readADC();
//   digitalConvert();

//   if (allWhite()) {
//     stopMotors();
//     Serial.println("All white detected. Verifying area...");
//     allWhiteDetected();

//   } else if (allBlack()) {
//     stopMotors();

//     stopMotorsForTime(50);
//     moveForwardForTime(75, 10);
//     Serial.println("All black detected. Moving slow.");

//   } else {
//     int corner = isCorner();

//     if (corner == 1) {
//       moveForwardForTime(75, 100); 
//       turnFunction(70, 1);

//     } else if (corner == 2) {
//       moveForwardForTime(75, 100); 
//       turnFunction(70, 2);

//     } else {
//       // Continue line following
//       followLine();
//     }
//   }
// }




#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Motor Pins (if you still need them)
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

// PWM Channels
const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const int freq = 5000;
const int resolution = 8;


void setLeftSpeed(int speed) {
  int pwm = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    // Forward
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, pwm);
  } else if (speed < 0) {
    // Reverse
    ledcWrite(M1_IN_1_CHANNEL, pwm);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  } else {
    // Stop
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  }
}

void setRightSpeed(int speed) {
  int pwm = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    // Forward
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, pwm);
  } else if (speed < 0) {
    // Reverse
    ledcWrite(M2_IN_1_CHANNEL, pwm);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  } else {
    // Stop
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  }
}

void stopMotors() {
  setLeftSpeed(0);
  setRightSpeed(0);
}

void stopMotorsForTime(int duration) {
  setLeftSpeed(0);
  setRightSpeed(0);
  delay(duration);
  stopMotors();
}

void moveForwardForTime(int speed, int duration) {
  setLeftSpeed(speed+13);
  setRightSpeed(speed);
  delay(duration);
  stopMotors();
}

void moveBackwardForTime(int speed, int duration) {
  setLeftSpeed(-speed-13);
  setRightSpeed(-speed); // Adjust speed for balance
  delay(duration);
  stopMotors();
}

void setup() {
  Serial.begin(115200);

  // Stop the right motor by setting pin 14 low
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  
  // Setup PWM channels
  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  // Attach PWM channels to motor pins
  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);
  
}

void loop() {  
  stopMotorsForTime(2000);
  moveForwardForTime(80, 500);
  //stopMotorsForTime(1000);
  //moveBackwardForTime(80, 600);

  while(true){
    
  }
 
}

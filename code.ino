// setup- assume the ren and len pins have been connected to 5v
long steps = 0;   // Pulses from Hall Effect sensors
const int hallPinA = 2;
const int hallPinB = 3;
const int RPWM = 5;  // Forward PWM
const int LPWM = 6;  // Reverse PWM

// Motor Function Variables
int targetNumber;
float cm;
int currentPosition;
int lastPosition=0;
bool active = true; //change
bool EOSFlag = false;

int maxStroke;
int minStroke = 0;

// Variables for Debounce
const unsigned long motionTimeout = 500;  // Adjust this value based on your requirements (in milliseconds)
const unsigned long CALIBRATION_TIMEOUT=3000; // Adjust this value based on your requirements (in milliseconds)
unsigned long lastMotionTime = millis();

void setup() {
  pinMode(hallPinA, INPUT);
  pinMode(hallPinB, INPUT);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(hallPinA), countSteps, RISING);

  Serial.begin(9600);

  // Example: run forward at low speed
  delay(1000);
  Serial.println("done setting up");
  homingRoutine();
  // calibrateActuator();
}

//Home the linear actuator to fill retraction
void homingRoutine() {
  active=true;
  Serial.println("Homing Initiated");
  analogWrite(RPWM, 0); 
  analogWrite(LPWM, 250);  
  lastMotionTime = millis();      // <-- reset here after motor starts
  lastPosition = currentPosition; // <-- so motion detection starts fresh
  while (!EOSFlag) {
    isEndOfStroke();
    // Move actuator to full retraction
  }
  stopMotor();
  minStroke = 0;
  steps = 0;
  currentPosition = 0;
  Serial.println("Homing Completed");
  delay(50);
}

void stopMotor() {
  if (active) {
    active = false;
    analogWrite(LPWM, 0); // 0-255 PWM
    analogWrite(RPWM, 0);  // reverse off
  }
}

//check if reached end of stroke
bool isEndOfStroke() {
  // Check if there is motion (changes in the pulse count)
  if (active && (currentPosition != lastPosition)) {
    lastMotionTime = millis();  // Update the time of the last motion
    lastPosition = currentPosition;
    EOSFlag=false;
  }

  // Check if there is no motion for the specified timeout
  if (active && ((millis() - lastMotionTime) > motionTimeout)){
    if(EOSFlag!=true) {
      Serial.print("Timeout - ");
      Serial.println("At limit");
      EOSFlag=true;
    }
    stopMotor();
    return true;
  }
  return false;
}

//calibrate what the longest position is
void calibrateActuator() {
  Serial.println("Calibration Initiated");
  active = true;
  lastMotionTime=millis();
  EOSFlag = false;                // reset end flag
  lastMotionTime = millis();      // ensure timeout timer is fresh
  lastPosition = currentPosition; // so motion detection works

  // Move actuator to full extension
  analogWrite(LPWM, 0); 
  analogWrite(RPWM, 250);  

  // Wait until the end of stroke is reached during calibration
  while (!isEndOfStroke()) {
    // Add a timeout condition to avoid infinite loop
    if (millis() - lastMotionTime > motionTimeout) {
      Serial.println("Calibration Timeout");
        break;
    }
  }
  stopMotor();
  maxStroke=currentPosition;
  // Print the calibration results
  Serial.print("Calibration Complete. Minimum Stroke: ");
  Serial.print(minStroke);
  Serial.print(" Maximum Stroke: ");
  Serial.println(maxStroke);
  targetNumber=maxStroke;
}

void loop() {
  if (!active && Serial.available() > 0) {
    String serialInput = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(serialInput);
    if (serialInput.length() > 0) {
        cm = serialInput.toFloat();
        targetNumber = round(cm*411);
        //percent = serialInput.toInt();
        //targetNumber = map(percent, 0, 100, minStroke, maxStroke);
        
    /*If the above line is active, you will input a value between 0-100,
      The program will use this input as a % of stroke.*/

        Serial.print("Target number: ");
        Serial.println(targetNumber);
        EOSFlag = false;
      }
      // Clear the serial buffer
      while (Serial.available()) {
        Serial.read();
      }
    }
  if (targetNumber != currentPosition) {
    active = true;
    movement();
  } 
  if (active && targetNumber == currentPosition) {
   stopMotor();
   Serial.println("Target Met");
   delay(500);
   if (currentPosition + 100 > targetNumber && currentPosition - 100 < targetNumber){
      Serial.println("OKAY!!!!");
      targetNumber = currentPosition;
    }
  }
}

void movement() {
  if (targetNumber > currentPosition) {
    analogWrite(LPWM, 0); 
    analogWrite(RPWM, 255);
    Serial.println(" Extending");
  } else if (targetNumber < currentPosition) {
    analogWrite(LPWM, 255); 
    analogWrite(RPWM, 0);
    Serial.println("Retracting");
  } else if (targetNumber == currentPosition) {
    stopMotor();
    delay(100); 
    return;
  }
  if (isEndOfStroke()) {
  return;  // Skip further movement actions
  }
}

void countSteps() {
  bool A = digitalRead(hallPinA);
  bool B = digitalRead(hallPinB);
  

  if (A != B) {
    steps++;
  } else {
    steps--;
  }
  currentPosition = steps;
}

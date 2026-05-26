//Servo motor library#include <Servo.h>

// --- 1. PIN CONNECTIONS ---
const int ldrFront = A0, ldrLeft = A1, ldrRight = A2, ldrBack = A3;
const int batteryPin = A4, servoPin = 11, STBY = 4;
const int trigPin = 7, echoPin = 8; 
const int AIN1 = 3, AIN2 = 2, PWMA = 9;   
const int BIN1 = 5, BIN2 = 6, PWMB = 10;  

Servo myServo;

// --- 2. MOVEMENT SETTINGS ---
const int fullPower = 255;    
const int wallDistance = 5;   
const int recoilTime = 220;   

// --- 3. INCREMENTAL & PRECISION SETTINGS ---
int currentPos = 90;
const int minServo = 65, maxServo = 115;  
const int minMoveStep = 10;       
const int minWheelBurst = 180;    
const int globalThreshold = 45;      

// --- ASYMMETRIC SMOOTHING ---
const float smoothUp = 0.7;   
const float smoothDown = 0.15; 

float smoothF = 0, smoothB = 0, smoothL = 0, smoothR = 0;

// --- 4. NON-BLOCKING & SHADE SETTINGS ---
unsigned long lastActionTime = 0;
const int decisionInterval = 400;    

unsigned long motorStopTime = 0;
unsigned long servoDetachTime = 0;
bool motorsRunning = false;
bool servoAttached = false;

int baselineLight = 0;
bool hibernating = false;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  myServo.attach(servoPin);
  myServo.write(currentPos);
  delay(600);
  myServo.detach(); 

  smoothF = analogRead(ldrFront);
  smoothL = analogRead(ldrLeft);
  smoothR = analogRead(ldrRight);
  smoothB = analogRead(ldrBack);
  baselineLight = (smoothF + smoothL + smoothR + smoothB) / 4;
}

void loop() {
  // --- NON-BLOCKING HARDWARE CUTOFFS ---
  if (motorsRunning && millis() >= motorStopTime) {
    stopMotors();
    motorsRunning = false;
  }
  if (servoAttached && millis() >= servoDetachTime) {
    myServo.detach();
    servoAttached = false;
  }

  // --- DECISION LOOP ---
  if (millis() - lastActionTime >= decisionInterval) {
    
    // 1. Read raw sensor data
    int rawF = analogRead(ldrFront); int rawL = analogRead(ldrLeft);
    int rawR = analogRead(ldrRight); int rawB = analogRead(ldrBack);

    // 2. ANOMALY SAFEGUARD (OUTLIER REJECTION)
    int totalRaw = rawF + rawL + rawR + rawB;
    int avgOther;

    avgOther = (totalRaw - rawF) / 3;
    if (rawF < avgOther * 0.25) rawF = avgOther;

    avgOther = (totalRaw - rawL) / 3;
    if (rawL < avgOther * 0.25) rawL = avgOther;

    avgOther = (totalRaw - rawR) / 3;
    if (rawR < avgOther * 0.25) rawR = avgOther;

    avgOther = (totalRaw - rawB) / 3;
    if (rawB < avgOther * 0.25) rawB = avgOther;

    // 3. Apply Asymmetric Smoothing Filter
    smoothF = applySmoothing(rawF, smoothF);
    smoothL = applySmoothing(rawL, smoothL);
    smoothR = applySmoothing(rawR, smoothR);
    smoothB = applySmoothing(rawB, smoothB);

    int f = round(smoothF); int l = round(smoothL);
    int r = round(smoothR); int b = round(smoothB);

    int avgLight = (f + l + r + b) / 4;

    if (avgLight > baselineLight) {
      baselineLight = avgLight;
    }

    delay(10); 
    long dist = getDistance(); 

    // --- TELEMETRY ---
    Serial.print("DIST:"); Serial.print(dist);
    if(dist == 999) Serial.print(" [!] GHOST READ");
    Serial.print(" | LDR [F:"); Serial.print(f); Serial.print(" B:"); Serial.print(b);
    Serial.print(" L:"); Serial.print(l); Serial.print(" R:"); Serial.print(r);
    Serial.print("] | AVG:"); Serial.print(avgLight); 
    Serial.print(" BASE:"); Serial.println(baselineLight);

    // --- WAKE UP LOGIC ---
    if (hibernating) {
      if (avgLight >= baselineLight * 0.85) { 
        hibernating = false;
        Serial.println(" --- LIGHT RETURNED: WAKING UP ---");
      } else {
        lastActionTime = millis();
        return; 
      }
    }

    // --- SHADE DETECTION / SCOUT LOGIC ---
    if (avgLight < baselineLight * 0.8) {
      Serial.println(" --- SHADE DETECTED: INITIATING SCOUT ---");
      performDeepScout();
      lastActionTime = millis();
      return; 
    }

    // --- NORMAL AVOIDANCE & TRACKING ---
    if (dist <= wallDistance && dist > 0) {
      executeRecoil();
    } 
    else {
      // --- UPDATED: Vertical (Servo) Limit Check ---
      int vDiff = f - b;
      if (abs(vDiff) > globalThreshold) {
        
        // Figure out if we actually have physical room to move
        bool canMoveUp = (f > b) && (currentPos < maxServo);
        bool canMoveDown = (b > f) && (currentPos > minServo);

        // ONLY attach and command the servo if it is not at its limit
        if (canMoveUp || canMoveDown) {
          if (!myServo.attached()) {
            myServo.attach(servoPin);
            servoAttached = true;
          }
          
          if (canMoveUp) currentPos += minMoveStep;
          else if (canMoveDown) currentPos -= minMoveStep;
          
          myServo.write(currentPos);
          servoDetachTime = millis() + 150; 
        }
      }

      // Horizontal (Wheels)
      int hDiff = l - r;
      if (abs(hDiff) > globalThreshold) {
        int burst = map(abs(hDiff), globalThreshold, 700, minWheelBurst, 400);
        if (l > r) startMotorsNonBlocking(fullPower, -fullPower, burst); 
        else startMotorsNonBlocking(-fullPower, fullPower, burst);      
      }
    }
    lastActionTime = millis();
  }
}

// --- ASYMMETRIC SMOOTHING FUNCTION ---
float applySmoothing(int raw, float currentSmooth) {
  if (raw > currentSmooth) {
    return (raw * smoothUp) + (currentSmooth * (1.0 - smoothUp));
  } else {
    return (raw * smoothDown) + (currentSmooth * (1.0 - smoothDown));
  }
}

// --- DISTANCE LOGIC ---
long getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) {
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(200);
    pinMode(echoPin, INPUT);
    return 999;
  }
  return (duration * 0.034 / 2);        
}

// --- SCOUT LOGIC ---
void performDeepScout() {
  unsigned long scoutStartTime = millis();
  
  while (millis() - scoutStartTime < 60000) {
    int f = analogRead(ldrFront); int l = analogRead(ldrLeft);
    int r = analogRead(ldrRight); int b = analogRead(ldrBack);
    int currentAvg = (f + l + r + b) / 4;

    Serial.print("SCOUTING... CURRENT AVG: "); Serial.println(currentAvg);

    if (currentAvg >= baselineLight * 0.85) {
      Serial.println(" --- BETTER LIGHT FOUND! RESUMING ---");
      stopMotors();
      smoothF = f; smoothL = l; smoothR = r; smoothB = b;
      return; 
    }

    if (getDistance() < 15) {
      executeRecoil();
    } else {
      moveRobotBlocking(fullPower * 0.8, (random(0,2)==0 ? -fullPower * 0.8 : fullPower * 0.8), 400);
      delay(100); 
      moveRobotBlocking(fullPower, fullPower, 300); 
      delay(100);
    }
  }

  Serial.println(" --- SCOUT FAILED. ENTERING HIBERNATION. ---");
  stopMotors();
  hibernating = true;
}

// --- MOVEMENT FUNCTIONS ---
void startMotorsNonBlocking(int rSpd, int lSpd, int burst) {
  digitalWrite(AIN1, rSpd >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, rSpd >= 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(rSpd));
  digitalWrite(BIN1, lSpd >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, lSpd >= 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(lSpd));
  
  motorStopTime = millis() + burst;
  motorsRunning = true;
}

void moveRobotBlocking(int rSpd, int lSpd, int burst) {
  digitalWrite(AIN1, rSpd >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, rSpd >= 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(rSpd));
  digitalWrite(BIN1, lSpd >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, lSpd >= 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(lSpd));
  delay(burst);
  stopMotors();
}

void executeRecoil() {
  moveRobotBlocking(-fullPower, -fullPower, recoilTime);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}
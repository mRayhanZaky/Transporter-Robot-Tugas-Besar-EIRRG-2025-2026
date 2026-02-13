// Pin yang kepasang
#include <Servo.h>
#define ENR 3
#define ENL 11
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8
#define IRR A4
#define IRL A5
#define TRIG 2
#define ECHO 4
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define COLOR_OUT 12 

// State Robot
#define FOLLOW_LINE 0
#define PICK_OBJECT 1
#define PLACE_OBJECT 2
#define FINISH 3
#define PERSIMPANGAN 4
volatile int robotState = FOLLOW_LINE;
bool objectTaken = false;
bool missionComplete = false; 

// threshold IR
int threshold = 440; //440 400

// PID dan state persimpangan
float Kp = 1.0; // 1.2 1.4 1.1 1.0
float Ki = 0.0;
float Kd = 0.4; // 0.5 0.3 0.25 0.22
int baseSpeed = 165; // 145 140 150
float error = 0;
float lastError = 0;
float integral = 0;
int intersectionTimer = 0;
int intersectionCounter = 0;

// Hitung scan valid sensor ultrasonicnya
int ultraCount = 0;
// Feedback buat Gripper
#define MAX_GRIP_RETRY 3
int gripRetry = 0; 
bool bendaAman = false;

// Kasih nama untuk servo
Servo Gripper;
Servo Lifter;
long duration; 

// menyimpan nilai sensor warna saat mendeteksi zone
enum ZonaDrop {
  ZONA_NONE,
  ZONA_D,
  ZONA_C,
  ZONA_B,
  ZONA_A
}; 

// Kalibrasi sensor warna
#define R_MIN 90
#define R_MAX 450
#define G_MIN 100
#define G_MAX 480
#define B_MIN 80
#define B_MAX 420

// state sensor warna
bool allowDropScan = false;
bool allowFinishScan = false;

void setup() { 
  // Motor
  pinMode(ENR, OUTPUT);
  pinMode(ENL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 

  // Sensor Infra Red
  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT); 

  // Sensor ultrasonik
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT); 

  // Sensor Warna
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW); 

  // Gripper & Lifter
  Gripper.attach(9);
  Gripper.write(0);
  Lifter.attach(10);
  Lifter.write(50);
  Serial.begin(9600);
} 

void loop() {
Serial.println(robotState);
  if (robotState == FOLLOW_LINE) { 
    jalan();
  } else if (robotState == PERSIMPANGAN) {
    handlePersimpangan();
  } else if (robotState == PICK_OBJECT) { 
    ambilBenda(); 
  } else if (robotState == PLACE_OBJECT) { 
    taruhBenda(); 
  } else if (robotState == FINISH) { 
    stopMotor(); 
  }     
} 

void jalan() {
  int IRR_val = analogRead(IRR);
  int IRL_val = analogRead(IRL);
  int sensorKanan = (IRR_val < threshold) ? LOW : HIGH;
  int sensorKiri = (IRL_val < threshold) ? LOW : HIGH; 

  Serial.print("IRL: ");
  Serial.print(IRL_val);
  Serial.print(" IRR: ");
  Serial.println(IRR_val);
    
// deteksi perempatan
  if (sensorKiri == LOW && sensorKanan == LOW) {
    intersectionTimer++;
  } else {
    intersectionTimer = 0;
  }

  if (intersectionTimer > 25) { // 40
    stopMotor();
    robotState = PERSIMPANGAN;
    intersectionTimer = 0;
    return;
  }    
    
  error = map(IRL_val - IRR_val, -400, 400, -100, 100); // -600 600 -100 100, -400 400 -100 100
  integral += error;
  integral = constrain(integral, -1000, 1000);
  float derivative = (error - lastError) * 0.7; //peredam agar koreksi halus| 0.7  0.5
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -70, 70);
    
  int dynamicBase = baseSpeed - abs(error) * 0.01; // 0.05 0.03 0.02
  dynamicBase = constrain(dynamicBase, 140, baseSpeed); // 120 110 130

  int speedKiri = dynamicBase - correction;
  int speedKanan = dynamicBase + correction;

  speedKiri = constrain(speedKiri, 135, 235);// 0 60, 255 250, 70 230
  speedKanan = constrain(speedKanan, 135, 255); // 0 60, 255 250, 70 230 

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENL, speedKiri);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENR, speedKanan); 

lastError = error; 

// Deteksi Pick Up Zone (Ultrasonik cuma kerja di bagian ini)
  if (robotState == FOLLOW_LINE && missionComplete == false) {
    long jarak = calculateDistance(); 
      if (jarak < 3 && objectTaken == false) { 
        stopMotor(); 
        robotState = PICK_OBJECT;
        return;
      } 
  } 

  if (missionComplete == true && robotState == FOLLOW_LINE) {
    deteksiFinish();
  }

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Kiri: ");
  Serial.print(speedKiri);
  Serial.print(" | Kanan: ");
  Serial.println(speedKanan);
}

void mundur() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(ENL, 110);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENR, 110);
  delay(400);
} 

void belokKiri() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENL, 155);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENR, 155);
  delay(900);
} 

void belokKanan() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(ENL, 100);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENR, 100);
  delay(400);
} 

void handlePersimpangan() {

  stopMotor();
  delay(100);

  // maju sedikit ke tengah perempatan
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENL, 100);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENR, 100);
  delay(200);

  stopMotor();
  delay(100);

  if (intersectionCounter < 3) {
    intersectionCounter++;
  }

  if (intersectionCounter == 1) {
    Serial.println("Lurus ambil balok");
  } else if (intersectionCounter == 2) {
    belokKanan();
    allowDropScan = true;
  } else if (intersectionCounter == 3) {
    Serial.println("Lurus ke finish");
    allowDropScan = false;
    allowFinishScan = true;
  }
  robotState = FOLLOW_LINE;
}

void ambilBenda() {
  // stopMotor();
  long benda = calculateDistance();
  Serial.print("Jarak Benda: ");
  Serial.println(benda);

  if (benda < 3) {
    ultraCount++; 
  } else {
    ultraCount = 0; 
  }
    
  if (ultraCount >= 3 && objectTaken == false) {
    tutup(); 
    objectTaken = true; 
    ultraCount = 0; 
    gripRetry = 0; 
    angkat(); 
    delay(1000); 
    belokKiri();
    //stopMotor();
    robotState = PLACE_OBJECT; 
    Serial.println("GRIP SUCCESS"); 
}
    
} 

void taruhBenda() {
  if (!cekBenda()) {
    bendaAman = false;
    robotState = PICK_OBJECT;
    return;
  }

  if (!allowDropScan) {
    jalan();
    return;
  }
// validasi zona drop
  ZonaDrop zona = deteksiZonaWarna();
    
  if (zona == ZONA_C && objectTaken == true && missionComplete == false) {
// Zona drop terdeteksi
  stopMotor();
  Serial.println("DROP ZONE DETECTED");
  turun();
  buka();
  objectTaken = false;
  missionComplete = true;
  Serial.println("DROP SUCCESS");
  belokKiri();
  stopMotor();
  robotState = FOLLOW_LINE;
  return;
  }
  jalan();
} 

bool cekBenda() {
  long cek = calculateDistance();
  if (cek <= 3) {
    bendaAman = true;
  }
  return bendaAman;
}

void deteksiFinish() {
  if (!allowFinishScan) {
    return;
  }
  
  ZonaDrop finish = deteksiZonaWarna();  
  if (finish == ZONA_A) {
    robotState = FINISH; 
    Serial.println("WELL PLAYED!"); 
  }  
} 

long calculateDistance() {
  digitalWrite(TRIG, 0);
  delayMicroseconds(2);
  digitalWrite(TRIG, 1);
  delayMicroseconds(10);
  digitalWrite(TRIG, 0);
  duration = pulseIn(ECHO, 1, 25000);

  if (duration == 0) {
    return 999; 
  }
  
  return duration * 0.034 / 2;
} 

int readColor(char color) {
  if (color == 'R') {
    digitalWrite(S2, LOW); 
    digitalWrite(S3, LOW); 
  } else if (color == 'G') {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH); 
  } else if (color == 'B') {
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH); 
  }
    delay(3);
    return pulseIn(COLOR_OUT, LOW);
  } 

int kalibrasi(int val, int vMin, int vMax) {
  val = constrain(val, vMin, vMax);
  return map(val, vMin, vMax, 255, 0); // 255 - 0 karena semakin kecil semakin akurat
} 

ZonaDrop deteksiZonaWarna() {
  int rRaw = readColor('R');
  int R = kalibrasi(rRaw, R_MIN, R_MAX);
  delay(5);
      
  int gRaw = readColor('G');
  int G = kalibrasi(gRaw, G_MIN, G_MAX);
  delay(5);
      
  int bRaw = readColor('B');
  int B = kalibrasi(bRaw, B_MIN, B_MAX);
  delay(5);
      
  Serial.print("R: "); Serial.print(R);
  Serial.print(", G: "); Serial.print(G);
  Serial.print(", B: "); Serial.println(B);

  if (B > R + 50 && B > G + 50) { // 40 adalah jarak aman margin
    return ZONA_D; 
  } else if (R > 60 && R > G && R > B) {
    return ZONA_C; 
  } else if (G > R + 50 && G > B + 50) {
    return ZONA_B; 
  } else if ((abs(R - G) < 20 && abs(G - B) < 20) && (R > 200 && G > 200 && B > 200)) { // abs adalah syntax absolute value
    return ZONA_A; 
  }
  
  return ZONA_NONE;
} 

// pergerakan servo
void tutup() {
  Gripper.write(40);
  delay(1000);
}

void buka() {
  Gripper.write(0);
  delay(1000);
}

void angkat(){
  for (int i = 50; i <= 180; i++) {
    Lifter.write(i); delay(15); 
  }
}

void turun() {
  for (int i = 180; i >= 50; i--) {
    Lifter.write(i); delay(15); 
  }
} 

void stopMotor() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  analogWrite(ENL, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  analogWrite(ENR, 0);
}

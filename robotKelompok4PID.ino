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
int threshold = 440; 

// PID
float Kp = 1.2; // koreksi berdasarkan error
float Ki = 0.0; // membantu koreksi error kecil yang menumpuk
float Kd = 0.5; // menghaluskan gerakan koreksi
int baseSpeed = 120; // pwm utama
float error = 0; // posisi robot saat ini
float lastError = 0; // posisi terakhir sebelum posisi saat ini
float integral = 0; // menyimpan akumulasi error

// Hitung scan valid sensor ultrasonicnya
int ultraCount = 0;
// Feedback buat Gripper
#define MAX_GRIP_RETRY 3
int gripRetry = 0; 

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
bool allowDrop = false;
int persimpanganCount = 0;
unsigned long dropDelayStart = 0;
bool sudahBelokDrop = false;
unsigned long intersectionStart = 0;

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
  jalan();
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
    
// ? adalah ternary operator
// nilai sensor < threshold = LOW
// nilai sensor > threshold = HIGH

Serial.print("IRL: ");
Serial.print(IRL_val);
Serial.print(" IRR: ");
Serial.println(IRR_val);

error = map(IRL_val - IRR_val, -600, 600, -100, 100); // membandingkan sensor kiri kanan dengan nilai map agar tidak terlalu besar dan perhitungan PID stabil
    
integral += error; // menyimpan error yang terjadi terus terusan
    
integral = constrain(integral, -1000, 1000); // batasan akumulasi error agar tidak terlalu besar
    
float derivative = error - lastError; // ukur seberapa cepat error berubah, lalu koreksi secara halus agar tidak overshoot
    
float correction = (Kp * error) + (Ki * integral) + (Kd * derivative); // hasil akhir PID (Kp = koreksi posisi sejarang, Ki = koreksi error menumpuk, Kd = meredam gerakan)
    
int dynamicBase = baseSpeed - abs(error) * 0.05; // saat belok tajam robot akan melambat 
    
dynamicBase = constrain(dynamicBase, 80, baseSpeed); // batasan agar tidak terlalu pelan saat belok
    
correction = constrain(correction, -60, 60); // batasan koreksi PID agar tidak terlalu besar dan tidak terlalu kecil

// mengubah koreksi jadi pwm roda
int speedKiri = dynamicBase - correction;
int speedKanan = dynamicBase + correction;

// batasan agar roda tidak terlalu pelan dan tidak terlalu cepat
speedKiri = constrain(speedKiri, 50, 200);
speedKanan = constrain(speedKanan, 50, 200); 

bool duaSensorPutih = (sensorKiri == HIGH && sensorKanan == HIGH);

if (duaSensorPutih && abs(error) > 30) {

  if (lastError > 0) {
    // cari garis ke kanan
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    analogWrite(ENL,130);

    digitalWrite(IN3,1);
    digitalWrite(IN4,0);
    analogWrite(ENR,130);

  } else {
    // cari garis ke kiri
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    analogWrite(ENL,130);

    digitalWrite(IN3,0);
    digitalWrite(IN4,1);
    analogWrite(ENR,130);
  }
  lastError = error;
  return;
}
    
// deteksi perempatan
bool duaSensorHitam = (sensorKiri == LOW && sensorKanan == LOW);

if (duaSensorHitam) {

  // mulai timer saat pertama kali hitam
  if (intersectionStart == 0) {
    intersectionStart = millis();
  }

  // kalau hitam lebih dari 200ms = persimpangan
  if (millis() - intersectionStart > 200 && abs(error) < 35) {
    stopMotor();
    robotState = PERSIMPANGAN;
    intersectionStart = 0;
    return;
  }

} else {
  // reset kalau keluar dari hitam
  intersectionStart = 0;
}

// gerakan dan kecepatan motor
digitalWrite(IN1, 0);
digitalWrite(IN2, 1);
analogWrite(ENL, speedKiri);
digitalWrite(IN3, 0);
digitalWrite(IN4, 1);
analogWrite(ENR, speedKanan); 

lastError = error; 

// Deteksi Pick Up Zone (Ultrasonik cuma kerja di bagian ini)
if (robotState == FOLLOW_LINE && missionComplete == false && objectTaken == false) {
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
  delay(250);
  stopMotor();
  delay(50);
} 

void belokKiri() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENL, 200);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENR, 200);
  delay(950);
  
  stopMotor();
  delay(100);
} 

void belokKanan() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(ENL, 150);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENR, 150);
  delay(500);

  stopMotor();
  delay(100);
} 

void handlePersimpangan() {
  persimpanganCount++;
  Serial.print("Persimpangan ke: ");
  Serial.println(persimpanganCount);

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

  if(objectTaken && !missionComplete) {
    if (persimpanganCount == 2 && !sudahBelokDrop) {
      belokKanan();
      intersectionStart = 0;
      allowDrop = true;
      sudahBelokDrop = true;
      dropDelayStart = millis();
    }
  }

  if (objectTaken && !missionComplete) {
    robotState = PLACE_OBJECT;
  } else {
    robotState = FOLLOW_LINE;
  }
}

void ambilBenda() {
long benda = calculateDistance();
Serial.print("Jarak Benda: ");
Serial.println(benda);
    
if (benda < 3) {
ultraCount++; 
} else {
ultraCount = 0; 
}
    
if (ultraCount >= 3 && objectTaken == false) {
stopMotor();
tutup(); 
objectTaken = true; 
ultraCount = 0; 
gripRetry = 0; 
angkat(); 
belokKiri(); 
allowDrop = false;
sudahBelokDrop = false;
robotState = PLACE_OBJECT; 
Serial.println("GRIP SUCCESS"); 
}
    
} 

void taruhBenda() {
if (!allowDrop) {
  return;  // belum boleh drop
}

if (millis() - dropDelayStart < 60000) {
  return;
}

// validasi zona drop
ZonaDrop zona = deteksiZonaWarna();
if (zona == ZONA_NONE) {
return; 
}
    
// Zona drop terdeteksi
if (zona == ZONA_C && objectTaken == true && missionComplete == false) {
stopMotor();
Serial.println("DROP ZONE DETECTED");
turun();
buka();
objectTaken = false;
missionComplete = true;
allowDrop = false;
persimpanganCount = 0;
Serial.println("DROP SUCCESS");
mundur();
belokKiri();
robotState = FOLLOW_LINE;
} 
}

void deteksiFinish() {
ZonaDrop finish = deteksiZonaWarna();
    
if (finish == ZONA_NONE) {
return; 
} else if (finish == ZONA_A) {
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

  if (B > R + 50 && B > G + 50) {
     // dominan biru maka warna biru
    return ZONA_D; 
  } else if (R > 120 && R > G + 40 && R > B + 40) {
    // dominan merah maka warna merah
    return ZONA_C; 
  } else if (G > R + 50 && G > B + 50) {
    // dominan hijau maka warna hijau
    return ZONA_B; 
  } else if ((abs(R - G) < 20 && abs(G - B) < 20) && (R > 200 && G > 200 && B > 200)) {
    // R, G, B hampir sama maka warna putih
    return ZONA_A; 
  }
  // tidak ada yang cocok, bukan zona warna
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
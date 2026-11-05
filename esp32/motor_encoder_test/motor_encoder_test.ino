// ESP32-S3 모터 + 엔코더 테스트 펌웨어 (순찰 로봇)
//
// 보드 : ESP32-S3-DevKitC-1U-N8R8  (Arduino IDE 보드 선택: "ESP32S3 Dev Module")
// 통신 : USB 케이블을 보드의 "UART" 포트에 꽂고 115200 baud
//
// 배선
//   MDD3A 입력핀   M1A <- GPIO4,  M1B <- GPIO5   (좌측 모터)
//                  M2A <- GPIO6,  M2B <- GPIO7   (우측 모터)
//                  GND <- ESP32 G               (공통 GND, 꼭 연결)
//   좌측 엔코더    V -> 3V3, C -> G, A -> GPIO9,  B -> GPIO10
//   우측 엔코더    V -> 3V3, C -> G, A -> GPIO11, B -> GPIO12
//   (엔코더 V 는 반드시 3V3. 5V 나 배터리에 꽂지 말 것)
//
// 프로토콜 (한 줄 = 한 메시지, 끝은 '\n', 꼬리표 = '*' 뒤 XOR 체크섬 2자리 16진수)
//   라즈베리파이 -> ESP32
//     $M,<좌>,<우>*HH   모터 명령. 좌/우 = -255 ~ 255 (음수 = 역회전)
//     $R*HH             엔코더 카운트 0 으로 초기화
//   ESP32 -> 라즈베리파이 (50ms 마다)
//     $E,<ms>,<좌카운트>,<우카운트>,<좌PWM>,<우PWM>,<워치독>,<체크섬오류수>*HH
//       워치독 = 1 이면 300ms 동안 정상 명령이 없어서 모터를 멈춘 상태
//     $ERR,<이유>*HH    체크섬이 틀리거나 이해 못 한 명령
//
// 안전장치 (보고서 요구사항)
//   1. 체크섬이 맞지 않는 명령은 버림 (데이터 깨짐으로 인한 급발진 방지)
//   2. 300ms 동안 정상 명령이 없으면 스스로 모터를 정지

#include <Arduino.h>

// ---------- 핀 ----------
const int PIN_L_A = 4;    // MDD3A M1A
const int PIN_L_B = 5;    // MDD3A M1B
const int PIN_R_A = 6;    // MDD3A M2A
const int PIN_R_B = 7;    // MDD3A M2B

const int ENC_L_A = 9;
const int ENC_L_B = 10;
const int ENC_R_A = 11;
const int ENC_R_B = 12;

// ---------- 설정 ----------
const unsigned long WATCHDOG_MS = 300;   // 명령이 끊기면 정지하는 시간
const unsigned long STATUS_MS   = 50;    // 상태 보고 주기
const size_t RX_LINE_MAX = 64;

// ---------- 상태 ----------
volatile int32_t encLeft = 0;
volatile int32_t encRight = 0;

int pwmLeft = 0;
int pwmRight = 0;
bool watchdogStopped = true;             // 첫 명령 전에는 정지 상태
unsigned long lastCommandMs = 0;
unsigned long lastStatusMs = 0;
uint32_t checksumErrors = 0;

char lineBuf[RX_LINE_MAX];
size_t lineLen = 0;

// ---------- 엔코더 인터럽트 (A상 변화마다 B상으로 방향 판단, 2체배) ----------
void IRAM_ATTR isrLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) encLeft--; else encLeft++;
}

void IRAM_ATTR isrRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) encRight--; else encRight++;
}

// ---------- 모터 ----------
void setMotor(int pinA, int pinB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pinA, speed);
    analogWrite(pinB, 0);
  } else if (speed < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -speed);
  } else {                               // A, B 둘 다 LOW = MDD3A 브레이크
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void drive(int left, int right) {
  pwmLeft = constrain(left, -255, 255);
  pwmRight = constrain(right, -255, 255);
  setMotor(PIN_L_A, PIN_L_B, pwmLeft);
  setMotor(PIN_R_A, PIN_R_B, pwmRight);
}

// ---------- 체크섬 ----------
uint8_t xorChecksum(const char* s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

void sendLine(const char* body) {        // body 예: "E,123,0,0,0,0,1,0"
  char out[96];
  snprintf(out, sizeof(out), "$%s*%02X", body, xorChecksum(body, strlen(body)));
  Serial.println(out);
}

// ---------- 명령 처리 ----------
void handleLine(char* line) {
  // 형식: $BODY*HH
  if (line[0] != '$') return;
  char* star = strrchr(line, '*');
  if (star == nullptr || strlen(star) < 3) {
    checksumErrors++;
    sendLine("ERR,NO_CHECKSUM");
    return;
  }
  char* body = line + 1;
  size_t bodyLen = (size_t)(star - body);
  uint8_t expected = (uint8_t)strtoul(star + 1, nullptr, 16);
  if (xorChecksum(body, bodyLen) != expected) {
    checksumErrors++;
    sendLine("ERR,CHECKSUM");
    return;
  }
  *star = '\0';

  int left = 0, right = 0;
  if (sscanf(body, "M,%d,%d", &left, &right) == 2) {
    drive(left, right);
    lastCommandMs = millis();
    watchdogStopped = false;
  } else if (strcmp(body, "R") == 0) {
    encLeft = 0;
    encRight = 0;
  } else {
    sendLine("ERR,UNKNOWN");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < RX_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                       // 너무 긴 줄은 버림
    }
  }
}

void setup() {
  Serial.begin(115200);

  int motorPins[] = {PIN_L_A, PIN_L_B, PIN_R_A, PIN_R_B};
  for (int p : motorPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  drive(0, 0);

  int encPins[] = {ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B};
  for (int p : encPins) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);

  delay(200);
  sendLine("INFO,READY");
}

void loop() {
  readSerial();

  unsigned long now = millis();

  // 300ms 동안 정상 명령이 없으면 정지
  if (!watchdogStopped && now - lastCommandMs > WATCHDOG_MS) {
    drive(0, 0);
    watchdogStopped = true;
  }

  if (now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    char body[80];
    snprintf(body, sizeof(body), "E,%lu,%ld,%ld,%d,%d,%d,%lu",
             now, (long)encLeft, (long)encRight, pwmLeft, pwmRight,
             watchdogStopped ? 1 : 0, (unsigned long)checksumErrors);
    sendLine(body);
  }
}

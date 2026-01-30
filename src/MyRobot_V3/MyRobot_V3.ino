#include <WiFiS3.h>

char ssid[] = "SHAW-F4E1";
char pass[] = "charge8277enter";

WiFiServer server(5200);

// ===== L298N pins (match your wiring) =====
const int ENA = 9;
const int ENB = 10;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 6;
const int IN4 = 5;

int speedVal = 150;

unsigned long lastCmdTime = 0;
const unsigned long timeoutMs = 300; // dead-man stop

// ===== Encoder pins (match your build: D12/D13) =====
const int ENC_L = 12;   // left LM393 D0 -> D12
const int ENC_R = 13;   // right LM393 D0 -> D13

// Tick counters + last states (simple polling edge-detect like a "button")
volatile long ticksL = 0;
volatile long ticksR = 0;

int lastEncL = HIGH;
int lastEncR = HIGH;

// ===== Telemetry timing =====
unsigned long lastPrintMs = 0;
const unsigned long printEveryMs = 20; // ~50 Hz logging (adjust)

// Optional: print raw encoder states too
void printTelemetry(int encL, int encR) {
  // CSV-like: time_ms,encL,encR,ticksL,ticksR,lastCmdAge_ms
  unsigned long now = millis();
  Serial.print(now); Serial.print(',');
  Serial.print(encL); Serial.print(',');
  Serial.print(encR); Serial.print(',');
  Serial.print(ticksL); Serial.print(',');
  Serial.print(ticksR); Serial.print(',');
  Serial.println(now - lastCmdTime);
}

void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  // Encoders: use pullups, same idea as the lab example  [oai_citation:3‡Lab 04. Distance Measurement.pdf](sediment://file_00000000704471f5829a997e872b6c62)
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  Serial.begin(115200);

  // Print a header once so logs are self-describing
  Serial.println("t_ms,encL,encR,ticksL,ticksR,lastCmdAge_ms");

  WiFi.begin(ssid, pass);
  delay(2000);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected!");
  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Server started on port 5200");

  lastCmdTime = millis(); // initialize
}

void loop() {
  // --- Always read encoders + update tick counts (even if no client) ---
  int encL = digitalRead(ENC_L);
  int encR = digitalRead(ENC_R);

  // Count rising edges (LOW -> HIGH) to avoid double counting
  if (lastEncL == LOW && encL == HIGH) ticksL++;
  if (lastEncR == LOW && encR == HIGH) ticksR++;
  lastEncL = encL;
  lastEncR = encR;

  // Print telemetry at a steady rate (lab wants nice delimited output)  [oai_citation:4‡Lab 04. Distance Measurement.pdf](sediment://file_00000000704471f5829a997e872b6c62)
  unsigned long now = millis();
  if (now - lastPrintMs >= printEveryMs) {
    lastPrintMs = now;
    printTelemetry(encL, encR);
  }

  // --- WiFi client handling (unchanged logic, but non-blocking style) ---
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      // Keep updating encoders and printing telemetry while connected too
      encL = digitalRead(ENC_L);
      encR = digitalRead(ENC_R);

      if (lastEncL == LOW && encL == HIGH) ticksL++;
      if (lastEncR == LOW && encR == HIGH) ticksR++;
      lastEncL = encL;
      lastEncR = encR;

      now = millis();
      if (now - lastPrintMs >= printEveryMs) {
        lastPrintMs = now;
        printTelemetry(encL, encR);
      }

      if (client.available()) {
        char cmd = client.read();
        handleCommand(cmd);
        lastCmdTime = millis();
      }

      if (millis() - lastCmdTime > timeoutMs) {
        stopMotors();
      }
    }

    client.stop();
    stopMotors();
    Serial.println("Client disconnected");
  } else {
    // If no client is connected, still enforce dead-man stop.
    // This prevents the robot from running forever if a client disconnects abruptly.
    if (millis() - lastCmdTime > timeoutMs) {
      stopMotors();
    }
  }
}

void handleCommand(char cmd) {
  Serial.print("CMD,"); Serial.println(cmd);

  switch (cmd) {
    case 'U': forward(); break;
    case 'D': backward(); break;
    case 'L': left(); break;
    case 'R': right(); break;
    case 'S': stopMotors(); break;
  }
}

// Motor commands (unchanged)
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void left() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
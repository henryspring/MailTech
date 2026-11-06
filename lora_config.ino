// lora_config.ino - one-time setup for an E32-900T20D. Flash this on EACH ESP32,
// open the serial monitor @115200, and read the output. Then flash the real sketch.
//
// Sets: address 0x0000, channel MY_CHAN, 8N1 @ 9600 UART, 2.4 kbps air rate, 20 dBm,
//       transparent mode. Saved to the module's flash (command 0xC0).
// The module is in sleep/config mode (M0=M1=HIGH) the whole time, so it does not transmit.
//
// Both modules MUST get identical settings.

#define LORA_RX_PIN 1    // ESP32 RX <- module TXD
#define LORA_TX_PIN 2    // ESP32 TX -> module RXD
#define M0_PIN      14
#define M1_PIN      21
#define AUX_PIN     42

// ---- settings to write ----
#define MY_ADDH     0x00
#define MY_ADDL     0x00
#define MY_CHAN     60       // see the printed frequencies below; must be inside 902-928 MHz
#define SPED_BYTE   0x1C     // 00 011 010 = 8N1, 9600 baud, 2.4 kbps air
#define OPTION_BYTE 0x44     // transparent, push-pull, 250 ms wake, FEC on, 20 dBm

static void waitAux() {
  uint32_t t0 = millis();
  while (digitalRead(AUX_PIN) == LOW && millis() - t0 < 3000) delay(1);
  delay(20);
}

static void printHex(const char *label, const uint8_t *b, size_t n) {
  Serial.print(label);
  for (size_t i = 0; i < n; i++) Serial.printf(" %02X", b[i]);
  Serial.println();
}

static size_t txrx(const uint8_t *cmd, size_t n, uint8_t *resp, size_t rn) {
  while (Serial1.available()) Serial1.read();
  Serial1.write(cmd, n);
  Serial1.flush();
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < rn && millis() - t0 < 1500) {
    if (Serial1.available()) resp[got++] = Serial1.read();
  }
  return got;
}

static void printParams(const uint8_t *p) {   // p = C0 ADDH ADDL SPED CHAN OPTION
  uint8_t chan = p[4] & 0x7F;
  Serial.printf("  address: 0x%02X%02X\n", p[1], p[2]);
  Serial.printf("  SPED=0x%02X  OPTION=0x%02X\n", p[3], p[5]);
  Serial.printf("  channel: %u\n", chan);
  Serial.printf("  frequency if base=862 MHz   : %u MHz\n", 862 + chan);
  Serial.printf("  frequency if base=850.125   : %.3f MHz\n", 850.125 + chan);
  Serial.println("  (check your module's manual; US legal band is 902-928 MHz)");
}

static bool readParams(uint8_t *p) {
  const uint8_t cmd[3] = {0xC1, 0xC1, 0xC1};
  return txrx(cmd, 3, p, 6) == 6;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== E32 config ===");

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);
  digitalWrite(M0_PIN, HIGH);   // M0=M1=HIGH: sleep/config mode
  digitalWrite(M1_PIN, HIGH);
  delay(200);
  waitAux();

  Serial1.setRxBufferSize(256);
  Serial1.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);   // config mode is always 9600 8N1

  uint8_t buf[8];

  const uint8_t verCmd[3] = {0xC3, 0xC3, 0xC3};
  if (txrx(verCmd, 3, buf, 4) == 4) {
    printHex("Version bytes (C3 freq-code version features):", buf, 4);
    Serial.println("  (freq-code 0x32 = 868 MHz family, 0x38 = 915 MHz family on some variants.");
    Serial.println("   If it is locked to 868, the module cannot be retuned into the US band.)");
  } else {
    Serial.println("No reply to version query. Check wiring, M0/M1, and power.");
    return;
  }

  uint8_t cur[8];
  if (!readParams(cur)) { Serial.println("Could not read parameters"); return; }
  printHex("Current params:", cur, 6);
  printParams(cur);

  Serial.println("\nWriting new parameters...");
  const uint8_t wr[6] = {0xC0, MY_ADDH, MY_ADDL, SPED_BYTE, (uint8_t)MY_CHAN, OPTION_BYTE};
  txrx(wr, 6, buf, 6);
  delay(500);
  waitAux();

  uint8_t after[8];
  if (readParams(after)) {
    printHex("Params after write:", after, 6);
    printParams(after);
    bool ok = memcmp(wr + 1, after + 1, 5) == 0;
    Serial.println(ok ? "\nSUCCESS: settings saved." : "\nMISMATCH: settings did not stick.");
  } else {
    Serial.println("Could not read back parameters");
  }

  digitalWrite(M0_PIN, LOW);    // back to normal mode
  digitalWrite(M1_PIN, LOW);
  Serial.println("Module returned to normal mode. Flash the sender/receiver sketch next.");
}

void loop() {}

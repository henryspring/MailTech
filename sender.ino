
// Same on-air protocol as the original ESP32 sender. The ESP32 receiver needs no
// protocol changes. An actual camera is NOT implemented in this sketch.
// To send data: wait until paired, then use send_image.py over the board's
// ST-LINK virtual COM port (115200) to issue SEND <size> and stream the bytes.
// Type 's' in the Serial Monitor (newline) to send the current 5000-byte test
// pattern or resend the most recently loaded image.
//
// NUCLEO-U575ZI-Q wiring (example; adjust pins if using a custom PCB):
//   E32 TXD -> STM32 PD6 (USART2 RX, Arduino D52)
//   E32 RXD <- STM32 PD5 (USART2 TX, Arduino D53)
//   E32 AUX -> STM32 PC0 (Arduino A5)
//   E32 M0  <- STM32 PC1 (Arduino A4)
//   E32 M1  <- STM32 PC2 (board Morpho header)
//   E32 GND <-> STM32 GND; supply E32 with an adequately rated supply.
// PC USB serial: ST-LINK VCP (USART1 PA9/PA10), NOT the LoRa UART.

#include <Arduino.h>
#include <stdlib.h>

#if !defined(ARDUINO_ARCH_STM32)
#error "This sketch requires the STM32duino Arduino core."
#endif

// ===== protocol =====
// Frame: | SYNC | FLAGS | IMG_ID | SEQ(2,BE) | LEN | PAYLOAD (0-48) | CRC16(2,BE) |
// CRC16-CCITT (0x1021, init 0xFFFF) over FLAGS..PAYLOAD.
#ifndef LORA_PROTO_H
#define LORA_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define SYNC_BYTE    0xAA
#define F_SYN        0x01
#define F_ACK        0x02
#define F_END        0x04
#define F_REQ        0x08   // last packet in window, please ACK now
#define F_RST        0x10
#define F_HELLO      0x20   // pairing

#define MAX_PAYLOAD  48
#define HDR_LEN      6
#define MAX_FRAME    (HDR_LEN + MAX_PAYLOAD + 2)
#define WINDOW       32     // packets per window (bitmap = 32 bits)

typedef struct {
    uint8_t  flags;
    uint8_t  id;
    uint16_t seq;
    uint8_t  len;
    uint8_t  payload[MAX_PAYLOAD];
} frame_t;

typedef struct {
    uint8_t buf[MAX_FRAME];
    uint8_t n;
} parser_t;

static inline uint16_t crc16(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++)
            c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
    }
    return c;
}

static inline uint32_t crc32(const uint8_t *d, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *d++;
        for (int i = 0; i < 8; i++)
            c = (c & 1) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    }
    return ~c;
}

static inline void put32(uint8_t *p, uint32_t v) {
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}
static inline uint32_t get32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static inline size_t frame_build(uint8_t *out, uint8_t flags, uint8_t id,
                                 uint16_t seq, const uint8_t *pl, uint8_t len) {
    out[0] = SYNC_BYTE;
    out[1] = flags;
    out[2] = id;
    out[3] = seq >> 8;
    out[4] = seq & 0xFF;
    out[5] = len;
    if (len) memcpy(out + HDR_LEN, pl, len);
    uint16_t c = crc16(out + 1, 5 + len);
    out[HDR_LEN + len]     = c >> 8;
    out[HDR_LEN + len + 1] = c & 0xFF;
    return HDR_LEN + len + 2;
}

// Feed one received byte. Returns 1 when a CRC-valid frame is in *f.
static inline int parser_feed(parser_t *p, uint8_t b, frame_t *f) {
    if (p->n == 0 && b != SYNC_BYTE) return 0;
    p->buf[p->n++] = b;
    if (p->n < HDR_LEN) return 0;
    uint8_t len = p->buf[5];
    if (len > MAX_PAYLOAD) { p->n = 0; return 0; }
    uint8_t need = HDR_LEN + len + 2;
    if (p->n < need) return 0;
    p->n = 0;
    uint16_t rx = ((uint16_t)p->buf[need - 2] << 8) | p->buf[need - 1];
    if (crc16(p->buf + 1, need - 3) != rx) return 0;
    f->flags = p->buf[1];
    f->id    = p->buf[2];
    f->seq   = ((uint16_t)p->buf[3] << 8) | p->buf[4];
    f->len   = len;
    memcpy(f->payload, p->buf + HDR_LEN, len);
    return 1;
}

#endif

// ===== end protocol =====

#define AUX_PIN PC0  // E32 AUX (output from radio)
#define M0_PIN  PC1  // E32 mode control 0
#define M1_PIN  PC2  // E32 mode control 1
HardwareSerial LoRaSerial(PD_6, PD_5); // USART2: RX (E32 TXD), TX (E32 RXD)

#define TEST_IMAGE_SIZE 5000  // bytes
#define ACK_TIMEOUT_MS  6000 
#define MAX_TRIES       8

static parser_t parser;
static uint8_t *img = nullptr;
static uint32_t imgSize = 0;
static uint16_t pktCount = 0;
static uint8_t  imgId = 0;
static bool paired = false;
static bool sendPending = false;

// ---------- low-level ----------
static void waitAux() {
  uint32_t t0 = millis();
  while (digitalRead(AUX_PIN) == LOW && millis() - t0 < 2000) delay(1);
  delay(3);
}

static void sendFrame(uint8_t flags, uint8_t id, uint16_t seq, const uint8_t *pl, uint8_t len) {
  uint8_t out[MAX_FRAME];
  size_t n = frame_build(out, flags, id, seq, pl, len);
  waitAux();
  LoRaSerial.write(out, n);   // one contiguous burst
  LoRaSerial.flush();
  delay(5);                // let AUX drop
  waitAux();               // wait until it's fully on the air
}

static bool waitFrame(frame_t &f, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (LoRaSerial.available()) {
      if (parser_feed(&parser, LoRaSerial.read(), &f)) return true;
    }
    delay(1);
  }
  return false;
}

// ---------- pairing ----------
static void tryPair() {
  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  last = millis();
  sendFrame(F_HELLO, 0, 0, nullptr, 0);
  frame_t f;
  if (waitFrame(f, 2500) && f.flags == (F_HELLO | F_ACK)) {
    paired = true;
    // No auto-send; PC can push an image or you can send the test pattern with 's'.
    Serial.println("*** PAIRED with receiver ***");
  } else {
    Serial.println("Searching for receiver...");
  }
}

// ---------- image transfer ----------
static void sendData(uint16_t seq, bool req) {
  uint32_t off = (uint32_t)seq * MAX_PAYLOAD;
  uint32_t rem = imgSize - off;
  uint8_t len = rem > MAX_PAYLOAD ? MAX_PAYLOAD : (uint8_t)rem;
  uint8_t flags = 0;
  if (seq == pktCount - 1) flags |= F_END;
  if (req)                 flags |= F_REQ;
  sendFrame(flags, imgId, seq, img + off, len);
}

// Send every packet in the window whose bit is NOT set in `have`.
// The highest missing one carries REQ so the receiver knows to ACK.
static void sendMissing(uint16_t base, uint8_t n, uint32_t have) {
  int last = -1;
  for (int i = 0; i < n; i++) if (!((have >> i) & 1)) last = i;
  for (int i = 0; i < n; i++)
    if (!((have >> i) & 1)) sendData(base + i, i == last);
}

// returns 0 = receiver confirmed full image OK, <0 = failure
static int sendImage() {
  pktCount = (imgSize + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
  imgId++;
  uint32_t t0 = millis();
  frame_t f;

  Serial.printf("Sending image id=%u: %lu bytes, %u packets\n",
                imgId, (unsigned long)imgSize, pktCount);

  // ---- handshake: SYN (size, count, crc32) -> SYN|ACK ----
  uint8_t syn[10];
  put32(syn, imgSize);
  syn[4] = pktCount >> 8;
  syn[5] = pktCount & 0xFF;
  put32(syn + 6, crc32(img, imgSize));
  bool ok = false;
  for (int t = 0; t < 10 && !ok; t++) {
    sendFrame(F_SYN, imgId, 0, syn, sizeof syn);
    if (waitFrame(f, ACK_TIMEOUT_MS) && f.id == imgId && f.flags == (F_SYN | F_ACK)) ok = true;
  }
  if (!ok) { Serial.println("No SYN|ACK from receiver"); return -1; }

  // ---- windows ----
  for (uint16_t base = 0; base < pktCount; base += WINDOW) {
    uint8_t n = (pktCount - base < WINDOW) ? (uint8_t)(pktCount - base) : WINDOW;
    uint32_t full = (n == 32) ? 0xFFFFFFFFu : ((1u << n) - 1);
    uint32_t have = 0;
    bool done = false;

    sendMissing(base, n, have); // first burst: everything

    for (int t = 0; t < MAX_TRIES; t++) {
      if (waitFrame(f, ACK_TIMEOUT_MS) && f.id == imgId &&
          (f.flags == F_ACK) && f.seq == base && f.len == 4) {
        have = get32(f.payload);
        if ((have & full) == full) { done = true; break; }
        sendMissing(base, n, have);
      } else {
        sendData(base + n - 1, true); // poke: resend last packet with REQ
      }
    }
    if (!done) { Serial.printf("Window at %u failed\n", base); return -2; }
    Serial.printf("  window %u/%u acked\n", base / WINDOW + 1, (pktCount + WINDOW - 1) / WINDOW);
  }

  // ---- wait for final verdict (ACK|END, payload[0] = 0 OK / 1 CRC fail) ----
  for (int t = 0; t < MAX_TRIES; ) {
    if (waitFrame(f, ACK_TIMEOUT_MS)) {
      if (f.id == imgId && f.flags == (F_ACK | F_END) && f.len == 1) {
        if (f.payload[0] == 0) {
          Serial.printf("TRANSFER COMPLETE: receiver confirmed image OK (%lu bytes, %.1f s)\n",
                        (unsigned long)imgSize, (millis() - t0) / 1000.0);
          return 0;
        }
        Serial.println("Receiver reports CRC32 MISMATCH");
        return -3;
      }
      // some other frame (e.g. a late window ACK): keep waiting without burning a try
    } else {
      t++;
      sendData(pktCount - 1, true); // re-poke so receiver resends the verdict
    }
  }
  Serial.println("No final confirmation from receiver");
  return -4;
}

// ---------- test image ----------
static void makeTestImage() {
  imgSize = TEST_IMAGE_SIZE;
  img = (uint8_t *)malloc(imgSize);
  if (!img) { Serial.println("alloc failed"); while (1) delay(1000); }
  for (uint32_t i = 0; i < imgSize; i++) img[i] = (uint8_t)(i * 31 + (i >> 8));
  Serial.printf("Test image ready: %lu bytes, CRC32=%08lX\n",
                (unsigned long)imgSize, (unsigned long)crc32(img, imgSize));
}

// ---------- load an image pushed from the PC over USB serial ----------
static bool loadFromPC(uint32_t n) {
  if (n == 0 || n > 400000) { Serial.println("ERR bad size"); return false; }
  // Free previous buffer first so a large image needs only one allocation.
  // The STM32U575 has on-chip SRAM, not ESP32 PSRAM.
  if (img) { free(img); img = nullptr; }
  imgSize = 0;
  img = (uint8_t *)malloc(n);
  if (!img) {
    Serial.printf("ERR alloc (%lu bytes); reduce the input size\n", (unsigned long)n);
    return false;
  }
  Serial.println("OK_SEND"); // PC waits for this before streaming bytes
  Serial.setTimeout(20000);
  size_t got = Serial.readBytes(img, n);
  if (got != n) {
    Serial.printf("ERR got %lu of %lu bytes\n", (unsigned long)got, (unsigned long)n);
    free(img);
    img = nullptr;
    return false;
  }
  imgSize = n;
  Serial.printf("LOADED %lu bytes, CRC32=%08lX\n", (unsigned long)n, (unsigned long)crc32(img, n));
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(AUX_PIN, INPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  digitalWrite(M0_PIN, LOW);   // M0=M1=LOW: normal (transparent) mode
  digitalWrite(M1_PIN, LOW);
  delay(100);
  LoRaSerial.begin(9600, SERIAL_8N1);  // match E32 module UART setting
  delay(500);
  Serial.println("Sender ready");
  Serial.println("STM32U575 SRAM image buffering enabled (no PSRAM required)");
  makeTestImage();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "s" || line == "S") {
      if (img) sendPending = true; // resend current image
    } else if (line == "PING") {
      Serial.printf("PONG %d\n", paired ? 1 : 0);
    } else if (line.startsWith("SEND ")) {
      uint32_t n = line.substring(5).toInt();
      if (paired && loadFromPC(n)) sendPending = true;
    }
  }

  if (!paired) { tryPair(); return; }

  if (sendPending) {
    sendPending = false;
    if (sendImage() != 0) {
      paired = false;   // link problem: re-pair before trying again
      Serial.println("Transfer failed; will re-pair");
    }
  }
}

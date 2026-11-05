// WHAT THIS DOES
//   Listens for an image sent over LoRa, rebuilds it in memory, checks that
//   every byte arrived correctly, then sends it to the computer over USB
//   serial so the Python script (receive_image.py) can save it as a file.
//
// HOW THE CONVERSATION WITH THE SENDER WORKS
//   1. PAIRING - Sender sends HELLO. We reply HELLO + ACK.

//   2. START - Sender sends SYN (image size, number of packets, CRC32).
//   We reply SYN + ACK.

//   3. DATA - Sender sends the image as small numbered packets, in groups
//   of 32. The last packet in each window has the REQ flag, which means "please tell me what you got".

//   4. ACK - When we see REQ we reply with a bitmap of which packets in
//   that window we have. The sender re-sends any that are missing.

//   5. VERDICT - Once every packet has arrived we check the CRC32 of the whole
//   image and tell the sender whether it was OK or corrupted.
//
// SERIAL MONITOR (115200 baud) MESSAGES
//   "*** PAIRED with sender ***"  -> the first HELLO arrived
//   "IMAGE RECEIVED OK ..." -> every packet arrived AND the CRC32 matched
//
// ARDUINO IDE SETTING (MAKE SURE YOU DO THIS)
//   Tools > PSRAM > OPI PSRAM
//
// This file is self-contained, so you don't need a separate header file.
// The packet format below must match the sender exactly.
// =====================================================================

#include <stdint.h>

// Pins 
#define LORA_RX_PIN 1    // ESP32 RX  <- E32 TXD
#define LORA_TX_PIN 2    // ESP32 TX  -> E32 RXD
#define AUX_PIN 42   // <- E32 AUX (LOW while the module is busy)
#define M0_PIN 14   // -> E32 M0
#define M1_PIN 21   // -> E32 M1

// ---------------------------------------------------------------------
// Packet format (must match sender)
//
//   byte 0      SYNC     always 0xAA, marks the start of a packet
//   byte 1      FLAGS    what kind of packet this is (see below)
//   byte 2      IMAGE ID which image this packet belongs to
//   byte 3-4    SEQ      packet number (high byte first)
//   byte 5      LEN      how many payload bytes follow (0 to 48)
//   byte 6...   PAYLOAD  the data
//   last 2      CRC16    error check over FLAGS through PAYLOAD
// ---------------------------------------------------------------------
const uint8_t SYNC_BYTE = 0xAA;

// Flags (can be combined, e.g. SYN + ACK)
const uint8_t FLAG_SYN   = 0x01; // start of a new image
const uint8_t FLAG_ACK   = 0x02; // acknowledgment
const uint8_t FLAG_END   = 0x04; // last packet of the image
const uint8_t FLAG_REQ   = 0x08; // last packet in a window: please ACK now
const uint8_t FLAG_RST   = 0x10; // abort / error
const uint8_t FLAG_HELLO = 0x20; // pairing

const int MAX_PAYLOAD = 48; // data bytes per packet
const int HEADER_LEN  = 6; // SYNC, FLAGS, ID, SEQ(2), LEN
const int MAX_PACKET  = HEADER_LEN + MAX_PAYLOAD + 2; // header + payload + CRC16
const int WINDOW_SIZE = 32; // packets per window

const int SYN_PAYLOAD_LEN = 10; // image size (4) + packet count (2) + CRC32 (4)

// Verdict codes we send back at the end
const uint8_t VERDICT_OK = 0;
const uint8_t VERDICT_BAD_CRC = 1;

// One packet after it has been read and checked
typedef struct {
  uint8_t  flags;
  uint8_t  id;
  uint16_t seq;
  uint8_t  len;
  uint8_t  payload[MAX_PAYLOAD];
} Frame;

// Collects incoming bytes until a whole packet is there
typedef struct {
  uint8_t bytes[MAX_PACKET];
  uint8_t count; // how many bytes we have so far
} Parser;

// Program state
Parser parser;

bool isPaired = false;   // have we ever heard a HELLO?
bool transferActive = false;   // are we working on an image right now?
bool resultReported = false;   // have we already printed OK/CORRUPT for this image?
bool dumpPending = false;   // should we send the image to the PC after this ACK?

uint8_t  *imageBuffer = nullptr;  // where the image is rebuilt
uint8_t  *packetReceived = nullptr;  // packetReceived[n] = 1 once packet n has arrived
uint32_t  imageSize  = 0;        // bytes
uint16_t  totalPackets = 0;
uint32_t  expectedCrc32  = 0;        // CRC32 the sender says the image should have
uint8_t   currentImageId = 0;
uint32_t  transferStartMs = 0;
uint8_t   verdict = VERDICT_BAD_CRC;   // only meaningful once all packets are in

// Small helpers: CRCs and 32-bit numbers

// CRC16-CCITT (polynomial 0x1021, starting value 0xFFFF). Used on every packet.
uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// Standard CRC32. Used once on the whole image at the end.
uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
      else         crc = (crc >> 1);
    }
  }
  return ~crc;
}

// Write a 32-bit number into 4 bytes, high byte first
void putUint32(uint8_t *dest, uint32_t value) {
  dest[0] = value >> 24;
  dest[1] = value >> 16;
  dest[2] = value >> 8;
  dest[3] = value;
}

// Read a 32-bit number from 4 bytes, high byte first
uint32_t getUint32(const uint8_t *src) {
  return ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8)  |  (uint32_t)src[3];
}


// ---------------------------------------------------------------------
// Building and reading packets
// ---------------------------------------------------------------------

// Put a packet together in "out" (which must hold MAX_PACKET bytes).
// Returns the total number of bytes in the packet.
size_t buildFrame(uint8_t *out, uint8_t flags, uint8_t id, uint16_t seq,
                  const uint8_t *payload, uint8_t len) {
  out[0] = SYNC_BYTE;
  out[1] = flags;
  out[2] = id;
  out[3] = seq >> 8;
  out[4] = seq & 0xFF;
  out[5] = len;
  if (len > 0){
    memcpy(out + HEADER_LEN, payload, len);
  } 

  // CRC covers everything except the SYNC byte and the CRC itself
  uint16_t crc = crc16(out + 1, 5 + len);
  out[HEADER_LEN + len] = crc >> 8;
  out[HEADER_LEN + len + 1] = crc & 0xFF;

  return HEADER_LEN + len + 2;
}

// Give the parser one received byte.
// Returns true when a complete packet with a correct CRC has been read into "frame".
// A packet with a bad CRC is silently thrown away; the sender will re-send it.
bool feedByte(Parser *p, uint8_t b, Frame *frame) {
  // Ignore anything until we see the start-of-packet marker
  if (p->count == 0 && b != SYNC_BYTE){
    return false;
  } 

  p->bytes[p->count++] = b;

  // We can't know the packet length until the whole header is in
  if (p->count < HEADER_LEN) return false;

  uint8_t payloadLen = p->bytes[5];
  if (payloadLen > MAX_PAYLOAD) { // nonsense length, so start over
    p->count = 0;
    return false;
  }

  uint8_t totalLen = HEADER_LEN + payloadLen + 2;
  if (p->count < totalLen){
    return false;
  }   // still waiting for the rest

  p->count = 0;   // the packet is complete, get ready for the next one

  // Check the CRC16 at the end of the packet
  uint16_t crcReceived = ((uint16_t)p->bytes[totalLen - 2] << 8) | p->bytes[totalLen - 1];
  uint16_t crcCalculated = crc16(p->bytes + 1, totalLen - 3);
  if (crcReceived != crcCalculated){
    return false;
  } 

  // Good packet, so copy the fields out
  frame->flags = p->bytes[1];
  frame->id = p->bytes[2];
  frame->seq = ((uint16_t)p->bytes[3] << 8) | p->bytes[4];
  frame->len = payloadLen;
  memcpy(frame->payload, p->bytes + HEADER_LEN, payloadLen);
  return true;
}


// ---------------------------------------------------------------------
// Talking to the radio module
// ---------------------------------------------------------------------

// Wait until the module says it's idle. AUX is LOW while it is busy,
// so we wait for it to go HIGH (but give up after 2 seconds).
void waitForAux() {
  uint32_t startTime = millis();
  while (digitalRead(AUX_PIN) == LOW && millis() - startTime < 2000) {
    delay(1);
  }
  delay(3); // the module wants a couple of ms after AUX goes high
}

// Send one packet and wait until the module has fully transmitted it.
void sendFrame(uint8_t flags, uint8_t id, uint16_t seq, const uint8_t *payload, uint8_t len) {
  uint8_t packet[MAX_PACKET];
  size_t packetLen = buildFrame(packet, flags, id, seq, payload, len);

  waitForAux(); // make sure the module is ready
  Serial1.write(packet, packetLen);
  Serial1.flush(); // wait until all bytes have left the ESP32
  delay(5); // give AUX time to go LOW
  waitForAux(); // wait until it has been sent over the air
}


// ---------------------------------------------------------------------
// Keeping track of the image
// ---------------------------------------------------------------------

// How many packets have we received so far?
uint16_t countReceivedPackets() {
  uint16_t count = 0;
  for (uint16_t i = 0; i < totalPackets; i++){
    count += packetReceived[i];
  }
  return count;
}

bool allPacketsReceived() {
  for (uint16_t i = 0; i < totalPackets; i++) {
    if (!packetReceived[i]){
      return false;
    }
  }
  return true;
}

// Reply to the sender with a bitmap of which packets in this window we have.
// "windowStart" is the number of the first packet in the window.
// Bit n of the bitmap is 1 if packet (windowStart + n) has arrived.
void sendWindowAck(uint16_t windowStart) {
  uint32_t bitmap = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    uint16_t packetNumber = windowStart + i;
    // Positions past the end of the image count as "received" so the last
    // (shorter) window can still come out as all ones.
    if (packetNumber >= totalPackets || packetReceived[packetNumber]) {
      bitmap |= (1u << i);
    }
  }

  uint8_t payload[4];
  putUint32(payload, bitmap);
  sendFrame(FLAG_ACK, currentImageId, windowStart, payload, 4);
}

// Send the finished image to the computer over USB serial in this form:
//   "IMG_BEGIN <size>\n"  then the raw image bytes  then  "\nIMG_END\n"
// The Python script (receive_image.py) looks for these markers.
void dumpImageToPC() {
  Serial.printf("IMG_BEGIN %lu\n", (unsigned long)imageSize);
  Serial.write(imageBuffer, imageSize);
  Serial.println();
  Serial.println("IMG_END");
}


// ---------------------------------------------------------------------
// Handling each kind of packet
// ---------------------------------------------------------------------

// PAIRING: sender says hello, we say hello back.
void handleHello() {
  sendFrame(FLAG_HELLO | FLAG_ACK, 0, 0, nullptr, 0);

  if (!isPaired) {
    isPaired = true;
    Serial.println("*** PAIRED with sender ***");
  }
}

// Get ready for a brand-new image: free the old buffers, read the details
// from the SYN packet, and allocate new buffers.
// Returns false if we ran out of memory.
bool startNewTransfer(const Frame *f) {
  // Throw away whatever was left from the previous image
  if (imageBuffer){
    free(imageBuffer);
  }
  if (packetReceived){
    free(packetReceived);
  }

  // SYN payload layout: image size (4 bytes), packet count (2), CRC32 (4)
  imageSize = getUint32(f->payload);
  totalPackets = ((uint16_t)f->payload[4] << 8) | f->payload[5];
  expectedCrc32 = getUint32(f->payload + 6);

  // Prefer PSRAM; fall back to normal memory if there isn't any
  imageBuffer = (uint8_t *)ps_malloc(imageSize);
  if (!imageBuffer){
    imageBuffer = (uint8_t *)malloc(imageSize);
  }

  packetReceived = (uint8_t *)calloc(totalPackets, 1);   // starts as all zeros

  if (!imageBuffer || !packetReceived) {
    uint8_t reasonCode = 1;   // 1 = out of memory
    sendFrame(FLAG_RST, f->id, 0, &reasonCode, 1);
    Serial.println("Alloc failed, sent RST");
    return false;
  }

  currentImageId  = f->id;
  transferActive  = true;
  resultReported  = false;
  transferStartMs = millis();

  Serial.printf("Incoming image id=%u: %lu bytes, %u packets\n",
                currentImageId, (unsigned long)imageSize, totalPackets);
  return true;
}

// START OF IMAGE: sender tells us the size, packet count and CRC32.
void handleSyn(const Frame *f) {
  // If the sender repeats a SYN for the image we're already working on
  // (because our reply got lost), we just reply again without starting over.
  bool isNewImage = !(transferActive && f->id == currentImageId);

  if (isNewImage) {
    bool ok = startNewTransfer(f);
    if (!ok) return;
  }

  // Echo the SYN payload back as the SYN + ACK reply
  sendFrame(FLAG_SYN | FLAG_ACK, currentImageId, 0, f->payload, f->len);
}

// Once every packet is in, check the whole image and report the result.
void finishImage() {
  if (!resultReported) {
    resultReported = true;

    uint32_t actualCrc = crc32(imageBuffer, imageSize);
    verdict = (actualCrc == expectedCrc32) ? VERDICT_OK : VERDICT_BAD_CRC;

    if (verdict == VERDICT_OK) {
      Serial.printf("IMAGE RECEIVED OK: %lu bytes, CRC32 %08lX matches (%.1f s)\n",
                    (unsigned long)imageSize, (unsigned long)actualCrc,
                    (millis() - transferStartMs) / 1000.0);
      dumpPending = true;   // send to the PC after the verdict goes out over radio
    } else {
      Serial.printf("IMAGE CORRUPT: CRC32 %08lX, expected %08lX\n",
                    (unsigned long)actualCrc, (unsigned long)expectedCrc32);
    }
  }

  // Tell the sender the verdict. If the sender didn't hear it, it will ask
  // again with another REQ and we end up back here to repeat it.
  sendFrame(FLAG_ACK | FLAG_END, currentImageId, 0, &verdict, 1);

  if (dumpPending) {
    dumpPending = false;
    dumpImageToPC();
  }
}

// DATA: one numbered chunk of the image.
void handleData(const Frame *f) {
  // Store the chunk in the right spot in the image buffer
  if (f->seq < totalPackets) {
    uint32_t offset = (uint32_t)f->seq * MAX_PAYLOAD;
    if (offset + f->len <= imageSize) {         // make sure it fits
      memcpy(imageBuffer + offset, f->payload, f->len);
      packetReceived[f->seq] = 1;
    }
  }

  // Only reply when the sender asks (the REQ flag on the last packet of a window)
  if (f->flags & FLAG_REQ) {
    uint16_t windowStart = (f->seq / WINDOW_SIZE) * WINDOW_SIZE;
    sendWindowAck(windowStart);

    if (allPacketsReceived()) {
      finishImage();
    } else {
      Serial.printf("  %u/%u packets\n", countReceivedPackets(), totalPackets);
    }
  }
}

// Look at the flags of a valid packet and pass it to the right handler.
void handleFrame(const Frame *f) {
  if (f->flags == FLAG_HELLO) {
    handleHello();
    return;
  }

  if (f->flags == FLAG_SYN && f->len == SYN_PAYLOAD_LEN) {
    handleSyn(f);
    return;
  }

  // Anything else is a data packet, but only if it belongs to the image
  // we're currently receiving and isn't one of the control types.
  bool isControlPacket = f->flags & (FLAG_SYN | FLAG_ACK | FLAG_RST | FLAG_HELLO);
  if (transferActive && f->id == currentImageId && !isControlPacket) {
    handleData(f);
  }
}

void setup() {
  Serial.begin(115200); // USB connection to the computer

  pinMode(AUX_PIN, INPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  digitalWrite(M0_PIN, LOW); // M0 = M1 = LOW means normal (transparent) mode
  digitalWrite(M1_PIN, LOW);
  delay(100);

  Serial1.setRxBufferSize(1024);
  Serial1.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN); // must match the module's UART setting
  delay(500);

  Serial.println("Receiver ready, waiting for sender...");
}

void loop() {
  Frame frame;

  // Read every byte the radio has given us. Whenever a full, valid
  // packet has been assembled, deal with it.
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (feedByte(parser, b, frame)) {
      handleFrame(frame);
    }
  }
}
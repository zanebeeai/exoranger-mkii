#include <Arduino.h>
#include <SPI.h>

// ---------------- user pins ----------------
static const uint8_t PIN_CS   = 3;
static const uint8_t PIN_DRDY = 2;  // optional but recommended (active-low)

// ---------------- ADS1220 commands ----------------
static const uint8_t CMD_RESET      = 0x06;
static const uint8_t CMD_START_SYNC = 0x08;
static const uint8_t CMD_POWERDOWN  = 0x02;
static const uint8_t CMD_RDATA      = 0x10;
static const uint8_t CMD_RREG_BASE  = 0x20; // 0010 rrnn
static const uint8_t CMD_WREG_BASE  = 0x40; // 0100 rrnn

// ---------------- desired configuration ----------------
// REG0: MUX=AIN0-AIN1 (0000), GAIN=128 (111), PGA_BYPASS=0
static const uint8_t REG0 = 0x0E;

// REG1: DR=20SPS (000), MODE=normal (00), CM=1 (continuous), TS=0, BCS=0
static const uint8_t REG1 = 0x04;

// REG2: VREF=REFP0/REFN0 (01), 50/60 rejection=01, PSW=0, IDAC=000
static const uint8_t REG2 = 0x50;

// REG3: leave default
static const uint8_t REG3 = 0x00;

// Teensy SPISettings cannot be constexpr in this core.
static const SPISettings ADS_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE1); // ADS1220 uses SPI mode 1

// Use SPI1 (per your wiring)
#define ADS_SPI SPI1

volatile bool g_drdy = false;

void onDrdyFall() {
  g_drdy = true;
}

static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

uint8_t txrx(uint8_t b) {
  return ADS_SPI.transfer(b);
}

void adsSendCommand(uint8_t cmd) {
  csLow();
  ADS_SPI.beginTransaction(ADS_SPI_SETTINGS);
  txrx(cmd);
  ADS_SPI.endTransaction();
  csHigh();
}

void adsWriteRegs(uint8_t startReg, const uint8_t *data, uint8_t n) {
  // WREG: 0100 rrnn, nn=(n-1)
  uint8_t cmd = (uint8_t)(CMD_WREG_BASE | ((startReg & 0x03) << 2) | ((n - 1) & 0x03));
  csLow();
  ADS_SPI.beginTransaction(ADS_SPI_SETTINGS);
  txrx(cmd);
  for (uint8_t i = 0; i < n; i++) txrx(data[i]);
  ADS_SPI.endTransaction();
  csHigh();
}

void adsReadRegs(uint8_t startReg, uint8_t *out, uint8_t n) {
  // RREG: 0010 rrnn, nn=(n-1)
  uint8_t cmd = (uint8_t)(CMD_RREG_BASE | ((startReg & 0x03) << 2) | ((n - 1) & 0x03));
  csLow();
  ADS_SPI.beginTransaction(ADS_SPI_SETTINGS);
  txrx(cmd);
  for (uint8_t i = 0; i < n; i++) out[i] = txrx(0x00);
  ADS_SPI.endTransaction();
  csHigh();
}

int32_t adsReadData24() {
  csLow();
  ADS_SPI.beginTransaction(ADS_SPI_SETTINGS);

  txrx(CMD_RDATA);
  uint32_t b0 = txrx(0x00);
  uint32_t b1 = txrx(0x00);
  uint32_t b2 = txrx(0x00);

  ADS_SPI.endTransaction();
  csHigh();

  uint32_t raw24 = (b0 << 16) | (b1 << 8) | b2;

  // sign extend 24->32
  if (raw24 & 0x800000) raw24 |= 0xFF000000;
  return (int32_t)raw24;
}

static const int ADS_GAIN = 128;

double codeTo_mVperV(int32_t code) {
  // code ≈ (Vin/Vref) * gain * (2^23 - 1)
  // with ratiometric wiring: Vref = Vexc, so Vin/Vexc is what we want.
  const double FS = 8388607.0; // 2^23 - 1
  double ratio = (double)code / (FS * (double)ADS_GAIN); // Vin/Vexc
  return ratio * 1000.0; // mV/V
}

int32_t tare_code = 0;

void printRegs() {
  uint8_t r[4] = {0};
  adsReadRegs(0, r, 4);
  Serial.printf("regs: r0=0x%02X r1=0x%02X r2=0x%02X r3=0x%02X\r\n", r[0], r[1], r[2], r[3]);
}

int32_t avgN(int N) {
  int64_t acc = 0;
  for (int i = 0; i < N; i++) {
    // wait DRDY (active low)
    while (!g_drdy) { /* spin */ }
    g_drdy = false;
    acc += adsReadData24();
  }
  return (int32_t)(acc / N);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CS, OUTPUT);
  csHigh();

  pinMode(PIN_DRDY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), onDrdyFall, FALLING);

  ADS_SPI.begin();

  // Reset ADS1220
  adsSendCommand(CMD_RESET);
  delay(5);

  // Write config registers
  const uint8_t regs[4] = {REG0, REG1, REG2, REG3};
  adsWriteRegs(0, regs, 4);
  delay(2);

  // Start continuous conversions
  adsSendCommand(CMD_START_SYNC);
  delay(50);

  printRegs();

  // Initial tare
  tare_code = avgN(10);
  Serial.printf("tare=%ld\r\n", (long)tare_code);
  Serial.println("ready. 't' to tare, 'r' to read regs.");
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 't') {
      tare_code = avgN(20);
      Serial.printf("tare set: %ld\r\n", (long)tare_code);
    } else if (c == 'r') {
      printRegs();
    }
  }

  if (!g_drdy) return;
  g_drdy = false;

  int32_t code = adsReadData24();
  int32_t z = code - tare_code;

  double mv_v = codeTo_mVperV(code);
  double mv_v_z = codeTo_mVperV(z);

  Serial.printf("code=%ld  mV/V=%.6f  mV/V(tare)=%.6f\r\n", (long)code, mv_v, mv_v_z);
}

/*
    Component Paramenter Measurement Client
    Term project for AI2618
    Copyright © 2023 Fangyuan Zhou, All Rights Reserved.
    Shanghai Jiao Tong University, Artificial Intelligence
*/

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <SPI.h>

#include "X9C.h"
#include "bitmaps.h"
#include "Vector2.h"

X9C* resB = nullptr;
X9C* resC = nullptr;

#define withDisplay

#ifdef withDisplay
#define ALTERNATE_PINS
#define TFT_CS D1
#define TFT_RST D2  // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC D0
#define TFT_MOSI D11  // Data out
#define TFT_SCLK D13  // Clock out

SPIClass* hspi = nullptr;
Adafruit_ST7789* tft = nullptr;

#endif

// Analog Pin Configuration

#define highInput A3
#define highInputSense A2
#define lowInput A5
#define baseInputH A1
#define baseInputL A4
#define baseHigh D3
#define high1k D4
#define high100k A0
#define low1k A6
#define low100k A7
#define collectorHigh D10
#define collectorLow D9

// Digital Pin Configuration

#define X9C_INC D8
#define X9C_UD D7
#define X9C_CSb D5
#define X9C_CSc D6

#define MOS_Gate
#define MOS_Source
#define MOS_Drain

const bool debug = false;

const size_t adc_buffer_size = 4096;
int adc_buffer[adc_buffer_size];
unsigned long time_buffer[adc_buffer_size];

// calibration data
const double Vcc = 3.249;
const double high1k_value = 990.31;
const double high100k_value = 98976;
const double low1k_value = 997.35;
const double low100k_value = 101668;
const double baseSense_value = 200000;
const double highSense_value = 996.42;

double highResistance;
double lowResistance;

enum componentType { None, Capacitor, Resistor, PNP, NPN, PMOS, NMOS, Diode };

#ifdef withDisplay
componentType currentMode;
#endif

void initializePins() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(highInput, INPUT);
  pinMode(lowInput, INPUT);
  pinMode(baseInputH, INPUT);
  pinMode(baseInputL, INPUT);
  pinMode(high1k, OUTPUT);
  pinMode(low1k, OUTPUT);
  pinMode(high100k, OUTPUT);
  pinMode(low100k, OUTPUT);
  pinMode(baseHigh, OUTPUT);
  digitalWrite(baseHigh, HIGH);
}

#ifdef withDisplay
void initializeDisplay() {
  hspi = new SPIClass(HSPI);
  hspi->begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft = new Adafruit_ST7789(hspi, TFT_CS, TFT_DC, TFT_RST);
  tft->init(240, 240);
  hspi->setFrequency(80000000);
  tft->setSPISpeed(80000000);
  // tft->fillScreen(ST77XX_BLACK);

  // tft->setCursor(40, 90);
  // tft->setTextColor(ST77XX_WHITE);
  // tft->setTextSize(4);
  // tft->println("UIMicro");
  tft->drawRGBBitmap(0, 0, (const uint16_t*)gImage_uimicro_text, 240, 240);
}
#endif

void initializePeripherals() {
  Serial.begin(230400);
#ifdef withDisplay
  initializeDisplay();
  currentMode = None;
#endif
  resB = new X9C(X9C_CSb, X9C_INC, X9C_UD);
  resC = new X9C(X9C_CSc, X9C_INC, X9C_UD);
  resB->setPos(X9C::MAX);
  resC->setPos(X9C::MAX);
}

// enable weak pullup and pulldown resistor
void setHighImpedance() {
  highResistance = high100k_value;
  lowResistance = low100k_value;
  pinMode(high1k, INPUT);
  pinMode(low1k, INPUT);
  pinMode(high100k, OUTPUT);
  pinMode(low100k, OUTPUT);
  pinMode(highInput, INPUT);
  pinMode(lowInput, INPUT);
  pinMode(collectorHigh, INPUT);
  pinMode(collectorLow, INPUT);
  pinMode(baseInputL, INPUT);
  pinMode(baseInputH, INPUT);
  pinMode(baseHigh, OUTPUT);
  digitalWrite(baseHigh, LOW);
  digitalWrite(high100k, HIGH);
  digitalWrite(low100k, LOW);
  resB->setPos(X9C::MAX);
}

// enable strong pullup and pulldown resistor
void setLowImpedance() {
  highResistance = high1k_value;
  lowResistance = low1k_value;
  pinMode(high1k, OUTPUT);
  pinMode(low1k, OUTPUT);
  pinMode(high100k, INPUT);
  pinMode(low100k, INPUT);
  pinMode(highInput, INPUT);
  pinMode(lowInput, INPUT);
  pinMode(collectorHigh, INPUT);
  pinMode(collectorLow, INPUT);
  pinMode(baseInputL, INPUT);
  pinMode(baseInputH, INPUT);
  digitalWrite(high1k, HIGH);
  digitalWrite(low1k, LOW);
}

void setZero() {
  highResistance =
      (high100k_value * high1k_value) / (high100k_value + high1k_value);
  lowResistance = (low100k_value * low1k_value) / (low100k_value + low1k_value);
  pinMode(high1k, OUTPUT);
  pinMode(low1k, OUTPUT);
  pinMode(high100k, INPUT);
  pinMode(low100k, INPUT);
  digitalWrite(high1k, LOW);
  digitalWrite(low1k, LOW);
  digitalWrite(baseHigh, LOW);
}

// repeat sample to reduce noise
double repeatSample(uint8_t pin, size_t sampleTimes, uint32_t delay_us) {
  unsigned long long sum = 0;
  for (int i = 0; i < sampleTimes; ++i) {
    delayMicroseconds(delay_us);
    sum += analogReadMilliVolts(pin);
  }
  return (double)sum / sampleTimes / 1000.0;
}

// repeat sample to reduce noise
double repeatSampleDifferential(uint8_t pinH, uint8_t pinL, size_t sampleTimes,
                                uint32_t delay_us) {
  long long sum = 0;
  for (int i = 0; i < sampleTimes; ++i) {
    delayMicroseconds(delay_us);
    sum = sum + analogReadMilliVolts(pinH) -
          analogReadMilliVolts(pinL);  // strange problem. += doesn't work.
  }
  return (double)sum / sampleTimes / 1000.0;
}

double _measureCapacitor() {
  unsigned long start, end;
  int upperLimit = Vcc / 3.3 * 2600;
  int lowerLimit = Vcc / 3.3 * 500;
  const int timeout = 5000000;
  setZero();
  start = micros();
  while (analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput) > 5 &&
         micros() - start < timeout)
    ;
  setHighImpedance();
  int differentialVoltage = 0;
  while (differentialVoltage < lowerLimit) {
    differentialVoltage =
        analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput);
    // differentialVoltage = repeatSampleRaw(highInput, lowInput, 16);
  }
  start = micros();

  while (differentialVoltage < upperLimit) {
    differentialVoltage =
        analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput);
    // differentialVoltage = repeatSampleRaw(highInput, lowInput, 16);
  }
  end = micros();
  double capacitance =
      (double)(end - start) / (highResistance + lowResistance) / 1.64866;
  // if (capacitance < 0.5) {
  //     // re-measure
  //     setZero();
  //     while (analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput)
  //     > 5); start = micros(); setHighImpedance(); for (int i = 0; i < 4096;
  //     ++i) {
  //         adc_buffer[i] = analogReadMilliVolts(highInput) -
  //         analogReadMilliVolts(lowInput); time_buffer[i] = micros() - start;
  //     }
  //     for (int i = 0; i < 4096; ++i) {
  //         Serial.printf("%lf %d\n", time_buffer[i] / 1000000.0,
  //         adc_buffer[i]);
  //     }
  // }
  return capacitance;
}

void measureCapacitor() {
#ifdef withDisplay
  if (currentMode != Capacitor) {
    currentMode = Capacitor;
    tft->fillScreen(ST77XX_BLACK);

    tft->setCursor(50, 60);
    tft->setTextColor(ST77XX_WHITE);
    tft->setTextSize(3);
    tft->printf("Capacitor");

  }
#endif
  Serial.println("Measure Capacitor");

  double capacitance = _measureCapacitor();

  Serial.printf("Capacitance: %lf uF\n", capacitance);

  Serial.printf("Capacitance: %lf\n", capacitance);

#ifdef withDisplay
tft->fillRect(0, 110, 240, 40, ST77XX_BLACK);
  if (capacitance > 1) {
    tft->setCursor(40, 110);
    tft->printf("%.*g uF", 4, capacitance);
  } else {
    tft->setCursor(40, 110);
    tft->printf("%.*g nF", 4, capacitance * 1e3);
  }
#endif

  setZero();
  while (analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput) > 5)
    ;
}

double _measureResistor() {
  setHighImpedance();
  double highVoltage = repeatSample(highInput, 5120, 10);
  double lowVoltage = repeatSample(lowInput, 5120, 10);
  // double supplyVoltage = lowVoltage / lowResistance * highResistance +
  // highVoltage;
  double measuredResistance =
      (highVoltage - lowVoltage) / (lowVoltage / lowResistance);
  if (measuredResistance < 10000) {
    setLowImpedance();
    highVoltage = repeatSample(highInput, 256, 100);
    lowVoltage = repeatSample(lowInput, 256, 100);
    measuredResistance =
        (highVoltage - lowVoltage) / (lowVoltage / lowResistance);
  }
  if (debug) {
    Serial.printf(
        "Highside Voltage: %lf, Lowside Voltage: %lf, Resistance: %lf\nWaiting "
        "for serial.",
        highVoltage, lowVoltage, measuredResistance);
    // while (!Serial.available());
    // Serial.read();
  }
  return measuredResistance;
}

void measureResistor() {
  double resistance = _measureResistor();
  Serial.printf("Resistance: %lf\n", resistance);

#ifdef withDisplay
  if (currentMode != Resistor) {
    currentMode = Resistor;
    tft->fillScreen(ST77XX_BLACK);

    tft->setCursor(50, 60);
    tft->setTextColor(ST77XX_WHITE);
    tft->setTextSize(3);
    tft->printf("Resistor");

  } else {
    tft->fillRect(0, 110, 240, 40, ST77XX_BLACK);
  }

  if (resistance > 1000) {
    tft->setCursor(20, 110);
    tft->printf("%.*g kOhms", 4, resistance / 1000);
  } else {
    tft->setCursor(30, 110);
    tft->printf("%4.*g Ohms", resistance);
  }

#endif
}

void measurePNP() {
  double beta;

  // set pin mode to measure PNP
  pinMode(high1k, INPUT);
  pinMode(low1k, OUTPUT);
  pinMode(high100k, INPUT);
  pinMode(low100k, INPUT);
  pinMode(highInput, OUTPUT);
  pinMode(lowInput, INPUT);
  pinMode(collectorHigh, INPUT);
  pinMode(collectorLow, INPUT);
  pinMode(baseInputL, INPUT);
  pinMode(baseInputH, INPUT);
  pinMode(baseHigh, OUTPUT);
  digitalWrite(baseHigh, HIGH);
  digitalWrite(low1k, LOW);
  digitalWrite(highInput, HIGH);

  resB->setPos(X9C::MAX);
  while (repeatSample(lowInput, 16, 10) < 1.4 && resB->getPos() > 5) {
    delay(1);
    resB->setPos(resB->getPos() - 1);
  }

  double collectorCurrent = repeatSample(lowInput, 512, 10) / low1k_value;
  double baseCurrent =
      repeatSampleDifferential(baseInputL, baseInputH, 512, 10) /
      baseSense_value;
  beta = collectorCurrent / baseCurrent;

  Serial.printf("beta: %lf\n", beta);

#ifdef withDisplay
  currentMode = PNP;
  tft->fillScreen(ST77XX_BLACK);
  tft->setCursor(60, 60);
  tft->setTextColor(ST77XX_WHITE);
  tft->setTextSize(3);
  tft->printf("PNP BJT");
  const uint16_t bar_x = 10, bar_y = 164, bar_w = 140, bar_h = 20;
  tft->setCursor(34, 100);
  tft->printf("beta:%.*g", 4, beta);
#endif
}

void measureNPN() {
  double beta;

  // set pin mode to measure PNP
  pinMode(high1k, OUTPUT);
  pinMode(low1k, INPUT);
  pinMode(high100k, INPUT);
  pinMode(low100k, INPUT);
  pinMode(highInput, INPUT);
  pinMode(lowInput, OUTPUT);
  pinMode(collectorHigh, INPUT);
  pinMode(collectorLow, INPUT);
  pinMode(baseInputL, INPUT);
  pinMode(baseInputH, INPUT);
  pinMode(baseHigh, OUTPUT);
  digitalWrite(baseHigh, HIGH);
  digitalWrite(high1k, HIGH);
  digitalWrite(lowInput, LOW);

  // while (1) {

  // digitalWrite(lowInput, LOW);
  // Serial.println("0");
  // delay(1);
  // digitalWrite(lowInput, HIGH);
  // Serial.println("1");
  // }

  resB->setPos(X9C::MAX);
  while (repeatSample(highInput, 16, 10) < 2.1 && resB->getPos() > 5) {
    delay(1);
    resB->setPos(resB->getPos() - 1);
  }

  if (resB->getPos() == 5 || resB->getPos() == X9C::MAX) {
    tft->drawRGBBitmap(0, 0, (const uint16_t*)gImage_syh, 240, 240);
    return;
  }
  double collectorCurrent =
      (Vcc - repeatSample(highInput, 512, 10)) / low1k_value;
  double baseCurrent =
      repeatSampleDifferential(baseInputH, baseInputL, 512, 10) /
      baseSense_value;
  beta = collectorCurrent / baseCurrent;

  Serial.printf("beta: %lf\n", beta);

#ifdef withDisplay
  currentMode = NPN;
  tft->fillScreen(ST77XX_BLACK);
  tft->setCursor(60, 60);
  tft->setTextColor(ST77XX_WHITE);
  tft->setTextSize(3);
  tft->printf("NPN BJT");
  const uint16_t bar_x = 10, bar_y = 164, bar_w = 140, bar_h = 20;
  tft->setCursor(34, 100);
  tft->printf("beta:%.*g", 4, beta);
  tft->setTextSize(2);
  tft->setCursor(30, 140);
  tft->printf("Measuring curve");
  tft->fillRect(bar_x, bar_y, bar_w, bar_h, ST77XX_WHITE);
  tft->fillRect(bar_x + 4, bar_y + 4, bar_w - 8, bar_h - 8, ST77XX_BLACK);
#endif

  // perform curve measurement
  pinMode(high1k, INPUT);
  pinMode(collectorHigh, OUTPUT);
  pinMode(collectorLow, OUTPUT);
  digitalWrite(collectorLow, LOW);
  digitalWrite(collectorHigh, HIGH);

  const int curveNum = 5;

  double* ibs = new double[curveNum];
  double** ics = new double*[curveNum];
  double** uce = new double*[curveNum];

  for (int i = 0; i < curveNum; ++i) {
    ics[i] = new double[100];
    uce[i] = new double[100];
  }

  resC->setPos(X9C::MAX);
  resB->setPos(X9C::MAX);

  double startib;
  // make sure the transistor don't saturate.
  while (repeatSample(highInput, 16, 10) < 2.5 && resB->getPos() > 5) {
    resB->setPos(resB->getPos() - 1);
  }

  if (resB->getPos() == 5 || resB->getPos() == X9C::MAX) {
    tft->drawRGBBitmap(0, 0, (const uint16_t*)gImage_syh, 240, 240);
    return;
  }

  if (debug) {
    Serial.printf("base resistor: %d\n", resB->getPos());
  }

  startib = repeatSampleDifferential(baseInputH, baseInputL, 64, 10) /
            baseSense_value;

  // while (repeatSampleDifferential(baseInputH, baseInputL, 64, 10) /
  // baseSense_value > startib / 3 && resB->getPos() > 10) {
  //     resB->setPos(resB->getPos() - 1);
  // }

  // startib = repeatSampleDifferential(baseInputH, baseInputL, 64, 10) /
  // baseSense_value;
  if (debug) {
    Serial.printf("startib: %lfuA\n", startib * 1000000);
  }
  for (int curveCnt = 0; curveCnt < curveNum; curveCnt++) {
    Serial.printf("Measuring curve %d\n", curveCnt);
    while (repeatSampleDifferential(baseInputH, baseInputL, 64, 10) /
                   baseSense_value >
               startib / curveNum * (curveNum - curveCnt) &&
           resB->getPos() > 5) {
      resB->setPos(resB->getPos() - 1);
    }
    ibs[curveCnt] = repeatSampleDifferential(baseInputH, baseInputL, 256, 1) /
                    baseSense_value;
    if (debug) {
      Serial.printf("pos: %d, IBS%d: %lfuA\n", resB->getPos(), curveCnt,
                    ibs[curveCnt] * 1000000);
    }
    double finishRatio;
    for (auto i = X9C::MIN; i <= X9C::MAX; ++i) {
      resC->setPos(i);
      ics[curveCnt][i] =
          repeatSampleDifferential(highInputSense, highInput, 320, 0) /
          highSense_value;
      uce[curveCnt][i] = repeatSampleDifferential(highInput, low1k, 320, 0);

#ifdef withDisplay
      finishRatio =
          (i + 1.0 * curveCnt * X9C::MAX) / (1.0 * curveNum * X9C::MAX);
      if (true) {
        tft->setCursor(160, 168);
        tft->fillRect(160, 168, 80, 20, ST77XX_BLACK);
        tft->printf("%.3g%%", finishRatio * 100.0);
      }
      tft->fillRect(bar_x + 6, bar_y + 6,
                    (uint16_t)(finishRatio * (bar_w - 12.0)), bar_h - 12,
                    ST77XX_WHITE);
#endif
    }
  }

  Serial.printf("Measurement Complete.\n");

  for (int curveCnt = 0; curveCnt < curveNum; curveCnt++) {
    Serial.printf("Curve %d: ib = %lfuA\n", curveCnt, ibs[curveCnt] * 1000000);
    for (int i = X9C::MIN; i <= X9C::MAX; ++i) {
      Serial.printf("ic: %lfmA, uce: %lf\n", ics[curveCnt][i] * 1000,
                    uce[curveCnt][i]);
    }
  }

#ifdef withDisplay
  int x1, x2, y1, y2;

  // draw axis
  tft->fillScreen(ST77XX_BLACK);
  tft->drawLine(10, 230, 230, 230, ST77XX_WHITE);
  tft->drawLine(10, 10, 10, 230, ST77XX_WHITE);
  tft->drawLine(6, 14, 10, 10, ST77XX_WHITE);
  tft->drawLine(10, 10, 14, 14, ST77XX_WHITE);
  tft->drawLine(226, 226, 230, 230, ST77XX_WHITE);
  tft->drawLine(226, 234, 230, 230, ST77XX_WHITE);

  uint16_t color;
  double max_uce = 0, max_ics = 0;
  for (int i = 0; i < curveNum; ++i) {
    for (int j = 0; j <= 90; ++j) {
      if (uce[i][j] > max_uce) max_uce = uce[i][j];
      if (ics[i][j] > max_ics) max_ics = ics[i][j];
    }
  }
  double amp_uce = 220.0 / max_uce;
  double amp_ics = 220.0 / max_ics;
  // axis
  for (double ic = 0; ic < max_ics; ic += 0.1 / 1000) {
    if (((ic * 2000) - (int)(ic * 2000)) < 0.0001) {
      x1 = 6;
      x2 = 14;
      color = ST77XX_WHITE;
    } else {
      x1 = 8;
      x2 = 12;
      color = ST77XX_RED;
    }
    y1 = (int)(230.0 - ic * amp_ics);
    y2 = (int)(230.0 - ic * amp_ics);

    // Serial.printf("(%d, %d), (%d, %d)\n", x1, y1, x2, y2);
    tft->drawLine(x1, y1, x2, y2, color);
  }

  for (double u = 0; u < max_uce; u += 0.5) {
    if ((u - (int)(u)) < 0.1) {
      y1 = 226;
      y2 = 234;
      color = ST77XX_WHITE;
    } else {
      y1 = 228;
      y2 = 232;
      color = ST77XX_RED;
    }

    x1 = (int)(10.0 + u * amp_uce);
    x2 = (int)(10.0 + u * amp_uce);
    Serial.printf("(%d, %d), (%d, %d)\n", x1, y1, x2, y2);
    tft->drawLine(x1, y1, x2, y2, color);
  }

  // curve
  for (int i = 0; i < curveNum; ++i) {
    for (int j = 0; j < 90; ++j) {
      x1 = (int)(10.0 + uce[i][j] * amp_uce);
      x2 = (int)(10.0 + uce[i][j + 1] * amp_uce);
      y1 = (int)(230.0 - ics[i][j] * amp_ics);
      y2 = (int)(230.0 - ics[i][j + 1] * amp_ics);
      // Serial.printf("(%d, %d), (%d, %d)\n", x1, y1, x2, y2);
      tft->drawLine(x1, y1, x2, y2, ST77XX_WHITE);
      if (j == 89) {
        x2 -= 64;
        y2 += 10;
        tft->setTextSize(1);
        tft->setCursor(x2, y2);
        tft->setTextColor(ST77XX_GREEN);
        tft->printf("ib: %.*guA", 4, ibs[i] * 1000000);
      }
    }
  }
#endif

  delete[] ibs;
  for (int i = 0; i < curveNum; ++i) {
    delete[] ics[i];
    delete[] uce[i];
  }
  delete[] ics;
  delete[] uce;
}


void measureDiode() {

#ifdef withDisplay
  currentMode = Diode;
  tft->fillScreen(ST77XX_BLACK);
  tft->setCursor(80, 90);
  tft->setTextColor(ST77XX_WHITE);
  tft->setTextSize(3);
  tft->printf("Diode");
  const uint16_t bar_x = 10, bar_y = 144, bar_w = 140, bar_h = 20;
  tft->setTextSize(2);
  tft->setCursor(30, 120);
  tft->printf("Measuring curve");
  tft->fillRect(bar_x, bar_y, bar_w, bar_h, ST77XX_WHITE);
  tft->fillRect(bar_x + 4, bar_y + 4, bar_w - 8, bar_h - 8, ST77XX_BLACK);
#endif

  pinMode(high1k, INPUT);
  pinMode(low1k, INPUT);
  pinMode(high100k, INPUT);
  pinMode(low100k, INPUT);
  pinMode(highInput, INPUT);
  pinMode(lowInput, OUTPUT);
  pinMode(collectorHigh, OUTPUT);
  pinMode(collectorLow, OUTPUT);
  pinMode(baseInputL, INPUT);
  pinMode(baseInputH, INPUT);
  pinMode(baseHigh, INPUT);
  digitalWrite(collectorHigh, HIGH);
  digitalWrite(collectorLow, LOW);
  digitalWrite(lowInput, LOW);

  double* id = new double[X9C::MAX + 1];
  double* ud = new double[X9C::MAX + 1];
  double finishRatio;
  for (int i = X9C::MIN; i <= X9C::MAX; ++i) {

    resC->setPos(i);
    id[i] = repeatSampleDifferential(highInputSense, highInput, 320, 0) /
          highSense_value;
    ud[i] = repeatSampleDifferential(highInput, low1k, 320, 0);
    #ifdef withDisplay
      finishRatio =
          (i * 1.0) / (1.0 * X9C::MAX);
      if (true) {
        tft->setCursor(160, 148);
        tft->fillRect(160, 148, 80, 20, ST77XX_BLACK);
        tft->printf("%.3g%%", finishRatio * 100.0);
      }
      tft->fillRect(bar_x + 6, bar_y + 6,
                    (uint16_t)(finishRatio * (bar_w - 12.0)), bar_h - 12,
                    ST77XX_WHITE);
      #endif
  }
  #ifdef withDisplay
  int x1, x2, y1, y2;

  // draw axis
  tft->fillScreen(ST77XX_BLACK);
  tft->drawLine(10, 230, 230, 230, ST77XX_WHITE);
  tft->drawLine(10, 10, 10, 230, ST77XX_WHITE);
  tft->drawLine(6, 14, 10, 10, ST77XX_WHITE);
  tft->drawLine(10, 10, 14, 14, ST77XX_WHITE);
  tft->drawLine(226, 226, 230, 230, ST77XX_WHITE);
  tft->drawLine(226, 234, 230, 230, ST77XX_WHITE);

  uint16_t color;
  double max_ud = 0, max_id = 0;
    for (int i = 0; i <= 99; ++i) {
      if (ud[i] > max_ud) max_ud = ud[i];
      if (id[i] > max_id) max_id = id[i];
    }
  double amp_u = 210.0 / max_ud;
  double amp_i = 210.0 / max_id;
  // axis
  for (double ic = 0; ic < max_id; ic += 0.1 / 1000) {
    if (((ic * 2000) - (int)(ic * 2000)) < 0.0001) {
      x1 = 6;
      x2 = 14;
      color = ST77XX_WHITE;
    } else {
      x1 = 8;
      x2 = 12;
      color = ST77XX_RED;
    }
    y1 = (int)(230.0 - ic * amp_i);
    y2 = (int)(230.0 - ic * amp_i);

    // Serial.printf("(%d, %d), (%d, %d)\n", x1, y1, x2, y2);
    tft->drawLine(x1, y1, x2, y2, color);
  }

  for (double u = 0; u < max_ud; u += 0.1) {
    if (((u * 2) - (int)(u * 2)) < 0.01) {
      y1 = 226;
      y2 = 234;
      color = ST77XX_WHITE;
    } else {
      y1 = 228;
      y2 = 232;
      color = ST77XX_RED;
    }

    x1 = (int)(10.0 + u * amp_u);
    x2 = (int)(10.0 + u * amp_u);
    Serial.printf("(%d, %d), (%d, %d)\n", x1, y1, x2, y2);
    tft->drawLine(x1, y1, x2, y2, color);
  }

  // curve
    for (int i = 0; i < 99; ++i) {
      x1 = (int)(10.0 + ud[i] * amp_u);
      x2 = (int)(10.0 + ud[i + 1] * amp_u);
      y1 = (int)(230.0 - id[i] * amp_i);
      y2 = (int)(230.0 - id[i + 1] * amp_i);
      // Serial.printf("(%d, %d), (%d, %d)\n", x1, y1, x2, y2);
      tft->drawLine(x1, y1, x2, y2, ST77XX_WHITE);
  }
#endif
  delete[] id;
  delete[] ud;
}

componentType decideComponent() {
  // discharge component
  setZero();
  delay(100);

  double transientVoltage, finalVoltage;
  setHighImpedance();

  if (debug) {
    Serial.printf("fuck");
    // while (!Serial.available());
    // Serial.read();
  }
  transientVoltage = repeatSampleDifferential(highInput, lowInput, 16, 0);
  delay(200);
  finalVoltage = repeatSampleDifferential(highInput, lowInput, 16, 0);

  if (debug) {
    Serial.printf("transientVoltage, %lf, finalVoltage, %lf", transientVoltage,
                  finalVoltage);
    // while (!Serial.available());
    // Serial.read();
  }

  if (transientVoltage - finalVoltage < 0.08 &&
      transientVoltage - finalVoltage > -0.08) {
    // BJT
    double collectorInitialVoltage = repeatSample(highInput, 16, 0);
    digitalWrite(baseHigh, HIGH);
    delay(10);
    double collectorFinalVoltage = repeatSample(highInput, 16, 0);
    double baseVoltage;
    if (debug) {
      Serial.printf("%lf, %lf, %lf", collectorInitialVoltage,
                    collectorFinalVoltage, baseVoltage);
    }
    if (collectorFinalVoltage - collectorInitialVoltage < -0.5) {
      baseVoltage = repeatSample(baseInputL, 16, 0);
      if (debug) {
        Serial.printf("%lf, %lf, %lf", collectorInitialVoltage,
                      collectorFinalVoltage, baseVoltage);
      }
      if (repeatSampleDifferential(baseInputL, lowInput, 16, 10) > 1.5) {
        return NMOS;
      } else {
        return NPN;
      }
    }
    if (collectorFinalVoltage - collectorInitialVoltage > 0.5) {
      digitalWrite(baseHigh, LOW);
      if (repeatSampleDifferential(highInput, baseInputL, 16, 10) > 1.5) {
        return PMOS;
      } else {
        return PNP;
      }
    }
    if (transientVoltage > 3.0) return None;
    // resistor & Diode
    double& highImpedanceDifferentialVoltage = transientVoltage;
    double& lowImpedanceDifferentialVoltage = finalVoltage;
    highImpedanceDifferentialVoltage =
        repeatSampleDifferential(highInput, lowInput, 16, 0);
    setLowImpedance();
    lowImpedanceDifferentialVoltage =
        repeatSampleDifferential(highInput, lowInput, 16, 0);
    if (lowImpedanceDifferentialVoltage - highImpedanceDifferentialVoltage >
        0.2) {
      return Resistor;
    } else {
      return Diode;
    }
  } else {
    return Capacitor;
  }
}

#ifdef withDisplay
void drawDynamicTriangle() {
    static float xmin = 240, ymin = 240, xmax = 0, ymax = 0;
    static unsigned long endt;
    static bool first_launch = true;
    static float dt = 0.033f;
    static const float mass = 1;
    static const float springK = 12;
    static const float damping = 0.9999f; 
    static const int vertCount = 6;
    static const int edgeCount = 9;
    static const int colliderCount = 4;
    static const int E[] = {0, 1, 0, 2, 1, 2, 0, 3, 2, 3, 0, 4, 1, 4, 1, 5, 2, 5};
    static const float L[] = {60, 60, 60, 60, 60, 60, 60, 60, 60};
    static const Vector2 points[] = {Vector2(0, 0), Vector2(0, 0), Vector2(240, 240), Vector2(240, 240)};
    static const Vector2 normals[] = {Vector2(1, 0), Vector2(0, 1), Vector2(-1, 0), Vector2(0, -1)};

    static Vector2 V[] = {Vector2(300, 200), Vector2(100, 205), Vector2(150, 100), Vector2(100, 200), Vector2(100, 210), Vector2(200, 100)};
    static Vector2 X[] = {Vector2(50, 50), Vector2(80, 50), Vector2(65, 75), Vector2(90, 90), Vector2(90, 30), Vector2(90, 60)};
    static Vector2 X_hat[vertCount];

    if (first_launch) {
      endt = micros();
      first_launch = false;
    }

    for (auto i = 0; i < vertCount; ++i) {
        V[i] = V[i];
        X_hat[i] = X[i] + V[i] * dt;
        X[i] = X_hat[i];
    }

    Vector2 g[vertCount];


    //compute gradient
    for (int i = 0; i < vertCount; ++i) {
      g[i] = mass * (X[i] - X_hat[i]) / dt / dt;
    }

    Vector2 dx;
    float l, dx_mag;
    for (int i = 0; i < edgeCount * 2; i += 2) {
      dx = X[E[i]] - X[E[i + 1]];
      dx_mag = dx.magnitude();
      l = L[i / 2];
      g[E[i]] += springK * (1 - l / dx.magnitude()) * dx;
      g[E[i + 1]] -= springK * (1 - l / dx.magnitude()) * dx;
    }

    float a = mass / dt / dt + 4 * springK;
    for (int i = 0; i < vertCount; ++i) {
      X[i] -= g[i] / a;
    }

    for (int i = 0; i < vertCount; ++i) {
      V[i] += (X[i] - X_hat[i]) / dt;
    }

    //collision
    for (int i = 0; i < vertCount; ++i) {
      for (int j = 0; j < colliderCount; ++j) {
        if ((X[i] - points[j]).Dot(normals[j]) < 0) {
          if (V[i].Dot(normals[j]) < 0) {
            V[i] -= 2 * normals[j] * V[i].Dot(normals[j]);
          }
        }
      }
    }

    tft->fillRect((int16_t)(xmin - 7), (int16_t)(ymin - 7), (int16_t)(xmax - xmin + 14), (int16_t)(ymax - ymin + 14), ST77XX_BLACK);
    if (debug) {
    tft->setCursor(0, 0);
    tft->setTextColor(ST77XX_WHITE);
    tft->setTextSize(2);
    tft->fillRect(0, 0, 240, 40, ST77XX_BLACK);
    tft->printf("Frametime: %.3gms, %.3gFPS", dt * 1000, 1 / dt);

    }
    for (int i = 0; i < vertCount; ++i) {
      tft->fillCircle((int16_t)X[i][0], (int16_t)X[i][1], 5, ST77XX_WHITE);
    }
    for (int i = 0; i < edgeCount * 2; i += 2) {
        tft->drawLine((int16_t)X[E[i]][0], (int16_t)X[E[i]][1], (int16_t)X[E[i + 1]][0], (int16_t)X[E[i + 1]][1], ST77XX_WHITE);
    }
    xmin = 240; ymin = 240; xmax = 0; ymax = 0;
    for (int i = 0; i < vertCount; ++i) {
        if (X[i][0] < xmin) xmin = X[i][0];
        if (X[i][1] < ymin) ymin = X[i][1];
        if (X[i][0] > xmax) xmax = X[i][0];
        if (X[i][1] > ymax) ymax = X[i][1];
    }
    // spur
    if (V[0].magnitude() < 80 && V[1].magnitude() < 80 && V[2].magnitude() < 80) {
      V[0][0] += 300;
      V[1][1] += 300;
      V[2][0] += 300;
      V[2][1] += 300;
    }
    dt = (micros() - endt) / 1000000.0;
    endt = micros();
}

#endif
void setup() {
  // put your setup code here, to run once:
  initializePins();
  initializePeripherals();
  delay(3000);
  Serial.println("Start detecting.");
}

componentType component;



void loop() {
  component = decideComponent();
  switch (component) {
    case None:

      Serial.println("NO LOAD.");
      break;
    case Resistor:
      Serial.println("Resistor detected.");
      measureResistor();
      break;
    case Capacitor:
      Serial.println("Capacitor detected.");
      measureCapacitor();
      break;
    case PNP:
      Serial.println("PNP BJT detected.");
      measurePNP();
      break;
    case NPN:
      Serial.println("NPN BJT detected.");
      measureNPN();
      while (decideComponent() == NPN)
        ;
      break;
    case PMOS:
      Serial.println("P-Channel MOSFET detected.");
      break;
    case NMOS:
      Serial.println("N-Channel MOSFET detected.");
      break;
    case Diode:
      Serial.println("Diode detected.");
      measureDiode();
      while (decideComponent() == Diode)
        ;
      break;
    default:
      break;
  }

  delay(100);

  // wait until component is connected
  setHighImpedance();
  double highVoltage = repeatSample(highInput, 16, 0);
  double tempVoltage = repeatSample(highInput, 16, 0);
  uint32_t cnt;
  while (tempVoltage - highVoltage < 0.1 && tempVoltage - highVoltage > -0.1 &&
         highVoltage > 3) {
    cnt = (cnt + 1) % 16;
    digitalWrite(baseHigh, (cnt >= 8) ? HIGH : LOW);
    digitalWrite(low100k, (cnt >= 12 || cnt < 4) ? HIGH : LOW);
    tempVoltage = repeatSample(highInput, 16, 10);

#ifdef withDisplay
    if (currentMode != None && currentMode != NPN && currentMode != Diode) {
      currentMode = None;
      tft->fillScreen(ST77XX_BLACK);

      tft->setCursor(50, 60);
      tft->setTextColor(ST77XX_WHITE);
      tft->setTextSize(3);
      tft->printf("NO LOAD.");
    } else if (currentMode == None) {
      drawDynamicTriangle();
      delay(12);
    }
#endif
  }
  delay(500);
}

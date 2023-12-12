/*
    Component Paramenter Measurement Client
    Term project for AI2618
    Copyright © 2023 Fangyuan Zhou, All Rights Reserved.
    Shanghai Jiao Tong University, Artificial Intelligence
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "X9C.h"

X9C* resB = nullptr;
X9C* resC = nullptr;

#define withDisplay

#ifdef withDisplay

    #define TFT_CS        D9
    #define TFT_RST       D12 // Or set to -1 and connect to Arduino RESET pin
    #define TFT_DC        D10
    #define TFT_MOSI      D11  // Data out
    #define TFT_SCLK      D13  // Clock out

    Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#endif

// Analog Pin Configuration

#define highInput A3
#define highInputSense A1
#define lowInput A5
#define baseInputH A2
#define baseInputL A4
#define baseHigh D3
#define high1k D4
#define high100k A0
#define low1k A6
#define low100k A7
#define collectorHigh D0
#define collectorLow D1

// Digital Pin Configuration

#define X9C_INC D3
#define X9C_UD D2
#define X9C_CSb D5
#define X9C_CSc D6

#define MOS_Gate
#define MOS_Source
#define MOS_Drain

const bool debug = true;

const size_t adc_buffer_size = 4096;
int adc_buffer[adc_buffer_size];
unsigned long time_buffer[adc_buffer_size];

// calibration data
const double Vcc = 3.249;
const double high1k_value = 990.31;
const double high100k_value = 98976;
const double low1k_value = 997.35;
const double low100k_value = 101668;
const double baseSense_value = 9972.1;
const double highSense_value = 996.42;

double highResistance;
double lowResistance;

enum componentType {
    None, 
    Capacitor, 
    Resistor, 
    PNP, 
    NPN, 
    PMOS, 
    NMOS, 
    Diode
};

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
        tft.init(240, 240);
        tft.setSPISpeed(80000000);
        tft.fillScreen(ST77XX_BLACK);

        tft.setCursor(40, 90);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(4);
        tft.println("UIMicro");
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
    highResistance = (high100k_value * high1k_value) / (high100k_value + high1k_value);
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
double repeatSampleDifferential(uint8_t pinH, uint8_t pinL, size_t sampleTimes, uint32_t delay_us) {
    long long sum = 0;
    for (int i = 0; i < sampleTimes; ++i) {
        delayMicroseconds(delay_us);
        sum = sum + analogReadMilliVolts(pinH) - analogReadMilliVolts(pinL); //strange problem. += doesn't work.
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
    while (analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput) > 5 && micros() - start < timeout);
    setHighImpedance();
    int differentialVoltage = 0;
    while (differentialVoltage < lowerLimit) {
        differentialVoltage = analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput);
        // differentialVoltage = repeatSampleRaw(highInput, lowInput, 16);
    }
    start = micros();
    
    while (differentialVoltage < upperLimit) {
        differentialVoltage = analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput);
        // differentialVoltage = repeatSampleRaw(highInput, lowInput, 16);
    }
    end = micros();
    double capacitance = (double)(end - start) / (highResistance + lowResistance) / 1.64866;
    // if (capacitance < 0.5) {
    //     // re-measure
    //     setZero();
    //     while (analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput) > 5);
    //     start = micros();
    //     setHighImpedance();
    //     for (int i = 0; i < 4096; ++i) {
    //         adc_buffer[i] = analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput);
    //         time_buffer[i] = micros() - start;
    //     }
    //     for (int i = 0; i < 4096; ++i) {
    //         Serial.printf("%lf %d\n", time_buffer[i] / 1000000.0, adc_buffer[i]);
    //     }
    // }
    return capacitance;
}

void measureCapacitor() {
    Serial.println("Measure Capacitor");

    double capacitance = _measureCapacitor();

    Serial.printf("Capacitance: %lf uF\n", capacitance);

    Serial.printf("Capacitance: %lf\n", capacitance);
    
    #ifdef withDisplay
        if (currentMode != Capacitor) {
            currentMode = Capacitor;
            tft.fillScreen(ST77XX_BLACK);

            tft.setCursor(50, 60);
            tft.setTextColor(ST77XX_WHITE);
            tft.setTextSize(3);
            tft.printf("Capacitor");

        }
        else {
            tft.fillRect(0, 110, 240, 40, ST77XX_BLACK);
        }

        if (capacitance > 1) {
            tft.setCursor(40, 110);
            tft.printf("%.*g uF", 4, capacitance);
        }
        else {
            tft.setCursor(40, 110);
            tft.printf("%.*g nF", 4, capacitance * 1e3);
        }
    #endif

    setZero();
    while (analogReadMilliVolts(highInput) - analogReadMilliVolts(lowInput) > 5);
}

double _measureResistor() {
    setHighImpedance();
    double highVoltage = repeatSample(highInput, 5120, 10);
    double lowVoltage = repeatSample(lowInput, 5120, 10);
    // double supplyVoltage = lowVoltage / lowResistance * highResistance + highVoltage;
    double measuredResistance = (highVoltage - lowVoltage) / (lowVoltage / lowResistance);
    if (measuredResistance < 10000) {
        setLowImpedance();
        highVoltage = repeatSample(highInput, 256, 100);
        lowVoltage = repeatSample(lowInput, 256, 100);
        measuredResistance = (highVoltage - lowVoltage) / (lowVoltage / lowResistance);
    }
    if (debug) {
        Serial.printf("Highside Voltage: %lf, Lowside Voltage: %lf, Resistance: %lf\nWaiting for serial.", highVoltage, lowVoltage, measuredResistance);
        while (!Serial.available());
        Serial.read();
    }
    return measuredResistance;
}

void measureResistor() {
    double resistance = _measureResistor();
    Serial.printf("Resistance: %lf\n", resistance);

    #ifdef withDisplay
        if (currentMode != Resistor) {
            currentMode = Resistor;
            tft.fillScreen(ST77XX_BLACK);

            tft.setCursor(50, 60);
            tft.setTextColor(ST77XX_WHITE);
            tft.setTextSize(3);
            tft.printf("Resistor");

        }
        else {
            tft.fillRect(0, 110, 240, 40, ST77XX_BLACK);
        }

        if (resistance > 1000) {
            tft.setCursor(20, 110);
            tft.printf("%.*g kOhms", 4, resistance / 1000);
        }
        else {
            tft.setCursor(30, 110);
            tft.printf("%4.*g Ohms", resistance);
        }

    #endif
}

void measurePNP() {
    double beta;

    //set pin mode to measure PNP   
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
    double baseCurrent = repeatSampleDifferential(baseInputL, baseInputH, 512, 10) / baseSense_value;
    beta = collectorCurrent / baseCurrent;

    Serial.printf("beta: %lf\n", beta);

    #ifdef withDisplay

    #endif
}

void measureNPN() {
    double beta;

    //set pin mode to measure PNP   
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


    resB->setPos(X9C::MAX);
    while (repeatSample(lowInput, 16, 10) < 2.1 && resB->getPos() > 5) {
        delay(1);
        resB->setPos(resB->getPos() - 1);
    }

    if (debug) {
        Serial.printf("base resistor: %d\n", resB->getPos());
        while (!Serial.available());
        Serial.read();
    }

    double collectorCurrent = (Vcc - repeatSample(lowInput, 512, 10)) / low1k_value;
    double baseCurrent = repeatSampleDifferential(baseInputH, baseInputL, 512, 10) / baseSense_value;
    beta = collectorCurrent / baseCurrent;

    Serial.printf("beta: %lf\n", beta);

    // perform curve measurement
    pinMode(high1k, INPUT);
    pinMode(collectorHigh, OUTPUT);
    pinMode(collectorLow, OUTPUT);
    digitalWrite(collectorLow, LOW);
    digitalWrite(collectorHigh, HIGH);

    const int curveNum = 4;

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
    //make sure the transistor don't saturate.
    while (repeatSample(highInput, 16, 10) < 2 && resB->getPos() > 5) {
        resB->setPos(resB->getPos() - 1);
    }

    if (debug) {
        Serial.printf("base resistor: %d\n", resB->getPos());
        while (!Serial.available());
        Serial.read();
    }

    startib = repeatSampleDifferential(baseInputH, baseInputL, 64, 10) / baseSense_value;

    // while (repeatSampleDifferential(baseInputH, baseInputL, 64, 10) / baseSense_value > startib / 3 && resB->getPos() > 10) {
    //     resB->setPos(resB->getPos() - 1);
    // }

    // startib = repeatSampleDifferential(baseInputH, baseInputL, 64, 10) / baseSense_value;

    for (int curveCnt = 0; curveCnt < curveNum; curveCnt++) {
        Serial.printf("Measuring curve %d\n", curveCnt);
        while ((baseInputH, baseInputL, 64, 10) / baseSense_value > startib / curveNum * (curveNum - curveCnt) && resB->getPos() > 10) {
            resB->setPos(resB->getPos() - 1);
        }

        ibs[curveCnt] = repeatSampleDifferential(baseInputH, baseInputL, 64, 10) / baseSense_value;

        for (auto i = X9C::MIN; i <= X9C::MAX; ++i) {
            resC->setPos(i);
            ics[curveCnt][i] = repeatSampleDifferential(highInputSense, highInput, 64, 10) / highSense_value;
            uce[curveCnt][i] = repeatSample(highInput, 64, 10);
        }
    }

    Serial.printf("Measurement Complete.\n");

    for (int curveCnt = 0; curveCnt < curveNum; curveCnt++) {
        Serial.printf("Curve %d: ib = %lf\n", curveCnt, ibs[curveCnt]);
        for (int i = X9C::MIN; i <= X9C::MAX; ++i) {
            Serial.printf("ic: %lf, uce: %lf", ics[curveCnt][i], uce[curveCnt][i]);
        }
    }

    #ifdef withDisplay
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setCursor(105, 10);
        tft.printf("ib: %.*guA", 4, ibs[0] * 1000000);
    #endif

    delete[] ibs;
    for (int i = 0; i < curveNum; ++i) {
        delete[] ics[i];
        delete[] uce[i];
    }
    delete[] ics;
    delete[] uce;
}

componentType decideComponent() {
    // discharge component
    setZero();
    delay(100);
    
    double transientVoltage, finalVoltage;
    setHighImpedance();
    
    if (debug) {
        Serial.printf("fuck");
        //while (!Serial.available());
        //Serial.read();
    }
    transientVoltage = repeatSampleDifferential(highInput, lowInput, 16, 0);
    delay(200);
    finalVoltage = repeatSampleDifferential(highInput, lowInput, 16, 0);

    if (debug) {
        Serial.printf("transientVoltage, %lf, finalVoltage, %lf", transientVoltage, finalVoltage);
        //while (!Serial.available());
        //Serial.read();
    }

    if (transientVoltage - finalVoltage  < 0.08 && transientVoltage - finalVoltage > -0.08) {
        // BJT
        double collectorInitialVoltage = repeatSample(highInput, 16, 0);
        digitalWrite(baseHigh, HIGH);
        delay(10);
        double collectorFinalVoltage = repeatSample(highInput, 16, 0);
        double baseVoltage;
        if (debug) {
            Serial.printf("%lf, %lf, %lf", collectorInitialVoltage, collectorFinalVoltage, baseVoltage);
        }
        if (collectorFinalVoltage - collectorInitialVoltage < -0.5) {
            baseVoltage = repeatSample(baseInputL, 16, 0);
            if (debug) {
                Serial.printf("%lf, %lf, %lf", collectorInitialVoltage, collectorFinalVoltage, baseVoltage);
            }
            if (repeatSampleDifferential(baseInputL, lowInput, 16, 10) > 1.5) {
                return NMOS;
            }
            else {
                return NPN;
            }
        }
        if (collectorFinalVoltage - collectorInitialVoltage > 0.5) {
            digitalWrite(baseHigh, LOW);
            if (repeatSampleDifferential(highInput, baseInputL, 16, 10) < 1.5) {
                return PMOS;
            }
            else {
                return PNP;
            }
        }
        if (transientVoltage > 3.0) return None;
        // resistor & Diode
        double& highImpedanceDifferentialVoltage = transientVoltage;
        double& lowImpedanceDifferentialVoltage = finalVoltage;
        highImpedanceDifferentialVoltage = repeatSampleDifferential(highInput, lowInput, 16, 0);
        setLowImpedance();
        lowImpedanceDifferentialVoltage = repeatSampleDifferential(highInput, lowInput, 16, 0);
        if (lowImpedanceDifferentialVoltage - highImpedanceDifferentialVoltage > 0.2) {
            return Resistor;
        }
        else {
            return Diode;
        }
    }
    else {
        return Capacitor;
    }
}

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
    switch (component)
    {
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
        while (decideComponent() == NPN);
        measureNPN();
        break;
    case PMOS:
        Serial.println("P-Channel MOSFET detected.");
        break;
    case NMOS:
        Serial.println("N-Channel MOSFET detected.");
        break;
    case Diode:
        Serial.println("Diode detected.");
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
    while (tempVoltage - highVoltage <0.1 && tempVoltage - highVoltage > -0.1 && highVoltage > 3) {
        cnt = (cnt + 1) % 16;
        digitalWrite(baseHigh, (cnt >= 8) ? HIGH : LOW);
        digitalWrite(low100k, (cnt >= 12 || cnt < 4) ? HIGH : LOW);
        tempVoltage = repeatSample(highInput, 16, 10);

        #ifdef withDisplay
            if (currentMode != None && currentMode != NPN) {
                currentMode = None;
                tft.fillScreen(ST77XX_BLACK);

                tft.setCursor(50, 60);
                tft.setTextColor(ST77XX_WHITE);
                tft.setTextSize(3);
                tft.printf("NO LOAD.");
            }
            else {
                currentMode = None;
            }
        #endif
    }
    delay(500);

}

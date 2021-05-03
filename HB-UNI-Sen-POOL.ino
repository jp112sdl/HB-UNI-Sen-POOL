//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2021-03-03 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=1284STD aes=no

#define HIDE_IGNORE_MSG

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <SPI.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>
#include <ContactState.h>
#include <Switch.h>
#include <sensors/Ds18b20.h>
#include <PCF8583.h>

#include <LiquidCrystal_I2C.h>
#define LCD_ADDRESS        0x3f
#define LCD_ROWS           4
#define LCD_COLUMNS        20

#define LED_PIN              0  //PB0
#define CONFIG_BUTTON_PIN    1  //PB1
#define SC_1_PIN             2  //PB2
#define SC_2_PIN             3  //PB3
#define CC1101_CS_PIN        4  //PB4
//PB5...7 SPI
//PD0...1 TX/RX FTDI

#define CC1101_GDO0          10 //PD2
#define SW_BUTTON_PIN_1      11 //PD3
#define SW_BUTTON_PIN_2      12 //PD4
#define DS18B20_1_PIN        13 //PD5
#define DS18B20_2_PIN        14 //PD6
#define SENSOR_SWITCH_PIN    15 //PD7
#define OFF LOW
#define ON  HIGH

//PC0...1 I2C
#define SW_BUTTON_PIN_3      18 //PC2
#define SW_BUTTON_PIN_4      19 //PC3
#define RELAY_PIN_3          20 //PC4
#define RELAY_PIN_4          21 //PC5
#define RELAY_PIN_1          22 //PC6
#define RELAY_PIN_2          23 //PC7

                                //PA0
                                //PA1
                                //PA2
                                //PA3
                                //PA4
#define PRESSURE_PIN         A5 //PA5
#define ORP_SIGNAL_PIN       A6 //PA6
#define PH_SIGNAL_PIN        A7 //PA7
#define SENSOR_SWITCH_PIN     6
#define OFF LOW
#define ON  HIGH

#define ANALOG_SOCKET_VALUE 108

#define PCF8583_ADDRESS      0xA0


#define REF_VOLTAGE               3300
#define CALIBRATION_MODE_TIMEOUT  600   //seconds
#define INVERT_RELAY false

#define PEERS_PER_CHANNEL        6
#define PEERS_PER_SCCHANNEL     10
#define PEERS_PER_SwitchChannel 10



using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
  {0xF3, 0x18, 0x01},          // Device ID
  "JPPOOL0001",                // Device Serial
  {0xF3, 0x18},                // Device Model
  0x10,                        // Firmware Version
  0x53,                        // Device Type
  {0x01, 0x01}                 // Info Bytes
};

typedef AskSin<StatusLed<LED_PIN>, NoBattery, Radio<LibSPI<CC1101_CS_PIN>, CC1101_GDO0>> Hal;
Hal hal;

DEFREGISTER(UReg0, MASTERID_REGS, 0x1f, 0x20, 0x21, DREG_BACKONTIME, DREG_INTKEY)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint8_t value) const {
      return this->writeRegister(0x21, value & 0xff);
    }
    uint8_t Sendeintervall () const {
      return this->readRegister(0x21, 0);
    }

    bool Messintervall (uint16_t value) const {
      return this->writeRegister(0x1f, (value >> 8) & 0xff) && this->writeRegister(0x20, value & 0xff);
    }
    uint16_t Messintervall () const {
      return (this->readRegister(0x1f, 0) << 8) + this->readRegister(0x20, 0);
    }

    void defaults () {
      clear();
      lowBatLimit(22);
      backOnTime(60);
      Sendeintervall(18);
      Messintervall(10);
      intKeyVisible(true);
    }
};

DEFREGISTER(UReg1, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)
class UList1 : public RegList1<UReg1> {
  public:
    UList1 (uint16_t addr) : RegList1<UReg1>(addr) {}

    bool TemperatureOffsetIndex1 (uint8_t value) const { return this->writeRegister(0x05, value & 0xff); }
    uint8_t TemperatureOffsetIndex1 () const { return this->readRegister(0x05, 0); }

    bool TemperatureOffsetIndex2 (uint8_t value) const { return this->writeRegister(0x06, value & 0xff); }
    uint8_t TemperatureOffsetIndex2 () const { return this->readRegister(0x06, 0); }

    bool OrpOffset (int32_t value) const {
      return
          this->writeRegister(0x01, (value >> 24) & 0xff) &&
          this->writeRegister(0x02, (value >> 16) & 0xff) &&
          this->writeRegister(0x03, (value >> 8) & 0xff) &&
          this->writeRegister(0x04, (value) & 0xff)
          ;
    }

    int32_t OrpOffset () const {
      return
          ((int32_t)(this->readRegister(0x01, 0)) << 24) +
          ((int32_t)(this->readRegister(0x02, 0)) << 16) +
          ((int32_t)(this->readRegister(0x03, 0)) << 8) +
          ((int32_t)(this->readRegister(0x04, 0)))
          ;
    }

    bool FlowRateQFactor (uint16_t value) const { return this->writeRegister(0x07, (value >> 8) & 0xff) && this->writeRegister(0x08, value & 0xff); }
    uint16_t FlowRateQFactor () const { return (this->readRegister(0x07, 0) << 8) + this->readRegister(0x08, 0); }

    void defaults () {
      clear();
      TemperatureOffsetIndex1(7);
      TemperatureOffsetIndex2(7);
      OrpOffset(0);
      FlowRateQFactor(10);
    }
};

DEFREGISTER(Reg1, CREG_AES_ACTIVE, CREG_MSGFORPOS, CREG_EVENTDELAYTIME, CREG_LEDONTIME, CREG_TRANSMITTRYMAX)
class SCList1 : public RegList1<Reg1> {
  public:
    SCList1 (uint16_t addr) : RegList1<Reg1>(addr) {}
    void defaults () {
      clear();
      msgForPosA(1);
      msgForPosB(2);
      aesActive(false);
      eventDelaytime(0);
      ledOntime(100);
      transmitTryMax(6);
    }
};
typedef TwoStateChannel<Hal, UList0, SCList1, DefList4, PEERS_PER_SCCHANNEL> SCChannel;
typedef SwitchChannel<Hal, PEERS_PER_SwitchChannel, UList0>  SWChannel;

class LcdType {
public:
  class BacklightAlarm : public Alarm {
    LcdType& lcdDev;
  public:
    BacklightAlarm (LcdType& l) :  Alarm(0), lcdDev(l) {}
    virtual ~BacklightAlarm () {}
    void restartTimer(uint8_t sec) {
      sysclock.cancel(*this);
      set(seconds2ticks(sec));
      lcdDev.lcd.backlight();
      sysclock.add(*this);
    }

    virtual void trigger (__attribute__((unused)) AlarmClock& clock) {
      lcdDev.lcd.noBacklight();
    }
  }backlightalarm;
private:
  uint8_t backlightOnTime;
  byte degree[8] = { 0b00111, 0b00101, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };

  String tempToStr(int16_t t) {
    String s_temp = "--.-";
    s_temp = (String)((float)t / 10.0);
    s_temp = s_temp.substring(0, s_temp.length() - 1);
    //if (t < 1000 && t >= 0) s_temp = " " + s_temp;
    return s_temp;
  }

  String phToStr(uint16_t p) {
    String s_ph = " --.-";
    s_ph = (String)((float)p / 100.0);
    if (p < 1000) s_ph = " " + s_ph;
    return s_ph ;
  }

  String orpToStr(int16_t o) {
    String s_orp = "----";
    s_orp = (String)o;
    if (o >= 0) {
      if (o < 10)        s_orp = "   " + s_orp;
      else if (o < 100)  s_orp = "  "  + s_orp;
      else if (o < 1000)   s_orp = " " + s_orp;
    } else {
      if (o < -9) s_orp = " " + s_orp;
      else if (o < 0) s_orp = "  " + s_orp;
    }
    return s_orp ;
  }

public:
  LiquidCrystal_I2C lcd;
  LcdType () :  backlightalarm(*this), backlightOnTime(10), lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS){}
  virtual ~LcdType () {}

  void showMeasureValues(int16_t t1, int16_t t2, uint16_t ph, int16_t orp, uint16_t pressure, uint8_t flow) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  T1  |  PH  |  ORP ");
    lcd.setCursor(t1 > 0 ? 1:0,1);
    lcd.print(tempToStr(t1));
    lcd.setCursor(6,1);lcd.print("|");
    lcd.print(phToStr(ph));
    lcd.setCursor(13,1);lcd.print("| ");
    lcd.print(orpToStr(orp));
    lcd.setCursor(0,2);
    lcd.print("  T2  |   p  | l/min");
    lcd.setCursor(t2 > 0 ? 1:0,3);
    lcd.print(tempToStr(t2));
    lcd.setCursor(6,3);lcd.print("| ");
    lcd.print(orpToStr(pressure));
    lcd.setCursor(13,3);lcd.print("| ");
    lcd.print(orpToStr(flow));
  }

  void showCalibrationMenu(uint8_t step, uint16_t n, uint16_t a, int16_t t) {
    lcd.clear();
    switch (step) {
    case 0:
      lcd.setCursor(2,0);lcd.print(F("CALIBRATION MODE"));
      lcd.setCursor(4,1);lcd.print(F("Press button"));
      break;
    case 1:
      lcd.setCursor(0,0);lcd.print(F("Put in 7.0 solution"));
      lcd.setCursor(4,1);lcd.print(F("Press button"));
      break;
    case 2:
      lcd.setCursor(2,0);lcd.print(F("Reading 7.0 DONE"));
      lcd.setCursor(4,1);lcd.print(F("Press button"));
      break;
    case 3:
      lcd.setCursor(0,0);lcd.print(F("Put in 4.0 solution"));
      lcd.setCursor(4,1);lcd.print(F("Press button"));
      break;
    case 4:
      lcd.setCursor(2,0);lcd.print(F("Reading 4.0 DONE"));
      lcd.setCursor(4,1);lcd.print(F("Press button"));
      break;
    case 5:
      lcd.setCursor(0,0);lcd.print(F("Saving."));lcd.print(tempToStr(t));lcd.setCursor(11,0);lcd.write(byte(0));
      lcd.setCursor(0,1);lcd.print("7:");lcd.print(n);lcd.print(" 4:");lcd.print(a);
      _delay_ms(2000);
      break;
    case 6:
      lcd.setCursor(5,0);lcd.print(F("CAL FAILED"));
      lcd.setCursor(3,1);lcd.print(F("STARTING AGAIN"));
      _delay_ms(2000);
      break;
    }

  }
  void initLCD(uint8_t *serial) {
    Wire.begin();
    Wire.beginTransmission(LCD_ADDRESS);
    if (Wire.endTransmission() == 0) {
      lcd.init();
      lcd.setContrast(200);
      lcd.createChar(0, degree);
      lcd.backlight();
      lcd.setCursor(0, 0);
      lcd.print(ASKSIN_PLUS_PLUS_IDENTIFIER);
      lcd.setCursor(5, 3);
      lcd.print((char*)serial);

      if (backlightOnTime > 0) backlightalarm.restartTimer(backlightOnTime);

    } else {
      DPRINT("LCD Display not found at 0x");DHEXLN((uint8_t)LCD_ADDRESS);
    }
  }

  void setBackLightOnTime(uint8_t t) {
    backlightOnTime = t;
    if (backlightOnTime == 0)
      lcd.backlight();
    else
      lcd.noBacklight();
  }
};
LcdType lcd;

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int16_t temp, int16_t temp2, uint16_t ph, int16_t orp, uint16_t pressure, uint8_t flow) {
      Message::init(0x17, msgcnt, 0x53, BIDI | WKMEUP, 0x41, (temp >> 8) & 0xff);
      pload[0] = temp & 0xff;
      pload[1] = (ph >> 8) & 0xff;
      pload[3] =  ph & 0xff;
      pload[4] = (orp >> 8) & 0xff;
      pload[5] =  orp & 0xff;
      pload[6] = (pressure >> 8) & 0xff;
      pload[7] =  pressure & 0xff;
      pload[8] =  flow & 0xff;
      pload[9] = 0x42;
      pload[10] = (temp2 >> 8) & 0xff;
      pload[11] = temp2 & 0xff;
    }
};

class MeasureChannel : public Channel<Hal, UList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
private:
    MeasureEventMsg   msg;
    UserStorage       us;
    OneWire           dsWire1;
    OneWire           dsWire2;
    Ds18b20           ds18b20_1[1];
    Ds18b20           ds18b20_2[1];
    PCF8583           pcf8583;
    bool              ds18b20_present1;
    bool              ds18b20_present2;
    bool              phcalibrationMode;
    bool              first;
    bool              calib_valid;
    int16_t           currentTemperature1;
    int16_t           currentTemperature2;
    int16_t           calib_Temperature;
    uint8_t           phcalibrationStep;
    uint16_t          ph;
    uint16_t          pressure;
    int16_t           orp;
    uint16_t          calib_neutralVoltage;
    uint16_t          calib_acidVoltage;
    uint16_t          measureCount;
    uint32_t          ph_cumulated;
    uint32_t          pressure_cumulated;
    int32_t           orp_cumulated;
    uint32_t          temperature1_cumulated;
    uint32_t          temperature2_cumulated;
    uint8_t           flowrate;
    uint16_t          flowrate_cumulated;
  public:
    MeasureChannel () : Channel(), Alarm(seconds2ticks(3)), us(0), dsWire1(DS18B20_1_PIN), dsWire2(DS18B20_2_PIN), pcf8583(PCF8583_ADDRESS), ds18b20_present1(false), ds18b20_present2(false), phcalibrationMode(false), first(true), calib_valid(false), currentTemperature1(0), currentTemperature2(0), calib_Temperature(0), phcalibrationStep(0), ph(0), pressure(0), orp(0), calib_neutralVoltage(0), calib_acidVoltage(0), measureCount(0), ph_cumulated(0), pressure_cumulated(0), orp_cumulated(0), temperature1_cumulated(0), temperature2_cumulated(0), flowrate(0), flowrate_cumulated(0) {}
    virtual ~MeasureChannel () {}

    int16_t readTemperature1() {
      if (ds18b20_present1 == false) return 250;

      Ds18b20::measure(ds18b20_1, 1);
      DPRINT(F("1. Temperature    : "));DDECLN(ds18b20_1[0].temperature());
      return (ds18b20_1[0].temperature()) + (-14+2*this->getList1().TemperatureOffsetIndex1());
    }

    int16_t readTemperature2() {
      if (ds18b20_present2 == false) return -400;

      Ds18b20::measure(ds18b20_2, 1);
      DPRINT(F("2. Temperature    : "));DDECLN(ds18b20_2[0].temperature());
      return (ds18b20_2[0].temperature()) + (-14+2*this->getList1().TemperatureOffsetIndex2());
    }


    uint32_t readVoltage(uint8_t pin) {
      analogRead(pin);

      //Mittelwert über 5 Messungen
      uint32_t analogValue = 0;
      for (uint8_t i=0; i <5; i++) {
        _delay_ms(5);
        analogValue += analogRead(pin);
      }
      analogValue = analogValue / 5;
      DPRINT(F("analogValue ("));DDEC(pin);DPRINT(F(")  : "));DDECLN(analogValue);
      //

      uint32_t voltage = ((uint32_t)analogValue * REF_VOLTAGE * 10UL) / 1024;
      DPRINT(F("measured Voltage  : "));DDECLN(voltage);

      return voltage;
    }

    void validatePHCalibrationValues() {
      calib_valid =
          (calib_neutralVoltage > 13220 && calib_neutralVoltage < 16780)
          &&
          (calib_acidVoltage    > 18540 && calib_acidVoltage    < 22100) ;

      DPRINT("Calibration is ");DPRINTLN(calib_valid == true ? "valid":"INVALID");
    }

    void restorePHCalibrationValues() {
      calib_neutralVoltage = ((uint16_t)(us.getByte(1)) << 8) + ((uint16_t)(us.getByte(2)));
      calib_acidVoltage    = ((uint16_t)(us.getByte(3)) << 8) + ((uint16_t)(us.getByte(4)));
      calib_Temperature    = ((uint16_t)(us.getByte(5)) << 8) + ((uint16_t)(us.getByte(6)));

      DPRINTLN(F("Restored Calibration Values:"));
      DPRINT(F("-CAL neutralVoltage: "));DDECLN(calib_neutralVoltage);
      DPRINT(F("-CAL acidVoltage   : "));DDECLN(calib_acidVoltage);
      DPRINT(F("-CAL temperature   : "));DDECLN(calib_Temperature);
      validatePHCalibrationValues();
      this->changed(true);
    }

    void disablePHCalibrationMode() {
      DPRINTLN(F("Exiting Calibration Mode"));
      phcalibrationMode = false;
      sysclock.cancel(*this);
      phcalibrationStep = 0;
      this->changed(true);
      set(millis2ticks(1000));
      sysclock.add(*this);
    }

    void enablePHCalibrationMode() {
      DPRINTLN(F("Entering Calibration Mode"));
      phcalibrationMode = true;
      sysclock.cancel(*this);
      this->changed(true);
      set(seconds2ticks(CALIBRATION_MODE_TIMEOUT));
      sysclock.add(*this);
      nextCalibrationStep();
    }

    void savePHCalibrationValues() {
      DPRINTLN(F("Saving Calibration Values:"));
      DPRINT(F("-CAL neutralVoltage: "));DDECLN(calib_neutralVoltage);
      DPRINT(F("-CAL acidVoltage   : "));DDECLN(calib_acidVoltage);
      DPRINT(F("-CAL temperature   : "));DDECLN(calib_Temperature);

      us.setByte(1, (calib_neutralVoltage >> 8) & 0xff);
      us.setByte(2, (calib_neutralVoltage)      & 0xff);

      us.setByte(3, (calib_acidVoltage >> 8) & 0xff);
      us.setByte(4, (calib_acidVoltage)      & 0xff);

      us.setByte(5, (calib_Temperature >> 8) & 0xff);
      us.setByte(6, (calib_Temperature)      & 0xff);
    }

    void togglePHCalibrationMode() {
      phcalibrationMode = !phcalibrationMode;
      if (phcalibrationMode == true) enablePHCalibrationMode(); else disablePHCalibrationMode();
    }

    bool getCalibrationMode() {
      return phcalibrationMode;
    }

    void nextCalibrationStep() {
      if (phcalibrationMode == true) {
        uint32_t voltage = 0;
        DPRINT(F("CALIB STEP "));DDECLN(phcalibrationStep);
        switch (phcalibrationStep) {
        case 0:
          break;
        case 1:
          break;
        case 2:
          voltage = readVoltage(PH_SIGNAL_PIN);
          //voltage = 14900;
          if (voltage > 13220 && voltage < 16780) {
            calib_neutralVoltage = voltage;
          } else phcalibrationStep = 6;
          break;
        case 3:
          break;
        case 4:
          voltage = readVoltage(PH_SIGNAL_PIN);
          //voltage = 19900;
          if (voltage > 18540 && voltage < 22100) {
            calib_acidVoltage = voltage;
          } else phcalibrationStep = 6;
          break;
        case 5:
          calib_Temperature = readTemperature1();
          savePHCalibrationValues();
          disablePHCalibrationMode();
          break;
        }
        lcd.showCalibrationMenu(phcalibrationStep, calib_neutralVoltage, calib_acidVoltage, calib_Temperature);
        if (phcalibrationStep < 6) phcalibrationStep++; else phcalibrationStep = 1;
      } else {
        phcalibrationStep = 0;
      }
    }

    uint16_t readPH() {
      if (first) {
        restorePHCalibrationValues();
        first = false;
      }
      //Erfassen der PH-Sensor Spannung
      uint32_t measuredVoltage = readVoltage(PH_SIGNAL_PIN);

      //PH-Berechnung:
      //1.) slope
      float slope = (7.0-4.0)/((((float)calib_neutralVoltage/10.0)-1500.0)/3.0 - (((float)calib_acidVoltage / 10.0)-1500.0)/3.0);
      DPRINT(F("         SLOPE    : "));DDECLN(slope);

      //1a.) slope temperature compensation
      float slope_corrected = slope * ( ( ((float)currentTemperature1 / 10.0) +273.15) / ( ((float)calib_Temperature / 10.0)  + 273.15) );
      DPRINT(F("         SLOPECORR: "));DDECLN(slope_corrected);

      //2.) intercept
      float intercept =  7.0 - slope*(((float)calib_neutralVoltage/10.0)-1500.0)/3.0;
      DPRINT(F("         INTERCEPT: "));DDECLN(intercept);
      //3.) PH
      uint16_t _ph = ( slope*( ((float)measuredVoltage/10.0) - 1500.0 ) / 3.0 + intercept ) * 100.0; //PH Wert muss mit 100 multipliziert werden, da nur "ganze Bytes" übertragen werden können (PH 7.2 ^= 72)
      DPRINT(F("         PH       : "));DDECLN(_ph);
      return _ph;
    }

    int16_t readORP() {
      //Erfassen der ORP-Sensor Spannung
      int32_t measuredVoltage = readVoltage(ORP_SIGNAL_PIN);
      int16_t orpValue=((30L * REF_VOLTAGE) -(75*(measuredVoltage/10L)))/75;

      orpValue += this->getList1().OrpOffset();

      DPRINT(F("        ORP       : "));DDECLN(orpValue);
      return orpValue;
    }

    uint16_t readPressure() {
      uint16_t analogValue = 0;
      for (uint8_t i = 0; i < 10; i++) {
        analogValue += analogRead(PRESSURE_PIN);
        _delay_ms(5);
      }
      analogValue = analogValue / 10;

      float sensor_factor = 1.6; //1.6 for 0.5MPa Sensor, 0.75 for 1.2 MPa Sensor
      float _p = (((analogValue / 1024.0) - (ANALOG_SOCKET_VALUE / 1000.0)) / sensor_factor) * 1000;
      DPRINT(F("+Pressure  (#")); DDEC(number()); DPRINT(F(") Analogwert: ")); DDECLN(analogValue);
      DPRINT(F("+Pressure  (#")); DDEC(number()); DPRINT(F(")       mBar: ")); DDECLN(_p * 10);
      return _p > 0 ? (uint16_t)_p : 0;
    }

    uint8_t readFlowrate() {
      uint8_t lmin = 0;
      /*static unsigned long millis_lastread = 0;
      if (millis_lastread == 0) {
        pcf8583.reset();
        pcf8583.setMode(MODE_EVENT_COUNTER);
        pcf8583.setCount(0);
      } else {
        uint32_t cnt = pcf8583.getCount();
        uint32_t hz = (cnt * 1000) / (millis() - millis_lastread);
        lmin = hz* 10 / this->getList1().FlowRateQFactor();
        pcf8583.setCount(0);
      }
      millis_lastread = millis();*/
      return lmin;
    }

    void run() {
      measureCount++;

      DPRINT(F("Messung #"));DDECLN(measureCount);

      set(seconds2ticks(max((uint16_t)5,(uint16_t)device().getList0().Messintervall())));

      flowrate = readFlowrate();
      currentTemperature1 = readTemperature1();
      currentTemperature2 = readTemperature2();
      ph = readPH();
      digitalWrite(SENSOR_SWITCH_PIN, ON);
      _delay_ms(100);
      orp = readORP();
      digitalWrite(SENSOR_SWITCH_PIN, OFF);
      pressure = readPressure();

      //Anzeige der Daten auf dem LCD Display
      lcd.showMeasureValues(currentTemperature1, currentTemperature2, ph, orp, pressure, flowrate);

      flowrate_cumulated    += flowrate;
      ph_cumulated          += ph;
      orp_cumulated         += orp;
      temperature1_cumulated += currentTemperature1;
      temperature2_cumulated += currentTemperature2;
      pressure_cumulated += pressure;

      if (measureCount >= device().getList0().Sendeintervall()) {
        msg.init(device().nextcount(),
                (ds18b20_present1 == true) ? temperature1_cumulated / measureCount  : -400,
                (ds18b20_present2 == true) ? temperature2_cumulated / measureCount  : -400,
                 ph_cumulated / measureCount,
                 orp_cumulated / measureCount,
                 pressure_cumulated / measureCount,
                 flowrate_cumulated / measureCount);
        device().broadcastEvent(msg);
        measureCount = 0;
        ph_cumulated = 0;
        orp_cumulated = 0;
        temperature1_cumulated = 0;
        temperature2_cumulated = 0;
        pressure_cumulated = 0;
        flowrate_cumulated = 0;
      }
      sysclock.add(*this);
    }

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      if (phcalibrationMode == false) {
        run();
      } else {
        disablePHCalibrationMode();
      }
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      pinMode(PH_SIGNAL_PIN, INPUT);
      pinMode(ORP_SIGNAL_PIN, INPUT);
      pinMode(PRESSURE_PIN, INPUT);
      pinMode(SENSOR_SWITCH_PIN, OUTPUT);
      digitalWrite(SENSOR_SWITCH_PIN, OFF);
      ds18b20_present1 = (Ds18b20::init(dsWire1, ds18b20_1, 1) == 1);
      ds18b20_present2 = (Ds18b20::init(dsWire2, ds18b20_2, 1) == 1);
      DPRINT(F("1. DS18B20: "));DPRINTLN( ds18b20_present1 == true ? "OK":"FAIL");
      DPRINT(F("2. DS18B20: "));DPRINTLN( ds18b20_present2 == true ? "OK":"FAIL");
      sysclock.add(*this);
    }

    void setUserStorage(const UserStorage& storage) {
      us = storage;
    }

    void configChanged() {
      DPRINT(F("*1.Temperature Offset : "));DDECLN(this->getList1().TemperatureOffsetIndex1());
      DPRINT(F("*2.Temperature Offset : "));DDECLN(this->getList1().TemperatureOffsetIndex2());
      DPRINT(F("*Orp         Offset   : "));DDECLN(this->getList1().OrpOffset());
      DPRINT(F("*FlowRateQFactor      : "));DDECLN(this->getList1().FlowRateQFactor());
    }

    uint8_t status () const { return 0; }
    uint8_t flags () const {
      uint8_t flg = 0x00;
      if (phcalibrationMode == true)  flg = 0x01<<1;
      else if (calib_valid == false)  flg = 0x01<<2;
      return flg;
    }
};

class DummyChannel : public Channel<Hal, UList1, EmptyList, DefList4, 1, UList0> {
public:
  typedef Channel<Hal,EmptyList,EmptyList,DefList4,1,UList0> BaseChannel;
  uint8_t status () const { return 0; }
  uint8_t flags ()  const { return 0; }
};

class UType : public ChannelDevice<Hal, VirtBaseChannel<Hal, UList0>, 8, UList0> {
  public:
    VirtChannel<Hal, SWChannel, UList0>         channel5, channel6, channel7, channel8;
    VirtChannel<Hal, SCChannel, UList0>         channel3, channel4;
    VirtChannel<Hal, MeasureChannel, UList0>    channel1;
    VirtChannel<Hal, DummyChannel, UList0>      channel2;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, UList0>, 8, UList0> DeviceType;

    UType (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(channel1, 1);
      DeviceType::registerChannel(channel2, 2);
      DeviceType::registerChannel(channel3, 3);
      DeviceType::registerChannel(channel4, 4);
      DeviceType::registerChannel(channel5, 5);
      DeviceType::registerChannel(channel6, 6);
      DeviceType::registerChannel(channel7, 7);
      DeviceType::registerChannel(channel8, 8);
    }
    virtual ~UType () {}

    SWChannel& swChannel1 ()  {
      return channel5;
    }

    SWChannel& swChannel2 ()  {
      return channel6;
    }

    SWChannel& swChannel3 ()  {
      return channel7;
    }

    SWChannel& swChannel4 ()  {
      return channel8;
    }

    SCChannel& scChannel1 ()  {
      return channel3;
    }

    SCChannel& scChannel2 ()  {
      return channel4;
    }

    MeasureChannel& measureChannel ()  {
      return channel1;
    }


    virtual void configChanged () {
      DeviceType::configChanged();
      DPRINT(F("*Messintervall        : ")); DDECLN(this->getList0().Messintervall());
      DPRINT(F("*Sendeintervall       : ")); DDECLN(this->getList0().Sendeintervall());

      uint8_t bOn = this->getList0().backOnTime();
      DPRINT(F("*LCD Backlight Ontime : ")); DDECLN(bOn);
      lcd.setBackLightOnTime(bOn);
    }
};

UType sdev(devinfo, 0x20);

class CalibButton : public StateButton<HIGH,LOW,INPUT_PULLUP> {
  UType& device;
public:
  typedef StateButton<HIGH,LOW,INPUT_PULLUP> ButtonType;
  CalibButton (UType& dev,uint8_t longpresstime=3) : device(dev) { this->setLongPressTime(seconds2ticks(longpresstime)); }
  virtual ~CalibButton () {}
  virtual void state (uint8_t s) {
    uint8_t old = ButtonType::state();
    ButtonType::state(s);
    if( s == ButtonType::released ) {
      if (device.measureChannel().getCalibrationMode() == true) {
        device.measureChannel().nextCalibrationStep();
      } else {
        device.startPairing();
      }
    }
    else if ( s == ButtonType::pressed) {
      lcd.backlightalarm.restartTimer( sdev.getList0().backOnTime() );
    }
    else if( s == ButtonType::longreleased ) {
      device.measureChannel().togglePHCalibrationMode();
    }
    else if( s == ButtonType::longpressed ) {
      if( old == ButtonType::longpressed ) {
        if( device.getList0().localResetDisable() == false ) {
          device.reset();
        }
      }
      else {
        device.led().set(LedStates::key_long);
      }
    }
  }
};
CalibButton calibBtn(sdev);
InternalButton<UType> swbtn1(sdev,5);
InternalButton<UType> swbtn2(sdev,6);
InternalButton<UType> swbtn3(sdev,7);
InternalButton<UType> swbtn4(sdev,8);

void initPeerings (bool first) {
  if ( first == true ) {
    HMID devid;
    sdev.getDeviceID(devid);
    sdev.swChannel1().peer(Peer(devid, 5));
    sdev.swChannel2().peer(Peer(devid, 6));
    sdev.swChannel3().peer(Peer(devid, 7));
    sdev.swChannel4().peer(Peer(devid, 8));
  }
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  bool first = sdev.init(hal);
  buttonISR(calibBtn, CONFIG_BUTTON_PIN);
  buttonISR(swbtn1,SW_BUTTON_PIN_1);
  buttonISR(swbtn2,SW_BUTTON_PIN_2);
  buttonISR(swbtn3,SW_BUTTON_PIN_3);
  buttonISR(swbtn4,SW_BUTTON_PIN_4);
  sdev.scChannel1().init(SC_1_PIN);
  sdev.scChannel2().init(SC_2_PIN);
  sdev.swChannel1().init(RELAY_PIN_1, INVERT_RELAY);
  sdev.swChannel2().init(RELAY_PIN_2, INVERT_RELAY);
  sdev.swChannel3().init(RELAY_PIN_3, INVERT_RELAY);
  sdev.swChannel4().init(RELAY_PIN_4, INVERT_RELAY);
  initPeerings(first);
  sdev.initDone();
  sdev.measureChannel().setUserStorage(sdev.getUserStorage());

  uint8_t serial[11];sdev.getDeviceSerial(serial);serial[10]=0;
  lcd.initLCD(serial);
}

void loop() {
  hal.runready();
  sdev.pollRadio();
}

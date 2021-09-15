#include <Arduino.h>
#include <WiFi.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <gps.h>

// EDIT AND INSERT YOUR ABP KEYS HERE!
#include <config.h>

#ifdef V1_1
    // Battery management
    #include "axp20x.h"
    #ifndef AXP192_SLAVE_ADDRESS
        #define AXP192_SLAVE_ADDRESS 0x34
    #endif

    #define PMU_IRQ 35

    AXP20X_Class axp;
    bool pmu_irq = false;
    bool axp192_found = true;
    String baChStatus = "No charging";
#endif

#ifdef V0_7
  bool buttonpressed=false;
#endif


// if not set in config.h defaults to 1
#ifndef TTN_PORT
    #define TTN_PORT 1
#endif

// keeps the millis since last "QUEUED" state
uint32_t lastsendjob = 0;

// Thread Handle for UI
TaskHandle_t uiThreadTask;

// Delay between Lora Send
uint8_t sendInterval[] = {20, 30, 40, 10};
uint8_t sendIntervalKey = 0;

// number of packets send to TTN
uint16_t packets_send = 0;

// last packet number !doesn't survive a reset!
u4_t lmicSeqNumber = 0;

// flag to clear display only if was inverse before
bool display_is_inverse = false;

// time since last buttonpress
// bool previousButtonState = 1; // will store last Button state. 1 = unpressed, 0 = pressed
uint32_t lastkeypress = 0;
volatile bool setlock = false;

// TinyGPS Wrapper
Gps gps;

// Global Status Flags
float vbat;
String LoraStatus;
boolean hasFix = false;

static osjob_t sendjob;

uint8_t loraBuffer[9];
uint16_t dispBuffer[6];

// Empty Callbacks since we are not using OTAA
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
void do_send(osjob_t *j); // declare for onEvent

// Setup Lora Pins
const lmic_pinmap lmic_pins = {
    //.nss = 18,
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN, // was "14,"
    //.dio = {26, 33, 32},
    .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};

void beep(uint32_t time)
{
#ifdef BUZZER_PIN
  digitalWrite(BUZZER_PIN, LOW);
  delay(time);
  digitalWrite(BUZZER_PIN, HIGH);
#endif
}

// Lora Event Handling
void onEvent(ev_t ev)
{

  // last packet count
  lmicSeqNumber = LMIC.seqnoUp;

  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    LoraStatus = "SCANTIMEO";
    break;
  case EV_BEACON_FOUND:
    LoraStatus = "BEAC_FOUN";
    break;
  case EV_BEACON_MISSED:
    LoraStatus = "BEAC_MISS";
    break;
  case EV_BEACON_TRACKED:
    LoraStatus = "BEAC_TRAC";
    break;
  case EV_JOINING:
    LoraStatus = "JOINING";
    break;
  case EV_JOINED:
    LoraStatus = "JOINED";
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    LoraStatus = "RFU1";
    break;
  case EV_JOIN_FAILED:
    LoraStatus = "JOIN_FAIL";
    break;
  case EV_REJOIN_FAILED:
    LoraStatus = "REJOIN_FA";
    break;
  case EV_TXCOMPLETE:
    packets_send++;
    LoraStatus = "TXCOMPL";
    digitalWrite(BUILTIN_LED, LOW);
    beep(50);
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      LoraStatus = "Recvd Ack";
    }
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks((sendInterval[sendIntervalKey])), do_send);
    break;
  case EV_LOST_TSYNC:
    LoraStatus = "LOST_TSYN";
    break;
  case EV_RESET:
    LoraStatus = "RESET";
    break;
  case EV_RXCOMPLETE:
    LoraStatus = "RXCOMPL";
    break;
  case EV_LINK_DEAD:
    LoraStatus = "LINK_DEAD";
    break;
  case EV_LINK_ALIVE:
    LoraStatus = "LINK_ALIV";
    break;
  default:
    LoraStatus = "UNKNOWN";
    break;
  }
}

void do_send(osjob_t *j)
{

  Serial.println("do send");
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    LoraStatus = "TXRXPEND";
  }
  else
  {
    if (hasFix)
    {
      // Prepare upstream data transmission at the next possible time.
      if (gps.buildPacket(loraBuffer))
      {

        LMIC_setTxData2(TTN_PORT, loraBuffer, sizeof(loraBuffer), 0);
        digitalWrite(BUILTIN_LED, HIGH);
        LoraStatus = "QUEUED";

        //keep time for timeout
        lastsendjob = millis();
      }
      else
      {
        LoraStatus = "GPS FAIL";
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
      }
    }
    else
    {
      LoraStatus = "NO FIX";
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
}

// UI Thread
void uiThread(void *parameter)
{

  // Initialize Display
#if defined V0_7 || defined V1_0 || defined V1_1
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
#endif

#ifdef HELTEC_WS
  //todo init for Onboard 0.49-inch 64*32
  U8X8_SSD1306_64X32_1F_SW_I2C u8x8(OLED_SCL, OLED_SDA, OLED_RST);
#endif

  u8x8.begin();
  u8x8.setPowerSave(0);

#ifdef V1_1

  /* Init AXP192 to get access to Battery management.
     We have to do that here because we have to use the Wire instance which
     is already in use by OLED and not accessible outside of this Thread.
     There are a few flags below which I have commented out. I don't know
     whether these are relly needed.
     The code is almost from https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam/blob/master/src/LilyGO-T-Beam.ino
     reduced to a minimum.
  */
  axp.begin(Wire, AXP192_SLAVE_ADDRESS);

  // if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
  //     Serial.println("AXP192 Begin PASS");
  // } else {
  //     Serial.println("AXP192 Begin FAIL");
  // }

  // Lora Power ON
  // Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setLDO2Voltage(3300); //LORA VDD set 3v3
  // Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");

  // GPS Power ON
  // Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setLDO3Voltage(3300); //GPS VDD      3v3
  // Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");

  // OLED Power ON
  // Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  axp.setDCDC1Voltage(3300); //esp32 core VDD    3v3
  // Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");

  // ??? Power ON
  // Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
  // axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  // Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");

  // Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");

  // Exten Power ON
  // Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
  // axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  // Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

  // setting the interrupt to handle state changes on AXP
  pinMode(PMU_IRQ, INPUT_PULLUP);
  attachInterrupt(
      PMU_IRQ, [] {
        pmu_irq = true;
      },
      FALLING);

  // enable ADC Measuring
  axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
  axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
  axp.clearIRQ();

  // set initial charge state
  if (axp.isChargeing())
  {
    baChStatus = "Charging";
  }
#endif

  // Endless Loop for Display Thread
  bool lastState = hasFix;
  bool cleared_setscreen = false;
  bool cleared_normscreen = false;
  for (;;)
  {
#ifdef V0_7
  int Push_button_state = digitalRead(BUTTON_R);
  if ( Push_button_state == HIGH ){
     buttonpressed=false;
  }else{
    setlock = true;
    lastkeypress = millis();
    if(!buttonpressed){
      buttonpressed=true;
      sendIntervalKey++;
      if (sendIntervalKey >= (sizeof(sendInterval)/sizeof(*sendInterval))){
        sendIntervalKey = 0;
      }
    }
  }
#endif

#ifdef V1_1
    // interrupt is triggered if charge state changes (cable plugged/ unplugged)
    if (axp192_found && pmu_irq)
    {
      pmu_irq = false;
      axp.readIRQ();
      if (axp.isChargingIRQ())
      {
        baChStatus = "Charging";
      }
      else
      {
        baChStatus = "No Charging";
      }
      if (axp.isVbusRemoveIRQ())
      {
        baChStatus = "No Charging";
      }
      digitalWrite(2, !digitalRead(2));
      axp.clearIRQ();
    }
    // set Bat Voltage for legacy code
    vbat = axp.getBattVoltage() / 1000.0;
#endif

    // Settings screen, shown 5secs after last keypress. Reset in main loop
    if (setlock)
    {

      cleared_normscreen = false;

      // avoid flicker
      if (!cleared_setscreen)
      {
        u8x8.clear();
        u8x8.setInverseFont(0);
        cleared_setscreen = true;
      }
      u8x8.home();
      u8x8.setFont(u8x8_font_victoriabold8_r);
      u8x8.println("Interval");
      u8x8.setFont(u8x8_font_courB18_2x3_n);
      u8x8.println(sendInterval[sendIntervalKey]);
      delay(50);
    }
    else
    {

      cleared_setscreen = false;
      if (!cleared_normscreen)
      {
        u8x8.clear();
        cleared_normscreen = true;
      }

      //u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
      u8x8.setFont(u8x8_font_victoriabold8_r); // <- this font is better readable IMHO

      if (lastState != hasFix)
      {
        lastState = hasFix;
        u8x8.clear();
      }
      else
      {
        u8x8.home();
      }

      if (hasFix)
      {
#if defined V0_7 || defined V1_0 || defined V1_1

        if (LoraStatus == "QUEUED")
        {
          u8x8.setInverseFont(1);
          display_is_inverse = true;
        }
        else
        {
          u8x8.setInverseFont(0);
          // this is to avoid white spaces on display
          if (display_is_inverse)
          {
            u8x8.clear();
            display_is_inverse = false;
          }
        }

        u8x8.print("HDOP:  ");
        u8x8.println(dispBuffer[4] / 10.0);
        u8x8.print("Sat:   ");
        u8x8.print(dispBuffer[5]); // NMEA Sentence GPGSV, Element 3 (Sat in View)
        u8x8.print("/");
        u8x8.println(dispBuffer[0]); // Number of satellites in use (u32)
        u8x8.print("Int:   ");
        u8x8.print(sendInterval[sendIntervalKey]);
        u8x8.print("  ");
        u8x8.print(packets_send);
        u8x8.println("x");
        u8x8.print("Speed: ");
        u8x8.print(dispBuffer[1]);
        u8x8.println("km/h");
        u8x8.print("Alt:   ");
        u8x8.print(dispBuffer[3]);
        u8x8.println("m");
        u8x8.print("Lora:  ");
        u8x8.println(LoraStatus);
        u8x8.print("Bat:   ");
        u8x8.print(vbat, 2);
        u8x8.println("V");
#ifdef V1_1
        // Charge status
        u8x8.println(baChStatus);
#endif
#endif

#ifdef HELTEC_WS
        u8x8.setInverseFont(0);
        u8x8.print("H: ");
        u8x8.println(dispBuffer[4] / 10.0);
        u8x8.print("S: ");
        u8x8.print(dispBuffer[5]);
        u8x8.print("/");
        u8x8.println(dispBuffer[0]);
        u8x8.print("X: ");
        u8x8.println(packets_send);
        u8x8.println(LoraStatus);
#endif
      }
      else
      {

#if defined V0_7 || defined V1_0 || defined V1_1
        u8x8.setInverseFont(0);
        u8x8.println("NO FIX");
        u8x8.print("Sat:   ");
        u8x8.print(dispBuffer[5]);
        u8x8.print("/");
        u8x8.println(dispBuffer[0]);
        u8x8.print("Int:   ");
        u8x8.print(sendInterval[sendIntervalKey]);
        u8x8.print("  ");
        u8x8.print(packets_send);
        u8x8.println("x");
        u8x8.println("");
        u8x8.println("");
        u8x8.println("");
        u8x8.print("Bat:   ");
        u8x8.print(vbat, 2);
        u8x8.println("V");
#ifdef V1_1
        // Charge status
        u8x8.println(baChStatus);
#endif
#endif

#ifdef HELTEC_WS
        u8x8.setInverseFont(0);
        u8x8.println("NO FIX");
        u8x8.print("S: ");
        u8x8.print(dispBuffer[5]);
        u8x8.print("/");
        u8x8.println(dispBuffer[0]);
        u8x8.print("X: ");
        u8x8.println(packets_send);
        u8x8.println(LoraStatus);
#endif
      }
      delay(50);
    }
  }
}

void init_LMIC()
{
  LMIC_reset();
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
// TTN defines an additional channel at 869.525Mhz using SF9 for class B
// devices' ping slots. LMIC does not have an easy way to define set this
// frequency and support for class B is spotty and untested, so this
// frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
// Set up the channels used in your country. Only two are defined by default,
// and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
// LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
// LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

// ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
// Set up the channels used in your country. Three are defined by default,
// and they cannot be changed. Duty cycle doesn't matter, but is conventionally
// BAND_MILLI.
// LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

// ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
// Set up the channels used in your country. Three are defined by default,
// and they cannot be changed. Duty cycle doesn't matter, but is conventionally
// BAND_MILLI.
// LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

// ... extra definitions for channels 3..n here.
#else
#error Region not supported
#endif

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Disable Data Rate Adaptation, for Mapping we want static SF7
  LMIC_setAdrMode(0);
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);
  // Don't do Link Checks
  LMIC_setLinkCheckMode(0);

  // set last packet count after re-init
  LMIC.seqnoUp = lmicSeqNumber;

  delay(500);
  do_send(&sendjob);
}

// interrupt handler for buttonpress
#ifndef V0_7
void IRAM_ATTR isr()
{
  setlock = true;
  lastkeypress = millis();
  sendIntervalKey++;
  if (sendIntervalKey >= (sizeof(sendInterval) / sizeof(*sendInterval)))
  {
    sendIntervalKey = 0;
  }
}
#endif

// void handleButton()
// {
//   bool inp = digitalRead(BUTTON_R);
//   if (inp == 0)
//   {
//     if (inp != previousButtonState)
//     {
//       Serial.printf("%lu: Button pressed short\n", millis());
//       lastButtonTimer = millis();
//     }
//     if ((millis() - lastButtonTimer >= 5000))
//     {
//       Serial.printf("%lu: Button pressed long\n", millis());
//       lastButtonTimer = millis();
//     }
//     // Delay a little bit to avoid bouncing
//     delay(50);
//   }
//   previousButtonState = inp;
// }

void setup()
{

  Serial.begin(115200);

  // Setup Hardware
#if defined V0_7 || defined V1_0
  pinMode(BAT_PIN, INPUT);
#endif

#ifdef V0_7
  pinMode(BUTTON_R, INPUT);
#endif

#ifdef BUZZER_PIN
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  beep(500);
#endif

  // Init GPS
  gps.init();

  // Wifi and BT Off
  WiFi.mode(WIFI_OFF);
  btStop();

  // Start UI-Thread on Second Core
  xTaskCreatePinnedToCore(uiThread, "uiThread", 10000, NULL, 1, &uiThreadTask, 0);

  // Builtin LED will be used to indicate LoRa Activity
  pinMode(BUILTIN_LED, OUTPUT);

  // LoRa Init
  os_init();
  init_LMIC();

  digitalWrite(BUILTIN_LED, LOW);

#ifndef V0_7
  // use an interrupt for buttonpress
  pinMode(BUTTON_R, INPUT);
  attachInterrupt(BUTTON_R, isr, FALLING);
#endif
}

void loop()
{

//Bord Rev1.1 cannot measure BAT this way. Measure is done in uiThread
#if defined V0_7 || defined V1_0
  vbat = (float)(analogRead(BAT_PIN)) / 4095 * 2 * 3.3 * 1.1;
#endif

  // Button
  // handleButton();

  // check whether LMIC hangs and re-init
  if (LoraStatus == "QUEUED" && lastsendjob > 0 && (millis() - lastsendjob > 10000))
  {
    init_LMIC();
    packets_send = 0;
    LoraStatus = "LMIC_RST";
  }

  // check settings lock for display
  if (lastkeypress > 0 && (millis() - lastkeypress > 5000))
  {
    setlock = false;
    lastkeypress = 0;

    // Delay a little bit to avoid bouncing
    delay(50);
  }

  hasFix = gps.checkGpsFix();
  gps.gdisplay(dispBuffer);
  os_runloop_once();
  yield();
}

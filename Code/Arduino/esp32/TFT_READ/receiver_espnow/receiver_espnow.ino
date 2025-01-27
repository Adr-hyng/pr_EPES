// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "receiver_espnow_GSLC.h"
#include <esp_now.h>
#include <WiFi.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "HX711.h"
#include <math.h>
#include <functional>

// ------------------------------------------------
// Program Globals
class Timer {
public:
  Timer(unsigned long interval, 
  std::function<void()> callback
  ) : interval(interval), callback(callback), 
  lastExecutionTime(0) {}
  void check() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastExecutionTime >= interval) {
      lastExecutionTime = currentMillis;
      callback();
    }
  }
private:
  unsigned long interval;
  std::function<void()> callback;
  unsigned long lastExecutionTime;
};

// Must match the sender structure
uint8_t ESPTMacAddr[] = {
  0x34, 0x5f, 0x45, 
  0xa9, 0xc1, 0x08
};
esp_now_peer_info_t peerInfo;
typedef struct struct_message {
  short ContainerCap;
  short Mode;
  short CurTemperature;
  bool isPushed;
  short SelTemp_MRange;
  bool heaterActivated;
  bool childLockActivated;
  bool resetMode;
} struct_message;
struct_message myData;
bool redrawRequired = false;

// Temperature Sensor
const int oneWireBus = 22;  
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Analog Joystick
const int joystick_y_pin = 34;

// Weight Sensor
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 33;
HX711 scale(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
float calibrationFactor = -3084.7;
// ------------------------------------------------

// Save some element references for direct access
//<Save_References !Start!>
gslc_tsElemRef* m_pElemOutCurTemp = NULL;
gslc_tsElemRef* m_pElemOutSelTemp = NULL;
gslc_tsElemRef* m_pElemOutThermometerIMG= NULL;
gslc_tsElemRef* m_pElemXRingGauge1= NULL;
gslc_tsElemRef* m_pElemHeaterState= NULL;
//<Save_References !End!>

// Define debug message function
static int16_t DebugOut(char ch) {
  if (ch == (char)'\n') Serial.println(""); 
  else Serial.write(ch); 
  return 0;
}

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
void initializeScale() {
  scale.set_scale();
  scale.tare();
  long zero_factor = scale.read_average();
  delay(2000);
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void execEveryMillis(unsigned long interval, std::function<void()> callback) {
  static unsigned long lastExecutionTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastExecutionTime >= interval) {
    lastExecutionTime = currentMillis;
    callback();
  }
}
void handleWeightMeasurement() {
  scale.set_scale(calibrationFactor);
  float weight = scale.get_units(45); 
  if(weight<0) { weight=0.00;}
  myData.ContainerCap = ((short)mapFloat(weight, 0.00, 91.00, 0.00, 100.00)) - 20;
  if(myData.ContainerCap<0) { myData.ContainerCap=0;}
}
void sendData() {
  myData.Mode = myData.Mode;
  myData.childLockActivated = myData.childLockActivated;
  esp_err_t result = esp_now_send(ESPTMacAddr, (uint8_t *)&myData, sizeof(myData));
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
  redrawRequired = true;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  return;
}
bool onStateChange(bool& currentState, bool newValue) {
  static bool previousState = false;
  if (currentState != newValue) {
    previousState = currentState;
    currentState = newValue;
    return true;
  }
  return false;
}
void UpdateThermometerImage() {
    static short lastSelTemp_MRange = -1;

    if (myData.SelTemp_MRange != lastSelTemp_MRange) {
        lastSelTemp_MRange = myData.SelTemp_MRange;

        if (myData.SelTemp_MRange == 85) {
            UpdateBitmapImage(&m_gui, m_pElemOutThermometerIMG, THERMOMETER);
        } else if (myData.SelTemp_MRange == 75) {
            UpdateBitmapImage(&m_gui, m_pElemOutThermometerIMG, THERMOMETER_1);
        } else {
            UpdateBitmapImage(&m_gui, m_pElemOutThermometerIMG, THERMOMETER_2);
        }

        static char TSelTemp[4] = "";
        snprintf(TSelTemp, sizeof(TSelTemp), "%d", myData.SelTemp_MRange);
        if (m_pElemOutSelTemp != NULL) {
          gslc_ElemSetTxtStr(&m_gui, m_pElemOutSelTemp, TSelTemp);
        } 
    }
}
void UpdateBitmapImage(gslc_tsGui* pGui, gslc_tsElemRef* pElemRef, const unsigned short* newBitmap) {
    if (pElemRef == nullptr) return;
    gslc_tsElem* pElem = gslc_GetElemFromRef(pGui, pElemRef);
    gslc_ElemSetRedraw(pGui, pElemRef, GSLC_REDRAW_FULL);
    gslc_tsImgRef updImgRef = gslc_GetImageFromProg((const unsigned char*)newBitmap, GSLC_IMGREF_FMT_BMP24);
    gslc_ElemSetImage(pGui, pElemRef, updImgRef, updImgRef);
    gslc_ElemSetRedraw(pGui, pElemRef, GSLC_REDRAW_FULL);
}

// Common Button callback
bool CbBtnCommon(void* pvGui,void *pvElemRef,gslc_teTouch eTouch,int16_t nX,int16_t nY) {
  // Typecast the parameters to match the GUI and element types
  gslc_tsGui*     pGui     = (gslc_tsGui*)(pvGui);
  gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)(pvElemRef);
  gslc_tsElem*    pElem    = gslc_GetElemFromRef(pGui,pElemRef);

  if ( eTouch == GSLC_TOUCH_UP_IN ) {
    // From the element's ID we can determine which button was pressed.
    switch (pElem->nId) {
//<Button Enums !Start!>
      case HeaterIndicator:
        break;
//<Button Enums !End!>
      default:
        break;
    }
  }
  return true;
}
void setup()
{
  // ----------------------4--------------------------
  // Initialize
  // ------------------------------------------------
  myData.ContainerCap = -1;
  Serial.begin(115200);
  sensors.begin();
  initializeScale();
  gslc_InitDebug(&DebugOut);

  // ------------------------------------------------
  // Create graphic elements
  // ------------------------------------------------
  InitGUIslice_gen();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  memcpy(peerInfo.peer_addr, ESPTMacAddr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) return;

  myData.resetMode = 1;
  esp_err_t result = esp_now_send(
    ESPTMacAddr, 
    (uint8_t *)&myData, 
    sizeof(myData)
  );
  delay(50);
  myData.resetMode = 0;
}

Timer timer1(500, []() {
  handleWeightMeasurement();
  Serial.print(1);
  Serial.print(",");
  Serial.print(myData.ContainerCap);
  Serial.print(",");
  Serial.print(myData.Mode);
  Serial.print(",");
  Serial.print(myData.CurTemperature);
  Serial.print(",");
  Serial.print(myData.isPushed);
  Serial.println();
});

Timer timer2(100, []() {
  int y_adc_val; 
  float y_volt;
  y_adc_val = analogRead(joystick_y_pin);
  y_volt = ( (y_adc_val * 3.3 ) / 4095 );
  myData.isPushed = (y_volt <= 1.3 && 
                     y_volt >= 0.6) || 
                     y_volt <= 0.4;
});
Timer timer3(1000, []() {
  sensors.requestTemperatures(); 
  myData.CurTemperature = sensors.getTempCByIndex(0);
  static char TCurTemp[4] = "";
  snprintf(TCurTemp, sizeof(TCurTemp), "%d", myData.CurTemperature);
  if (m_pElemOutCurTemp != NULL) {
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutCurTemp, TCurTemp);
  }
});

Timer timer4(200, []() {
  sendData();
});

// -----------------------------------
// Main event loop
// -----------------------------------
void loop()
{
  // ------------------------------------------------
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    int dummy;
    int tempSelTemp_MRange, 
    tempHeaterActivated, 
    tempChildLockActivated, 
    tempResetMode;
    int parsedValues = sscanf(
      receivedData.c_str(), 
      "%d,%d,%d,%d,%d",
      &dummy,
      &tempSelTemp_MRange,
      &tempHeaterActivated,
      &tempChildLockActivated,
      &tempResetMode
    );
    if (parsedValues == 5) {
      if (dummy == 0) {
        myData.SelTemp_MRange = tempSelTemp_MRange;
        if (onStateChange(myData.resetMode, tempResetMode)) ESP.restart();
        UpdateThermometerImage();
        if(onStateChange(myData.heaterActivated, tempHeaterActivated)) {
          m_pElemHeaterState = gslc_PageFindElemById(&m_gui, gslc_GetPageCur(&m_gui), HeaterIndicator);
          gslc_ElemSetGlowEn(&m_gui, m_pElemHeaterState, myData.heaterActivated);
          gslc_ElemSetGlow(&m_gui, m_pElemHeaterState, myData.heaterActivated);
        }
        if (onStateChange(myData.childLockActivated, 
        tempChildLockActivated)
        ) sendData();
      }
    }
  }
  timer1.check();
  timer2.check();
  timer3.check();
  timer4.check();

  // Update GUI Elements
  if (redrawRequired) {
    gslc_ElemSetRedraw(&m_gui, m_pElemOutCurTemp, GSLC_REDRAW_FOCUS);
    redrawRequired = false;
  }
  // ------------------------------------------------
  // Periodically call GUIslice update function
  // ------------------------------------------------
  gslc_Update(&m_gui);
}


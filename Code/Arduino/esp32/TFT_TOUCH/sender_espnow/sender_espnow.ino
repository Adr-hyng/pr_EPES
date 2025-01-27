// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "sender_espnow_GSLC.h"
#include <set>
#include <string>
#include <esp_now.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

// ------------------------------------------------
// Program Globals
uint8_t ESPRMacAddr[] = {
  0xb0, 0xb2, 0x1c, 
  0xa6, 0x45, 0xa0
};
esp_now_peer_info_t peerInfo;
struct_message myData;
const int buzzerPin = 5;
// ------------------------------------------------

// Save some element references for direct access
//<Save_References !Start!>
gslc_tsElemRef* m_pElemOutTxt3    = NULL;
gslc_tsElemRef* m_pElemXRingGauge1= NULL;
gslc_tsElemRef* m_pElemChildLock= NULL;
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
void sendData()
{
  // Set values to send
  myData.Mode = load_from_nvs(NVS_KEY_MODE, 0);

  m_pElemOutTxt3 = gslc_PageFindElemById(&m_gui, gslc_GetPageCur(&m_gui), ContainerCap);
  esp_err_t result = esp_now_send(ESPRMacAddr, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}
void execEveryMillis(unsigned long interval, void (*callback)()) {
  static unsigned long lastExecutionTime = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastExecutionTime >= interval) {
    lastExecutionTime = currentMillis;
    callback();
  }
}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Capacity: ");
  Serial.println(myData.ContainerCap);
  Serial.print("Mode: ");
  Serial.println(myData.Mode);
  Serial.print("Current Temp.: ");
  Serial.println(myData.CurTemperature);
  Serial.print("Child-Lock: ");
  Serial.println(myData.childLockActivated);
  Serial.print("Reset: ");
  Serial.println(myData.resetMode);
  Serial.println();

  if(myData.resetMode) {
    myData.resetMode = 0;
    ESP.restart();
  }

  m_pElemChildLock = gslc_PageFindElemById(&m_gui, gslc_GetPageCur(&m_gui), ChildLockIndicator);
  gslc_ElemSetGlowEn(&m_gui, m_pElemChildLock, myData.childLockActivated);
  gslc_ElemSetGlow(&m_gui, m_pElemChildLock, myData.childLockActivated);

  static char TContainerCap[4] = "";
  snprintf(TContainerCap, sizeof(TContainerCap), "%d", myData.ContainerCap);
  gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt3, TContainerCap);
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? 
  "Delivery Success" : "Delivery Fail");
}
void save_to_nvs(const char* key, int value) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
      nvs_set_i32(nvs_handle, key, value);
      nvs_commit(nvs_handle);
      nvs_close(nvs_handle);
  }
}

int load_from_nvs(const char* key, int default_value) {
  nvs_handle_t nvs_handle;
  int32_t value = default_value;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err == ESP_OK) {
      nvs_get_i32(nvs_handle, key, &value);
      nvs_close(nvs_handle);
  }
  return (int)value;
}

void beepBuzzer(unsigned long duration = 50) {
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
}

// Retoggle Buttons
void CbRetoggleBtn(gslc_tsGui* pGui, short selBtnElem)
{
    const int BUTTON_ENUMS[] = {AutoBtn, SafeBtn, TradBtn};
    short i = 0;
    for (short btn: BUTTON_ENUMS) {
        gslc_tsElemRef* pElemRef = gslc_PageFindElemById(pGui, gslc_GetPageCur(pGui), btn);
        if (pElemRef == nullptr) continue;
        g_ButtonFlags[i] = (btn == selBtnElem);
        gslc_ElemSetGlowEn(pGui, pElemRef, g_ButtonFlags[i]);
        gslc_ElemSetGlow(pGui, pElemRef, g_ButtonFlags[i]);
        i++;
    }
    beepBuzzer(100);
    sendData();
}
// Common Button callback
bool CbBtnCommon(void* pvGui,void *pvElemRef,gslc_teTouch eTouch,int16_t nX,int16_t nY)
{
  bool util_validBtnFlag = false;
  gslc_tsGui*     pGui     = (gslc_tsGui*)(pvGui);
  gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)(pvElemRef);
  gslc_tsElem*    pElem    = gslc_GetElemFromRef(pGui,pElemRef);

  if ( eTouch == GSLC_TOUCH_UP_IN ) {
    switch (pElem->nId) {
//<Button Enums !Start!>
      case AutoBtn:
        util_validBtnFlag = true;
        save_to_nvs(NVS_KEY_MODE, 2);
        break;
      case SafeBtn:
        util_validBtnFlag = true;
        save_to_nvs(NVS_KEY_MODE, 1);
        break;
      case TradBtn:
        util_validBtnFlag = true;
        save_to_nvs(NVS_KEY_MODE, 0);
        break;
      case ChildLockIndicator:
        break;
//<Button Enums !End!>
      default:
        break;
    }
    if(util_validBtnFlag) CbRetoggleBtn(pGui, pElem->nId);
  }
  return true;
}
void setup()
{
  // ------------------------------------------------
  // Initialize
  // ------------------------------------------------
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  gslc_InitDebug(&DebugOut);
  // ------------------------------------------------
  // Create graphic elements
  // ------------------------------------------------
  InitGUIslice_gen();
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  memcpy(peerInfo.peer_addr, ESPRMacAddr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

// -----------------------------------
// Main event loop
// -----------------------------------
void loop() { gslc_Update(&m_gui); }


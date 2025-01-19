//<App !Start!>
// FILE: [sender_espnow.ino]
// Created by GUIslice Builder version: [0.17.b34]
//
// GUIslice Builder Generated File
//
// For the latest guides, updates and support view:
// https://github.com/ImpulseAdventure/GUIslice
//
//<App !End!>

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
uint8_t broadcastAddress[] = {0xb0, 0xb2, 0x1c, 0xa6, 0x45, 0xa0}; // Old ESP
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
static int16_t DebugOut(char ch) { if (ch == (char)'\n') Serial.println(""); else Serial.write(ch); return 0; }

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
void sendData()
{
  // Set values to send
  myData.ContainerCap = myData.ContainerCap;
  myData.Mode = load_from_nvs(NVS_KEY_MODE, 0);

  m_pElemOutTxt3 = gslc_PageFindElemById(&m_gui, gslc_GetPageCur(&m_gui), ContainerCap);
  // Update the GUI element text
  // static char TContainerCap[4] = ""; // Ensure TContainerCap is properly declared
  // Generate a random number and convert it to a string
  // std::string randomStr = std::to_string(random(40, 100)); 
  // strncpy(TContainerCap, randomStr.c_str(), sizeof(TContainerCap) - 1); // Copy the string to TContainerCap
  // TContainerCap[sizeof(TContainerCap) - 1] = '\0'; // Null-terminate to prevent overflow

  // Update the GUI element
  // myData.ContainerCap = std::stoi(TContainerCap);
  // gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt3, TContainerCap);
  // gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}
void execEveryMillis(unsigned long interval, void (*callback)())
{
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
  Serial.println();

  m_pElemChildLock = gslc_PageFindElemById(&m_gui, gslc_GetPageCur(&m_gui), ChildLockIndicator);
  gslc_ElemSetGlowEn(&m_gui, m_pElemChildLock, myData.childLockActivated);
  gslc_ElemSetGlow(&m_gui, m_pElemChildLock, myData.childLockActivated);

  static char TContainerCap[4] = ""; // Ensure TContainerCap is properly declared
  snprintf(TContainerCap, sizeof(TContainerCap), "%d", myData.ContainerCap);
  if(myData.isStablized) {
    gslc_ElemXRingGaugeSetColorActiveFlat(&m_gui, m_pElemXRingGauge1, gslc_tsColor(0, 255, 0));
  } else {
    gslc_ElemXRingGaugeSetColorActiveFlat(&m_gui, m_pElemXRingGauge1, gslc_tsColor(255, 255, 255));
  }
  // Update the GUI element
  gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt3, TContainerCap);
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void UpdateBitmapImage(gslc_tsGui* pGui, gslc_tsElemRef* pElemRef, const unsigned short* newBitmap) 
{
    if (pElemRef == nullptr) {
        return; // Element reference is invalid
    }

    // Clear the element's background
    gslc_ElemSetRedraw(pGui, pElemRef, GSLC_REDRAW_FOCUS);

    // Create an empty image reference for the second image
    gslc_tsImgRef emptyImgRef = gslc_GetImageFromProg(nullptr, GSLC_IMGREF_NONE);

    // Update the image reference
    gslc_ElemSetImage(pGui, pElemRef, 
        gslc_GetImageFromProg((const unsigned char*)newBitmap, GSLC_IMGREF_FMT_BMP24), 
        emptyImgRef);

    // Trigger a redraw of the element to display the new image
    gslc_ElemSetRedraw(pGui, pElemRef, GSLC_REDRAW_FOCUS);
}



void save_to_nvs(const char* key, int value)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_i32(nvs_handle, key, value); // Use the key parameter
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
}

int load_from_nvs(const char* key, int default_value) 
{
    nvs_handle_t nvs_handle;
    int32_t value = default_value; // Use the default_value parameter
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        nvs_get_i32(nvs_handle, key, &value); // Use the key parameter
        nvs_close(nvs_handle);
    }
    return (int)value; // Cast back to int if necessary
}

void beepBuzzer(unsigned long duration = 50) {
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
}

// Retoggle Buttons
void CbRetoggleBtn(gslc_tsGui* pGui, short selBtnElem)
{
    // Iterate through the button IDs
    const int BUTTON_ENUMS[] = {AutoBtn, SafeBtn, TradBtn};
    short i = 0;
    for (short btn: BUTTON_ENUMS) {
        // Get the element reference for the current button ID
        gslc_tsElemRef* pElemRef = gslc_PageFindElemById(pGui, gslc_GetPageCur(pGui), btn);

        if (pElemRef == nullptr) {
            // Skip if the element reference is invalid
            continue;
        }

        // Set the button flags: true only for the selected button
        g_ButtonFlags[i] = (btn == selBtnElem);

        // Update glow state: Glow only the selected button, unglow others
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
  // Typecast the parameters to match the GUI and element types
  gslc_tsGui*     pGui     = (gslc_tsGui*)(pvGui);
  gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)(pvElemRef);
  gslc_tsElem*    pElem    = gslc_GetElemFromRef(pGui,pElemRef);

  if ( eTouch == GSLC_TOUCH_UP_IN ) {
    // From the element's ID we can determine which button was pressed.
    switch (pElem->nId) {
//<Button Enums !Start!>
      case AutoBtn:
        util_validBtnFlag = true;
        save_to_nvs(NVS_KEY_MODE, 2); // Auto Mode
        Serial.print("AUTO PRESSED");
        break;
      case SafeBtn:
        util_validBtnFlag = true;
        save_to_nvs(NVS_KEY_MODE, 1); // Safe Mode
        Serial.print("SAFE PRESSED");
        break;
      case TradBtn:
        util_validBtnFlag = true;
        save_to_nvs(NVS_KEY_MODE, 0); // Traditional Mode
        Serial.print("TRADITIONAL PRESSED");
        break;
      case ChildLockIndicator:
        break;
//<Button Enums !End!>
      default:
        break;
    }
    // sendData();
    if(util_validBtnFlag) CbRetoggleBtn(pGui, pElem->nId);
  }
  return true;
}
//<Checkbox Callback !Start!>
//<Checkbox Callback !End!>
//<Keypad Callback !Start!>
//<Keypad Callback !End!>
//<Spinner Callback !Start!>
//<Spinner Callback !End!>
//<Listbox Callback !Start!>
//<Listbox Callback !End!>
//<Draw Callback !Start!>
//<Draw Callback !End!>
//<Slider Callback !Start!>
//<Slider Callback !End!>
//<Tick Callback !Start!>
//<Tick Callback !End!>

void setup()
{
  // ------------------------------------------------
  // Initialize
  // ------------------------------------------------
  Serial.begin(9600);
  // Wait for USB Serial 
  //delay(1000);  // NOTE: Some devices require a delay after Serial.begin() before serial port can be used
  pinMode(buzzerPin, OUTPUT); // Set the buzzer pin as output
  digitalWrite(buzzerPin, LOW); // Ensure the buzzer is off initially

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

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

// -----------------------------------
// Main event loop
// -----------------------------------
void loop()
{

  // ------------------------------------------------
  // Update GUI Elements
  // ------------------------------------------------
  
  //TODO - Add update code for any text, gauges, or sliders
  // Execute sendData() every 5000 milliseconds (5 seconds)
  // execEveryMillis(1000, sendData);

  // ------------------------------------------------
  // Periodically call GUIslice update function
  // ------------------------------------------------
  gslc_Update(&m_gui);
    
}


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


// ------------------------------------------------
// Program Globals
bool g_ButtonFlags[4] = {false, false, false, false}; // Glowing buttons
// ------------------------------------------------

// Save some element references for direct access
//<Save_References !Start!>
//<Save_References !End!>

// Define debug message function
static int16_t DebugOut(char ch) { if (ch == (char)'\n') Serial.println(""); else Serial.write(ch); return 0; }

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
// Retoggle Buttons
void CbRetoggleBtn(gslc_tsGui* pGui, short selBtnElem) {
    // Iterate through the button IDs
    for (short i = E_ELEM_IMAGEBTN13; i <= E_ELEM_IMAGEBTN15; i++) {
        // Get the element reference for the current button ID
        gslc_tsElemRef* pElemRef = gslc_PageFindElemById(pGui, gslc_GetPageCur(pGui), i);

        if (pElemRef == nullptr) {
            // Skip if the element reference is invalid
            continue;
        }

        // Set the button flags: true only for the selected button
        g_ButtonFlags[i] = (i == selBtnElem);

        // Update glow state: Glow only the selected button, unglow others
        gslc_ElemSetGlowEn(pGui, pElemRef, g_ButtonFlags[i]);
        gslc_ElemSetGlow(pGui, pElemRef, g_ButtonFlags[i]);
    }
}

// Common Button callback
bool CbBtnCommon(void* pvGui,void *pvElemRef,gslc_teTouch eTouch,int16_t nX,int16_t nY)
{
  // Typecast the parameters to match the GUI and element types
  gslc_tsGui*     pGui     = (gslc_tsGui*)(pvGui);
  gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)(pvElemRef);
  gslc_tsElem*    pElem    = gslc_GetElemFromRef(pGui,pElemRef);

  if ( eTouch == GSLC_TOUCH_UP_IN ) {
    // From the element's ID we can determine which button was pressed.
    switch (pElem->nId) {
//<Button Enums !Start!>
      case E_ELEM_IMAGEBTN13:
        CbRetoggleBtn(pGui, E_ELEM_IMAGEBTN13);
        Serial.print("AUTO PRESSED");
        break;
      case E_ELEM_IMAGEBTN14:
        CbRetoggleBtn(pGui, E_ELEM_IMAGEBTN14);
        Serial.print("SAFE PRESSED");
        break;
      case E_ELEM_IMAGEBTN15:
        CbRetoggleBtn(pGui, E_ELEM_IMAGEBTN15);
        Serial.print("TRADITIONAL PRESSED");
        break;
//<Button Enums !End!>
      default:
        break;
    }
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

  gslc_InitDebug(&DebugOut);

  // ------------------------------------------------
  // Create graphic elements
  // ------------------------------------------------
  InitGUIslice_gen();

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
  
  // ------------------------------------------------
  // Periodically call GUIslice update function
  // ------------------------------------------------
  gslc_Update(&m_gui);
    
}


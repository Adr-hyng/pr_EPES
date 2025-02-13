//<File !Start!>
// FILE: [receiver_espnow_GSLC.h]
// Created by GUIslice Builder version: [0.17.b34]
//
// GUIslice Builder Generated GUI Framework File
//
// For the latest guides, updates and support view:
// https://github.com/ImpulseAdventure/GUIslice
//
//<File !End!>

#ifndef _GUISLICE_GEN_H
#define _GUISLICE_GEN_H

// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "GUIslice.h"
#include "GUIslice_drv.h"

// Include any extended elements
//<Includes !Start!>
// Include extended elements
#include "elem/XRingGauge.h"
//<Includes !End!>

// ------------------------------------------------
// Headers and Defines for fonts
// Note that font files are located within the Adafruit-GFX library folder:
// ------------------------------------------------
//<Fonts !Start!>
#if !defined(DRV_DISP_TFT_ESPI)
  #error E_PROJECT_OPTIONS tab->Graphics Library should be Adafruit_GFX
#endif
#include <TFT_eSPI.h>
#include "Seven_Segment18pt7b.h"
#include "Seven_Segment30pt7b.h"
//<Fonts !End!>

// ------------------------------------------------
// Defines for resources
// ------------------------------------------------
//<Resources !Start!>
extern "C" const unsigned short DROPLET_ICON[] PROGMEM;
extern "C" const unsigned short HEATER_GLOW[] PROGMEM;
extern "C" const unsigned short HEATER_NORMALIZE[] PROGMEM;
extern "C" const unsigned short THERMOMETER[] PROGMEM;
extern "C" const unsigned short THERMOMETER_1[] PROGMEM;
extern "C" const unsigned short THERMOMETER_2[] PROGMEM;
//<Resources !End!>

// ------------------------------------------------
// Enumerations for pages, elements, fonts, images
// ------------------------------------------------
//<Enum !Start!>
enum {E_PG_MAIN};
enum {ContainerCap,CurTemperature,E_ELEM_IMAGE3,E_ELEM_TEXT4
      ,E_ELEM_TEXT5,E_ELEM_TEXT7,E_ELEM_TEXT8,HeaterIndicator
      ,SelTempIMGREF};
// Must use separate enum for fonts with MAX_FONT at end to use gslc_FontSet.
enum {E_BUILTIN15X24,E_BUILTIN5X8,E_SEVEN_SEGMENT18,E_SEVEN_SEGMENT30
      ,MAX_FONT};
//<Enum !End!>

// ------------------------------------------------
// Instantiate the GUI
// ------------------------------------------------

// ------------------------------------------------
// Define the maximum number of elements and pages
// ------------------------------------------------
//<ElementDefines !Start!>
#define MAX_PAGE                1

#define MAX_ELEM_PG_MAIN 9 // # Elems total on page
#define MAX_ELEM_PG_MAIN_RAM MAX_ELEM_PG_MAIN // # Elems in RAM
//<ElementDefines !End!>

// ------------------------------------------------
// Create element storage
// ------------------------------------------------
gslc_tsGui                      m_gui;
gslc_tsDriver                   m_drv;
gslc_tsFont                     m_asFont[MAX_FONT];
gslc_tsPage                     m_asPage[MAX_PAGE];

//<GUI_Extra_Elements !Start!>
gslc_tsElem                     m_asPage1Elem[MAX_ELEM_PG_MAIN_RAM];
gslc_tsElemRef                  m_asPage1ElemRef[MAX_ELEM_PG_MAIN];
gslc_tsXRingGauge               m_sXRingGauge1;

#define MAX_STR                 100

//<GUI_Extra_Elements !End!>

// ------------------------------------------------
// Program Globals
// ------------------------------------------------

// Element References for direct access
//<Extern_References !Start!>
extern gslc_tsElemRef* m_pElemOutCurTemp;
extern gslc_tsElemRef* m_pElemOutSelTemp;
extern gslc_tsElemRef* m_pElemOutThermometerIMG;
extern gslc_tsElemRef* m_pElemXRingGauge1;
//<Extern_References !End!>

// Define debug message function
static int16_t DebugOut(char ch);

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
bool CbBtnCommon(void* pvGui,void *pvElemRef,gslc_teTouch eTouch,int16_t nX,int16_t nY);
bool CbCheckbox(void* pvGui, void* pvElemRef, int16_t nSelId, bool bState);
bool CbDrawScanner(void* pvGui,void* pvElemRef,gslc_teRedrawType eRedraw);
bool CbKeypad(void* pvGui, void *pvElemRef, int16_t nState, void* pvData);
bool CbListbox(void* pvGui, void* pvElemRef, int16_t nSelId);
bool CbSlidePos(void* pvGui,void* pvElemRef,int16_t nPos);
bool CbSpinner(void* pvGui, void *pvElemRef, int16_t nState, void* pvData);
bool CbTickScanner(void* pvGui,void* pvScope);

// ------------------------------------------------
// Create page elements
// ------------------------------------------------
void InitGUIslice_gen()
{
  gslc_tsElemRef* pElemRef = NULL;

  if (!gslc_Init(&m_gui,&m_drv,m_asPage,MAX_PAGE,m_asFont,MAX_FONT)) { return; }

  // ------------------------------------------------
  // Load Fonts
  // ------------------------------------------------
//<Load_Fonts !Start!>
    if (!gslc_FontSet(&m_gui,E_BUILTIN15X24,GSLC_FONTREF_PTR,NULL,3)) { return; }
    if (!gslc_FontSet(&m_gui,E_BUILTIN5X8,GSLC_FONTREF_PTR,NULL,1)) { return; }
    if (!gslc_FontSet(&m_gui,E_SEVEN_SEGMENT18,GSLC_FONTREF_PTR,&Seven_Segment18pt7b,1)) { return; }
    if (!gslc_FontSet(&m_gui,E_SEVEN_SEGMENT30,GSLC_FONTREF_PTR,&Seven_Segment30pt7b,1)) { return; }
//<Load_Fonts !End!>

//<InitGUI !Start!>
  gslc_PageAdd(&m_gui,E_PG_MAIN,m_asPage1Elem,MAX_ELEM_PG_MAIN_RAM,m_asPage1ElemRef,MAX_ELEM_PG_MAIN);

  // NOTE: The current page defaults to the first page added. Here we explicitly
  //       ensure that the main page is the correct page no matter the add order.
  gslc_SetPageCur(&m_gui,E_PG_MAIN);
  
  // Set Background to a flat color
  gslc_SetBkgndColor(&m_gui,GSLC_COL_BLACK);

  // -----------------------------------
  // PAGE: E_PG_MAIN
  

  // Create ring gauge ContainerCap 
  static char m_sRingText1[11] = "";
  pElemRef = gslc_ElemXRingGaugeCreate(&m_gui,ContainerCap,E_PG_MAIN,&m_sXRingGauge1,
          (gslc_tsRect){160,65,203,203},
          (char*)m_sRingText1,11,E_SEVEN_SEGMENT30);
  gslc_ElemXRingGaugeSetValRange(&m_gui, pElemRef, 0, 100);
  gslc_ElemXRingGaugeSetVal(&m_gui, pElemRef, 50); // Set initial value
  gslc_ElemXRingGaugeSetThickness(&m_gui,pElemRef, 20);
  gslc_ElemXRingGaugeSetColorActiveFlat(&m_gui,pElemRef, GSLC_COL_WHITE);
  gslc_ElemXRingGaugeSetColorInactive(&m_gui,pElemRef, GSLC_COL_GRAY_DK3);
  m_pElemXRingGauge1 = pElemRef;
 
  // Create E_ELEM_IMAGE3 using Image 
  pElemRef = gslc_ElemCreateImg(&m_gui,E_ELEM_IMAGE3,E_PG_MAIN,(gslc_tsRect){222,110,77,104},
    gslc_GetImageFromProg((const unsigned char*)DROPLET_ICON,GSLC_IMGREF_FMT_BMP24));
 
  // Create SelTempIMGREF using Image 
  pElemRef = gslc_ElemCreateImg(&m_gui,SelTempIMGREF,E_PG_MAIN,(gslc_tsRect){5,6,104,308},
    gslc_GetImageFromProg((const unsigned char*)THERMOMETER,GSLC_IMGREF_FMT_BMP24));
  m_pElemOutThermometerIMG = pElemRef;
  
  // Create HeaterIndicator button with image label
  pElemRef = gslc_ElemCreateBtnImg(&m_gui,HeaterIndicator,E_PG_MAIN,(gslc_tsRect){118,262,45,45},
          gslc_GetImageFromProg((const unsigned char*)HEATER_NORMALIZE,GSLC_IMGREF_FMT_BMP24),
          gslc_GetImageFromProg((const unsigned char*)HEATER_GLOW,GSLC_IMGREF_FMT_BMP24),
          &CbBtnCommon);
  gslc_ElemSetGlowEn(&m_gui, pElemRef, false);
  
  // Create CurTemperature runtime modifiable text
  static char m_sDisplayText3[4] = "0";
  pElemRef = gslc_ElemCreateTxt(&m_gui,CurTemperature,E_PG_MAIN,(gslc_tsRect){125,15,54,24},
    (char*)m_sDisplayText3,4,E_BUILTIN15X24);
  gslc_ElemSetTxtAlign(&m_gui,pElemRef,GSLC_ALIGN_MID_RIGHT);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  m_pElemOutCurTemp = pElemRef;
  
  // Create E_ELEM_TEXT4 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT4,E_PG_MAIN,(gslc_tsRect){183,9,20,8},
    (char*)"o",0,E_BUILTIN5X8);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  
  // Create E_ELEM_TEXT5 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT5,E_PG_MAIN,(gslc_tsRect){193,15,18,24},
    (char*)"C",0,E_BUILTIN15X24);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  
  // Create E_ELEM_TEXT7 runtime modifiable text
  static char m_sDisplayText7[3] = "75";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT7,E_PG_MAIN,(gslc_tsRect){30,255,31,25},
    (char*)m_sDisplayText7,3,E_SEVEN_SEGMENT18);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
  m_pElemOutSelTemp = pElemRef;
  
  // Create E_ELEM_TEXT8 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT8,E_PG_MAIN,(gslc_tsRect){70,255,12,25},
    (char*)"c",0,E_SEVEN_SEGMENT18);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
//<InitGUI !End!>

//<Startup !Start!>
  gslc_GuiRotate(&m_gui, 0);
//<Startup !End!>

}

#endif // end _GUISLICE_GEN_H

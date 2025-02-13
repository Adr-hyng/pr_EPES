# 🚧 Project EPES 🚧 
 
An open-source water dispenser using Raspberry Pi / Orange Pi utilizing Edge-AI technology for resource-constrained devices. *(Code, Circuit Design and Casing Design will be remodeled soon, project deadline on July 2025, Personal Gathered and Curated Image Dataset will be provided also)*. See early prototypes [here](Demo/) *(Still gathering images with [Roboflow](https://app.roboflow.com/) and using proper curation with [FastDup](https://github.com/visual-layer/fastdup))*.

## Target Features:
- [ ] Automatic Dispensing - Automatically dispense upon container recognition as well as stop operation upon closely full of the said container.
- [ ] Compatibility with Orange Pi One
- [ ] Customizable and Hackable like Mugsy

## Installation
 
### Installing Libraries (for both OS)
 - [**ESP32/8266 Board**](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)
 - [**Libraries:**](arduino-ide-prerequisite.zip)
   - OneWire
   - DallasTemperature
   - HX711
   - TFT_eSPI
   - GUIslice
 
### Required Software
 - **Arduino IDE**
 - **[GUIslice Builder](https://github.com/ImpulseAdventure/GUIslice-Builder)**
 - [**Image2c:**](https://github.com/Pconti31/GUIslice_Image2C)
 - **Blender** *(Optional in Windows)*
 - **Fritzing** *(Optional in Windows)*
 
---
 
## Raspberry Pi Setup
 
### Speaker On Boot *(Needs confirmation)*
 1. List available sound devices:
    ```bash
    aplay -l
    ```
    *(Identify the card number corresponding to the headphones for the speaker.)*
 2. Edit the ALSA configuration file:
    ```bash
    nano ~/.asoundrc
    ```
 3. Add the following configuration (adjust the card number if necessary):
    ```bash
    pcm.!default {
        type hw
        card 2
    }
 
    ctl.!default {
        type hw
        card 2
    }
    ```
 
### Script on Boot *(Needs confirmation)*
 - *(Instructions for setting up a script to run on boot can be added here.)*
 
### Rebooting the Raspberry Pi
 ```bash
 sudo shutdown -r now
 ```
 
> **Note:** It is recommended to use Remote Desktop to the Raspberry Pi for easier debugging.
 
---
 
## Windows Setup
 
 - **Software Installation:** Install the required software listed above.
 - **Library Installation:** Ensure all required libraries are installed.
 
---
 
## Setup
 
 1. **Clone the Repository:**
    ```bash
    git clone <repository-url>
    ```
 2. **Navigate to the Project Folder:**
    ```bash
    cd <project-repo-folder>
    ```
 3. **Prerequisites:**
    - **Main Materials:** ESP32 and Raspberry Pi 4B *(also applicable to Orange Pi One)*
    - **Circuit Design:** The design is provided in Fritzing format in `Circuit Design/Circuit_Design_EPES_v2.0.0.fzz`.
    - **Libraries:** Ensure all necessary libraries (listed above) are installed.
    - **Python Prerequisites:** Located in `Code/Raspiberry_Pi/src` (refer to the `requirements.txt` file for installation).
    - **Casing:** The 3D model is available in the `3D Model/Project_EPES_v1.0.0.blend` (Blender file format).
 4. **Test the System:**
    - Run the Python script to test the whole system:
      ```bash
      python classify.py
      ```
 
---

## Contributions
 
Contribution section are still a work in progress. If you would like to contribute or have any suggestions, hit me up with a private message on Discord:
 
**Discord Details:**
 - **Username:** `.adriancc`
 - **Display Name:** `Yang`
 - **User ID:** `531056443940536320`
 

--- 
 
## License
 
This project is licensed under the [MIT License](LICENSE.md).
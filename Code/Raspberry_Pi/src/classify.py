#!/usr/bin/env python
import RPi.GPIO as GPIO

import cv2
import os
import sys
import signal
import time
import subprocess
import threading
import serial
import json
import math
from edge_impulse_linux.image import ImageImpulseRunner

from button_handler import ButtonHandler
from timed_executor import TimedExecutor
from state_handler import StateHandler

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
runner = None
show_camera = True
if sys.platform == 'linux' and not os.environ.get('DISPLAY'):
    show_camera = False

SERIAL_PORT = '/dev/ttyUSB0'  # Adjust based on your setup
BAUD_RATE = 115200

#Button Pins
Button1 = 14 # Get Water Jug Capacity
Button2 = 4 # Get Temperature
Button3 = 2 # Decrease Temperature
Button4 = 3 # Increase Temperature
Button5 = 25 # Hot Dispense
Button6 = 22 # Warm Dispense
Button7 = 9 # Temperature Lock

# LED Pins
LED1 = 20
LED2 = 21
LED3 = 27

# Buzzer Pins
Buzzer1 = 5
Buzzer2 = 6

# Relay Pins
HotWater = 19
WarmWater = 26
Heater = 16
Solenoid1 = 13 # Child
Solenoid2 = 15 # Auto
Solenoid3 = 18 # Safe
Solenoid4 = 17 # Trad
Solenoid5 = 23 # Lock
Solenoid6 = None # Empty

# Relays Setup
GPIO.setup(HotWater,GPIO.OUT)
GPIO.setup(WarmWater,GPIO.OUT)
GPIO.setup(Heater,GPIO.OUT)
GPIO.setup(Solenoid1,GPIO.OUT)
GPIO.setup(Solenoid2,GPIO.OUT)
GPIO.setup(Solenoid3,GPIO.OUT)
GPIO.setup(Solenoid4,GPIO.OUT)
GPIO.setup(Solenoid5,GPIO.OUT)
if Solenoid6 is not None: GPIO.setup(Solenoid6,GPIO.OUT)

# LEDs setup
GPIO.setup(LED1,GPIO.OUT)
GPIO.setup(LED2,GPIO.OUT)
GPIO.setup(LED3,GPIO.OUT)

GPIO.output(WarmWater, GPIO.LOW);
GPIO.output(HotWater, GPIO.LOW);
GPIO.output(Heater, GPIO.LOW);
GPIO.output(Solenoid1, GPIO.LOW);
GPIO.output(Solenoid2, GPIO.LOW);
GPIO.output(Solenoid3, GPIO.LOW);
GPIO.output(Solenoid4, GPIO.LOW);
GPIO.output(Solenoid5, GPIO.LOW);
if Solenoid6 is not None: GPIO.output(Solenoid6, GPIO.LOW);
GPIO.output(LED1, GPIO.LOW)
GPIO.output(LED2, GPIO.LOW)
GPIO.output(LED3, GPIO.LOW)

# Program Globals
detection_timestamps = []  # List to store timestamaps of detections
min_detections = 10  # Number of detection in order to consider it as consistent
# Maximum number of milliseconds to consider it is consistent
# If a minimum of 10 was detected within 500 ms, then it is a valid container
frequency_millis = 500 

box_rect_size = 20 # Constant Box collider within the faucet, and outlet.

# Global variables from ESP32s
ContainerCap = 0
Mode = 2
CurTemperature = 0
IsPushed = False
WarmSelected = True
ChildLockActivated = False
TempSelectedIndex = 0
TempSelectedOptions = [65, 75, 85]
HeaterActivated = False
ResetMode = 0

IsSpeakerActive = False # Wait flag before TTS tries to execute again.
error_flag = False
STATE_FILE = "/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/state.json"
show_camera = False

def generate_and_play_audio(text):
    global IsSpeakerActive
    # Please download the binary for Raspberry 4 within Piper's repository.
    # In this case it was already downloaded.
    # It consumes about 40-50% of CPU spike of 500-1000 ms.
    try:
        IsSpeakerActive = True
        # Run Piper to generate audio from the input text and pipe the output directly to aplay
        piper_command = [
            '/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/piper/piper', 
            '-m', '/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/piper/models/en_US-amy-low.onnx', 
            '-c', '/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/piper/models/en_US-amy-low.onnx.json', 
            '-s', '0', 
            '-f', '-'
        ]

        
        # Use subprocess to pipe the text into Piper and then directly to aplay
        piper_process = subprocess.Popen(piper_command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        aplay_process = subprocess.Popen(['aplay'], stdin=piper_process.stdout)

        # Write the text to Piper's stdin and then wait for aplay to finish
        piper_process.stdin.write(text.encode())  # Pass text as bytes to Piper
        piper_process.stdin.close()  # Close stdin after writing
        aplay_process.wait()  # Wait for aplay to finish

        # Check if both processes completed successfully
        IsSpeakerActive = False
        print("DONE")

    except Exception as e:
        print(f"Error occurred: {e}")
        IsSpeakerActive = False

def playTTS(text):
    global IsSpeakerActive
    if IsSpeakerActive: return
    """Plays text-to-speech (TTS) in a separate thread."""
    tts_thread = threading.Thread(target=generate_and_play_audio, args=(text,))
    tts_thread.start()  # Start the thread to play audio

def selected_dispenser():
    # Pin selection based on boolean flag.
    return HotWater if not WarmSelected else WarmWater

def now():
    return round(time.time() * 1000)

def get_webcams():
    port_ids = []
    for port in range(5):
        print("Looking for a camera in port %s:" % port)
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) found in port %s " % (backendName, h, w, port))
                port_ids.append(port)
            camera.release()
    return port_ids

def sigint_handler(sig, frame):
    print('Interrupted')
    save_state()
    if runner:
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def save_state():
    state = {
        "WarmSelected": WarmSelected,
        "ChildLockActivated": ChildLockActivated,
        "TempSelectedIndex": TempSelectedIndex
    }
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)
    print("State saved:", state)

def load_state():
    global WarmSelected, ChildLockActivated, TempSelectedIndex
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            state = json.load(f)
            WarmSelected = state.get("WarmSelected", True)
            ChildLockActivated = state.get("ChildLockActivated", False)
            TempSelectedIndex = state.get("TempSelectedIndex", 0)
        print("State loaded:", state)
    else:
        print("State file not found. Using default values.")

def buttons_thread():
    alert_state = StateHandler(initial_state=ContainerCap)
    # Handles the button mechanism as well as the automatic heating 
    # system within a separate thread.
    global WarmSelected, TempSelectedIndex, HeaterActivated, ChildLockActivated, ResetMode, error_flag
    previous_warm_selected = None
    def dispense_toggling_mechanism():
        nonlocal previous_warm_selected
        global WarmSelected

        # Check if the state has changed
        if WarmSelected != previous_warm_selected:
            previous_warm_selected = WarmSelected  # Update the previous state
            GPIO.output(LED1, GPIO.LOW)
            GPIO.output(LED2, GPIO.LOW)
            if WarmSelected:       
                print("Warm is selected")     
                GPIO.output(LED3, GPIO.HIGH)
                GPIO.output(WarmWater, GPIO.LOW)
                GPIO.output(HotWater, GPIO.LOW)
            else:
                print("Warm is not selected")    
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(WarmWater, GPIO.LOW)
                GPIO.output(HotWater, GPIO.LOW)
    def toggle_hot_water():
        global WarmSelected
        WarmSelected = False
        save_state()
    def toggle_warm_water():
        global WarmSelected
        WarmSelected = True
        save_state()
    def increase_temp():
        global TempSelectedIndex
        if GPIO.input(LED3) != GPIO.LOW: return;
        if(TempSelectedIndex >= len(TempSelectedOptions) - 1):
            TempSelectedIndex = 0
            return
        TempSelectedIndex += 1
        save_state()
    def decrease_temp():
        global TempSelectedIndex
        if GPIO.input(LED3) != GPIO.LOW: return;
        if(TempSelectedIndex == 0):
            TempSelectedIndex = len(TempSelectedOptions) - 1
            return
        TempSelectedIndex -= 1
        save_state()
    def electric_heater_system():
        global HeaterActivated
        HeaterActivated = GPIO.input(Heater)
        if (CurTemperature >= TempSelectedOptions[TempSelectedIndex] and HeaterActivated == GPIO.HIGH):
            GPIO.output(Heater, GPIO.LOW)
        elif CurTemperature <= (TempSelectedOptions[TempSelectedIndex] - 5) and HeaterActivated == GPIO.LOW:
            GPIO.output(Heater, GPIO.HIGH)
    def toggle_childlock():
        global ChildLockActivated
        ChildLockActivated = not ChildLockActivated
        save_state()
    def pressed_reset():
        global ResetMode
        ResetMode = 1;
        print("RESET MODE = ", ResetMode)
    def solenoid_logic():
        GPIO.output(Solenoid1, ChildLockActivated)
        GPIO.output(Solenoid2, Mode == 2)
        GPIO.output(Solenoid3, Mode == 1)
        GPIO.output(Solenoid4, Mode == 0)
        GPIO.output(Solenoid5, not GPIO.input(LED3))
        if Solenoid6 is not None: GPIO.output(Solenoid6, ContainerCap <= 0)
    def alert_for_refill():
        global ContainerCap
        if alert_state.on_state_change(ContainerCap):  # Check if state has changed
            if ContainerCap <= 10:
                playTTS("Jug Capacity is 10 percent below.")
            elif ContainerCap <= 0:
                playTTS("Jug Capacity is empty. Please refill")
    
    warm_button = ButtonHandler(Button6, Buzzer2, -1, 1.5, short_press_callback=toggle_warm_water)
    hot_button = ButtonHandler(Button5, Buzzer2, Buzzer2, 1.5, short_press_callback=toggle_hot_water, long_press_callback=toggle_childlock)
    temp_lock_button = ButtonHandler(Button7, Buzzer2, Buzzer2, 1.5, long_press_callback=lambda: GPIO.output(LED3, not GPIO.input(LED3)))

    increase_button = ButtonHandler(Button4, Buzzer1, -1, 60, short_press_callback=increase_temp, short_output_condition=lambda: GPIO.input(LED3) == GPIO.LOW,pull_mode="PULLUP")
    decrease_button = ButtonHandler(Button3, Buzzer1, -1, 60, short_press_callback=decrease_temp, short_output_condition=lambda: GPIO.input(LED3) == GPIO.LOW,pull_mode="PULLUP")
    get_volume_button = ButtonHandler(Button1, Buzzer1, -1, 60,pull_mode="PULLUP", short_press_callback=lambda: playTTS(f"Jug Capacity is {ContainerCap} percent"))
    get_temperature_button = ButtonHandler(Button2, Buzzer1, Buzzer1, 5, short_press_callback=lambda: playTTS(f"Current temperature is {CurTemperature} degree celcius"), long_press_callback=pressed_reset,pull_mode="PULLUP")
    
    try:
        while True:
            alert_for_refill()
            #solenoid_logic()
            dispense_toggling_mechanism()
            electric_heater_system()
            current_time = time.time()
            warm_button.update(current_time)
            hot_button.update(current_time)
            temp_lock_button.update(current_time)
            increase_button.update(current_time)
            decrease_button.update(current_time)
            get_volume_button.update(current_time)
            get_temperature_button.update(current_time)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("exiting")
    except Exception:
        error_flag = True

def serial_reader_thread():
    # Thread managing the serial communication data from ESP32 through two-way communication
    # Raspberry Pi 4 <-USB-> ESP32-READ <-ESP_NOW-> ESP32-Touch
    global ContainerCap, Mode, CurTemperature, IsPushed, ChildLockActivated, HeaterActivated, ResetMode, error_flag
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            send_data = TimedExecutor(500) # 100 ms
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            while True:
                line = ser.readline().decode('utf-8').strip()
                send_data.execute(ser, f"0,{TempSelectedOptions[TempSelectedIndex]},{1 if HeaterActivated else 0},{1 if ChildLockActivated else 0 },{ResetMode}\n")
                
                if line:
                    try:
                        # Parse the CSV format
                        parts = line.split(',')
                        if len(parts) == 5:
                            Receivable = int(parts[0])
                            if Receivable == 0: continue # IGNORE some data that should not be received.
                            ContainerCap = int(parts[1])
                            Mode = int(parts[2])
                            CurTemperature = int(parts[3])
                            IsPushed = int(parts[4])
                            print(f"Parsed data: ContainerCap={ContainerCap}, Mode={Mode}, "
                                  f"CurTemperature={CurTemperature}, IsPushed={IsPushed}, Reset={ResetMode}, Child={ChildLockActivated}")
                        else:
                            print(f"Unexpected data format: {line}")
                        if ResetMode: 
                            ResetMode = 0
                    except ValueError:
                        print(f"Failed to parse: {line}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        playTTS("Serial Error. Please unplug and plug again.")
        error_flag = True

def remove_old_detections(current_time):
    """Remove detections older than x amount of seconds."""
    global detection_timestamps
    two_seconds_ago = current_time - frequency_millis  # Convert 2 seconds to milliseconds
    detection_timestamps = [timestamp for timestamp in detection_timestamps if timestamp >= two_seconds_ago]

is_pouring = False
def run_pump_thread(state):
    """
    Directly control the pump state with safe GPIO handling.
    This doesnt use a thread, it just safe GPIO handling.
    """
    global is_pouring
    sel_disp_pin = selected_dispenser()
    if state and not is_pouring:
        # Turn on the pump
        print("Turning pump ON - ", selected_dispenser())
        if sel_disp_pin == HotWater and ChildLockActivated: return
        GPIO.output(sel_disp_pin, GPIO.HIGH)
        is_pouring = True
    elif not state and is_pouring:
        # Turn off the pump
        print("Turning pump OFF - ", selected_dispenser())
        GPIO.output(sel_disp_pin, GPIO.LOW)
        is_pouring = False

def turn_pump(state):
    """
    Trigger the pump state change safely.
    """
    global is_pouring
    if (state and not is_pouring) or (not state and is_pouring):
        run_pump_thread(state)

def check_for_consistent_detections():
    """Check if there are enough recent detections to trigger an action."""
    global detection_timestamps
    if len(detection_timestamps) >= min_detections:
        print("Consistent container detections detected!")
        if Mode == 1:
            if IsPushed: turn_pump(True)
            else: turn_pump(False)
        elif Mode == 2:
            turn_pump(True)
        detection_timestamps.clear()
    else: turn_pump(False)

def main():
    global error_flag
    # Handles the object detection and dispensing mechanism
    model = "modelfile.eim"
    load_state()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)
    last_evaluation_time = 0    

    print('MODEL: ' + modelfile)
    
    closest_bb = None
    closest_distance = float("inf")

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            port_ids = get_webcams()
            if len(port_ids) == 0:
                playTTS("Camera Error. Please unplug and plug again.")
                raise Exception('Cannot find any webcams')
            videoCaptureDeviceId = int(port_ids[0])

            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) in port %s selected." % (backendName, h, w, videoCaptureDeviceId))
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")

            next_frame = 0  # limit to ~10 fps here

            for res, img in runner.classifier(videoCaptureDeviceId):
                if error_flag:
                    raise Exception("Something is wrong")
                color = (255, 0, 0)
                
                if Mode == 0: # traditional
                    if(IsPushed): turn_pump(True)
                    else: turn_pump(False)
                    pass
                elif Mode == 1: # safe 
                    if IsPushed: color = (0, 255, 0)
                    else: color = (255, 0, 0)
                        
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)
                        
                    current_time = now()  # Get current time in milliseconds
                    remove_old_detections(current_time)  # Remove old detections
                             
                    if "bounding_boxes" in res["result"].keys():
                        # ~ print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                        
                        target = (44, 50)
                        sorted_bounding_boxes = sorted(
                            res["result"]["bounding_boxes"],
                            key=lambda bb: math.sqrt(
                                (bb['x'] + bb['width'] // 2 - target[0]) ** 2 + (bb['y'] + bb['height'] // 2 - target[1]) ** 2
                            )
                        )
                        
                        if sorted_bounding_boxes:
                            closest_bb = sorted_bounding_boxes[0]
                            bb_center_x = closest_bb['x'] + closest_bb['width'] // 2
                            bb_center_y = closest_bb['y'] + closest_bb['height'] // 2
                            closest_distance = math.floor(math.sqrt((bb_center_x - target[0]) ** 2 + (bb_center_y - target[1]) ** 2))
                        
                            if closest_distance < box_rect_size:
                                detection_timestamps.append(current_time)
                                img = cv2.rectangle(img, (closest_bb['x'], closest_bb['y']), (closest_bb['x'] + closest_bb['width'], closest_bb['y'] + closest_bb['height']), color, 1)
                            
                    next_frame = now() + 5
                    cv2.rectangle(img, (target[0] - box_rect_size // 2, target[1] - box_rect_size // 2), (target[0] + box_rect_size, target[1] + box_rect_size), (0, 255, 0), 2)
                    
                    # Check if x amount of seconds have passed since the last evaluation
                    if current_time - last_evaluation_time >= frequency_millis:
                        if not IsPushed: turn_pump(False)
                        check_for_consistent_detections()  # Check for consistent detections    
                        last_evaluation_time = current_time  # Update the last evaluation time
                        
                elif Mode == 2: # automatic
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)
                        
                    current_time = now()  # Get current time in milliseconds
                    remove_old_detections(current_time)  # Remove old detections
                             
                    if "bounding_boxes" in res["result"].keys():
                        # ~ print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                        
                        target = (44, 50)
                        sorted_bounding_boxes = sorted(
                            res["result"]["bounding_boxes"],
                            key=lambda bb: math.sqrt(
                                (bb['x'] + bb['width'] // 2 - target[0]) ** 2 + (bb['y'] + bb['height'] // 2 - target[1]) ** 2
                            )
                        )
                        
                        if sorted_bounding_boxes:
                            closest_bb = sorted_bounding_boxes[0]
                            bb_center_x = closest_bb['x'] + closest_bb['width'] // 2
                            bb_center_y = closest_bb['y'] + closest_bb['height'] // 2
                            closest_distance = math.floor(math.sqrt((bb_center_x - target[0]) ** 2 + (bb_center_y - target[1]) ** 2))
                        
                            if closest_distance < box_rect_size:
                                detection_timestamps.append(current_time)
                                img = cv2.rectangle(img, (closest_bb['x'], closest_bb['y']), (closest_bb['x'] + closest_bb['width'], closest_bb['y'] + closest_bb['height']), color, 1)
                            
                    next_frame = now() + 5
                    cv2.rectangle(img, (target[0] - box_rect_size // 2, target[1] - box_rect_size // 2), (target[0] + box_rect_size, target[1] + box_rect_size), (0, 255, 0), 2)
                    
                    # Check if x amount of seconds have passed since the last evaluation
                    if current_time - last_evaluation_time >= frequency_millis:
                        check_for_consistent_detections()  # Check for consistent detections    
                        last_evaluation_time = current_time  # Update the last evaluation time
                
                if show_camera:
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break
        except KeyboardInterrupt:
            error_flag = True
            print("exiting")
        finally:
            GPIO.output(WarmWater, GPIO.LOW);
            GPIO.output(HotWater, GPIO.LOW);
            GPIO.output(Heater, GPIO.LOW);
            GPIO.output(Solenoid1, GPIO.LOW);
            GPIO.output(Solenoid2, GPIO.LOW);
            GPIO.output(Solenoid3, GPIO.LOW);
            GPIO.output(Solenoid4, GPIO.LOW);
            GPIO.output(Solenoid5, GPIO.LOW);
            if Solenoid6 is not None: GPIO.output(Solenoid6, GPIO.LOW);
            GPIO.cleanup()
            if runner:
                runner.stop()

if __name__ == "__main__":
    # In total it consumes 3-threads with remaining of 1-thread for 
    # future use cases.
    playTTS(f"Booting Completed")
    button_thread = threading.Thread(target=buttons_thread, daemon=True)
    button_thread.start()
    serial_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    serial_thread.start()
    main()
    # In future please improve dataset to dont detect hands and also detect overflow
    # separate the dataset with not_full, and full with unique and high quality data as possible
    # especially drinking container that has sparkles and bubbles after being dispensed.
    # Use ToF for distance, interface to ESP32
    # Use Raspberry Pi Screen instead, so you can just use the other ESP32s for improvement.
    # Cause currently ESP32s are used for TFT display, which are cheap.
    # Raspberry Pi Display is should be worth it.
    
    # The only thing that needs to be improved are:
    """
    - ToF for distance, just detect within the range of dispenser
    - Improved Model to classify between two variables: not_full, and full (Use classification instead of object detection)
    - Use Raspberry Pi Display 7 inches.
    - Dont buy a grounded power supply
    - Make Fabrication of Housing non-conductive as possible.
    - Space for components
    - Use Mechanical Advantage for Faucet Joystick (Push n Pull) that converts analog to digital
    - Use Raspberry Pi Camera, instead of web-cam
    - Use Lens or glass that does not get fog or to avoid fog from the camera.
    - Use Water Pump, as well as solenoid to avoid leaking
    - Be Precise with your fabrication such as 3d modeling as it could save time. 
    """

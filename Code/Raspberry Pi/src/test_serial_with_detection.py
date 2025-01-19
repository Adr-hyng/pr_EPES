#!/usr/bin/env python

import device_patches  # Device specific patches for Jetson Nano (needs to be before importing cv2)
import RPi.GPIO as GPIO

import cv2
import os
import sys, getopt
import signal
import time
import threading
import serial
import json
import math
import concurrent.futures
from edge_impulse_linux.image import ImageImpulseRunner

from button_handler import ButtonHandler

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
runner = None
show_camera = True
if sys.platform == 'linux' and not os.environ.get('DISPLAY'):
    show_camera = False

SERIAL_PORT = '/dev/ttyUSB0'  # Adjust based on your setup
BAUD_RATE = 115200

#Button Pins
Button1 = 0 # Get Water Jug Capacity
Button2 = 1 # Get Temperature
Button3 = 27 # Decrease
Button4 = 22 # Increase
Button5 = 25 # hot
Button6 = 21 # warm
Button7 = 9 # temp Lock
LED1 = 20
LED2 = 11
LED3 = 8
Buzzer1 = 6
Buzzer2 = 13
HotWater = 26
WarmWater = 19
Heater = 16

GPIO.setup(HotWater,GPIO.OUT)
GPIO.setup(WarmWater,GPIO.OUT)
GPIO.setup(Heater,GPIO.OUT)
#LED
GPIO.setup(LED1,GPIO.OUT)
GPIO.setup(LED2,GPIO.OUT)
GPIO.setup(LED3,GPIO.OUT)


GPIO.output(WarmWater, GPIO.LOW);
GPIO.output(HotWater, GPIO.LOW);
GPIO.output(Heater, GPIO.LOW);
GPIO.output(LED1, GPIO.LOW)
GPIO.output(LED2, GPIO.LOW)
GPIO.output(LED3, GPIO.LOW)


detection_timestamps = []  # List to store timestamps of detections
min_detections = 1  # 20 = 500
frequency_millis = 100 # Number of milliseconds to detect it is consistent container.
box_rect_size = 18

# Shared flag for joystick status
# Global variables for parsed data
ContainerCap = 0
Mode = 2
CurTemperature = 0
IsPushed = False
WarmSelected = True
ChildLockActivated = False
TempSelectedIndex = 0
TempSelectedOptions = [65, 75, 85]  # Send the selected Temperaturet to ESP32 - TODO

HeaterActivated = False # Send to ESP32

ResetMode = 0

show_camera = True

def selected_dispenser():
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
    if runner:
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def buttons_thread():
    global WarmSelected, TempSelectedIndex, HeaterActivated, ChildLockActivated, ResetMode
    # Track the previous state of WarmSelected
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
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(WarmWater, GPIO.LOW)
                GPIO.output(HotWater, GPIO.LOW)
            else:
                print("Warm is not selected")    
                GPIO.output(LED2, GPIO.HIGH)
                GPIO.output(WarmWater, GPIO.LOW)
                GPIO.output(HotWater, GPIO.LOW)
            
    def toggle_hot_water():
        global WarmSelected
        WarmSelected = False
    def toggle_warm_water():
        global WarmSelected
        WarmSelected = True
        
    def increase_temp():
        global TempSelectedIndex
        if GPIO.input(LED3) != GPIO.LOW: return;
        if(TempSelectedIndex >= len(TempSelectedOptions) - 1): return;
        TempSelectedIndex += 1
        
    def decrease_temp():
        global TempSelectedIndex
        if GPIO.input(LED3) != GPIO.LOW: return;
        if(TempSelectedIndex == 0): return;
        TempSelectedIndex -= 1
        
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
        
    def pressed_reset():
        global ResetMode
        ResetMode = 1;
        print("RESET MODE = ", ResetMode)
    
    warm_button = ButtonHandler(Button6, Buzzer2, -1, 1.5, short_press_callback=toggle_warm_water)
    hot_button = ButtonHandler(Button5, Buzzer2, Buzzer2, 1.5, short_press_callback=toggle_hot_water, long_press_callback=toggle_childlock)
    temp_lock_button = ButtonHandler(Button7, Buzzer2, Buzzer2, 1.5, long_press_callback=lambda: GPIO.output(LED3, not GPIO.input(LED3)))

    increase_button = ButtonHandler(Button4, Buzzer1, -1, 60, short_press_callback=increase_temp, short_output_condition=lambda: GPIO.input(LED3) == GPIO.LOW)
    decrease_button = ButtonHandler(Button3, Buzzer1, -1, 60, short_press_callback=decrease_temp, short_output_condition=lambda: GPIO.input(LED3) == GPIO.LOW)
    get_volume_button = ButtonHandler(Button1, Buzzer1, -1, 60)
    get_temperature_button = ButtonHandler(Button2, Buzzer1, Buzzer1, 5, long_press_callback=pressed_reset)
    
    try:
        while True:
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
    finally:
        GPIO.cleanup();

def serial_reader_thread():
    global ContainerCap, Mode, CurTemperature, IsPushed, ChildLockActivated, HeaterActivated, ResetMode
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            
            ResetMode = 1
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        # Parse the CSV format
                        parts = line.split(',')
                        ser.write(f"0,{TempSelectedOptions[TempSelectedIndex]},{1 if HeaterActivated else 0},{1 if ChildLockActivated else 0 },{ResetMode}\n".encode('utf-8'))
                        if len(parts) == 5:
                            Receivable = int(parts[0])
                            if Receivable == 0: continue # IGNORE some data that should not be received.
                            ContainerCap = int(parts[1])
                            Mode = int(parts[2])
                            CurTemperature = int(parts[3])
                            IsPushed = int(parts[4])
                            print(f"Parsed data: ContainerCap={ContainerCap}, Mode={Mode}, "
                                  f"CurTemperature={CurTemperature}, IsPushed={IsPushed}, Reset={ResetMode}")
                        else:
                            print(f"Unexpected data format: {line}")
                        if ResetMode: ResetMode = 0
                    except ValueError:
                        print(f"Failed to parse: {line}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")

def remove_old_detections(current_time):
    """Remove detections older than 2 seconds."""
    global detection_timestamps
    two_seconds_ago = current_time - frequency_millis  # Convert 2 seconds to milliseconds
    detection_timestamps = [timestamp for timestamp in detection_timestamps if timestamp >= two_seconds_ago]

is_pouring = False
def run_pump_thread(state):
    """
    Directly control the pump state with safe GPIO handling.
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
    model = "modelfile.eim"

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
                    
                    # Check if 2 seconds have passed since the last evaluation
                    if current_time - last_evaluation_time >= frequency_millis:
                        if not IsPushed: turn_pump(False)
                        check_for_consistent_detections()  # Check for consistent detections    
                        # print(RelayState)
                        # GPIO.output(RelayPump, RelayState)    
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
                    
                    # Check if 2 seconds have passed since the last evaluation
                    if current_time - last_evaluation_time >= frequency_millis:
                        check_for_consistent_detections()  # Check for consistent detections    
                        # GPIO.output(RelayPump, RelayState)    
                        last_evaluation_time = current_time  # Update the last evaluation time
                if show_camera:
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break
        finally:
            GPIO.output(WarmWater, GPIO.LOW);
            GPIO.output(HotWater, GPIO.LOW);
            GPIO.output(Heater, GPIO.LOW);
            GPIO.cleanup();
            if runner:
                runner.stop()

if __name__ == "__main__":
    serial_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    serial_thread.start()
    button_thread = threading.Thread(target=buttons_thread, daemon=True)
    button_thread.start()
    main()

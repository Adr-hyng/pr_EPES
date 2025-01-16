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

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
runner = None
show_camera = True
if sys.platform == 'linux' and not os.environ.get('DISPLAY'):
    show_camera = False

SERIAL_PORT = '/dev/ttyUSB0'  # Adjust based on your setup
BAUD_RATE = 115200

HotWater = 6
WarmWater = 13

GPIO.setup(HotWater,GPIO.OUT)
GPIO.setup(WarmWater,GPIO.OUT)

GPIO.output(WarmWater, GPIO.LOW);
GPIO.output(HotWater, GPIO.LOW);

detection_timestamps = []  # List to store timestamps of detections
min_detections = 1  # 20 = 500
frequency_millis = 50 # Number of milliseconds to detect it is consistent container.
dispense_state = 0

# Shared flag for joystick status
# Global variables for parsed data
ContainerCap = 0
Mode = 2
CurTemperature = 0
IsPushed = False

show_camera = True

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

def help():
    print('python classify.py <path_to_model.eim> <Camera port ID, only required when more than 1 camera is present>')

def serial_reader():
    global ContainerCap, Mode, CurTemperature, IsPushed
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        # Parse the CSV format
                        parts = line.split(',')
                        
                        if len(parts) == 4:
                            ContainerCap = int(parts[0])
                            Mode = int(parts[1])
                            CurTemperature = int(parts[2])
                            IsPushed = int(parts[3])
                            print(f"Parsed data: ContainerCap={ContainerCap}, Mode={Mode}, "
                                  f"CurTemperature={CurTemperature}, IsPushed={IsPushed}")
                        else:
                            print(f"Unexpected data format: {line}")
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

def turn_pump(state):
    global is_pouring
    pass
#    if state:
#        if not is_pouring:
#            GPIO.output(WarmWater, GPIO.HIGH)
#        is_pouring = True
#        
#    else:
#        if is_pouring:
#            GPIO.output(WarmWater, GPIO.LOW)
#        is_pouring = False
#    if state == True:
#        if is_pouring == True:
#            return
#        else:
#            GPIO.output(WarmWater, GPIO.HIGH)
#            is_pouring = True 
#    else:
#        if is_pouring == False:
#            return
#        else:
#            GPIO.output(WarmWater, GPIO.LOW)
#            is_pouring = False

def check_for_consistent_detections():
    """Check if there are enough recent detections to trigger an action."""
    global detection_timestamps
    if len(detection_timestamps) >= min_detections:
        print("Consistent container detections detected!")
        if Mode == 1:
            if IsPushed:
                #GPIO.output(WarmWater, GPIO.HIGH)
                turn_pump(True)
            else:
                #GPIO.output(WarmWater, GPIO.LOW)
                turn_pump(False)
        elif Mode == 2:
            #GPIO.output(WarmWater, GPIO.HIGH);
            turn_pump(True)
        detection_timestamps.clear()
    else:
        GPIO.output(WarmWater, GPIO.LOW);
        turn_pump(False)

box_rect_size = 20

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
                    if(IsPushed):
                        turn_pump(True)
                        #GPIO.output(WarmWater, GPIO.HIGH)
                    else:
                        #GPIO.output(WarmWater, GPIO.LOW)
                        turn_pump(False)
                    pass
                elif Mode == 1: # safe 
                    
                    if IsPushed:
                        color = (0, 255, 0)
                    else:
                        color = (255, 0, 0)
                        
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
                        
                            if closest_distance < 15:
                                detection_timestamps.append(current_time)
                                img = cv2.rectangle(img, (closest_bb['x'], closest_bb['y']), (closest_bb['x'] + closest_bb['width'], closest_bb['y'] + closest_bb['height']), (255, 0, 0), 1)
                                cv2.rectangle(img, (target[0], target[1]), (target[0] + box_rect_size, target[1] + box_rect_size), color, 1)
                            
                    next_frame = now() + 5
                    cv2.rectangle(img, (target[0] - box_rect_size // 2, target[1] - box_rect_size // 2), (target[0] + box_rect_size, target[1] + box_rect_size), (0, 255, 0), 2)
                    
                    # Check if 2 seconds have passed since the last evaluation
                    if current_time - last_evaluation_time >= frequency_millis:
                        if not IsPushed:
                            #GPIO.output(WarmWater, GPIO.LOW);
                            turn_pump(False)
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
                        
                            if closest_distance < 15:
                                detection_timestamps.append(current_time)
                                img = cv2.rectangle(img, (closest_bb['x'], closest_bb['y']), (closest_bb['x'] + closest_bb['width'], closest_bb['y'] + closest_bb['height']), color, 1)
                                #cv2.rectangle(img, (target[0], target[1]), (target[0] + box_rect_size, target[1] + box_rect_size), color, 1)
                            
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
            GPIO.cleanup();
            if runner:
                runner.stop()

if __name__ == "__main__":
    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    main()

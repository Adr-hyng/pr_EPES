#!/usr/bin/env python

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import sys
import signal
import time
import concurrent.futures
from edge_impulse_linux.image import ImageImpulseRunner
import psutil

import board
import busio
import adafruit_vl53l0x

# Button and Relay
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
DispenseSwitch = 27
AutoSwitch = 22
RelayPump = 17
GPIO.setup(DispenseSwitch,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(AutoSwitch,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(RelayPump,GPIO.OUT)

# ToF Sensor
i2c = busio.I2C(board.SCL, board.SDA)
vl53L0X = adafruit_vl53l0x.VL53L0X(i2c)
# ~ vl53L0X.measurement_timing_budget = 200000 # Slow, Accurate


runner = None
is_speaking = False
detection_timestamps = []  # List to store timestamps of detections
min_detections = 30  # Minimum number of detections required within 2 seconds
frequency_millis = 2000 # Number of milliseconds to detect it is consistent container.

# if you don't want to see a camera preview, set this to False
show_camera = True
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False

def is_process_running(process_name):
    """
    Check if there is any running process that contains the given name process_name.
    """
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Check if process name or command line matches the given process_name
            if process_name in proc.info['name'] or process_name in " ".join(proc.info['cmdline']):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False

def run_tts(text):
    import subprocess
    global is_speaking
    try:
        process_name = "ttsDemo"
        if not is_process_running(process_name):
            if(not is_speaking):
                is_speaking = True
                process = subprocess.Popen(['./ttsDemo'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                stdout, stderr = process.communicate(input=text)
                # After process completed.
                is_speaking = False
                if process.returncode != 0: print(f"Error: {stderr}")
        else:
            print("TTS process is already running.")
    except Exception as e:
        print(f"Failed to run TTS: {e}")

def remove_old_detections(current_time):
    """Remove detections older than 2 seconds."""
    global detection_timestamps
    two_seconds_ago = current_time - frequency_millis  # Convert 2 seconds to milliseconds
    detection_timestamps = [timestamp for timestamp in detection_timestamps if timestamp >= two_seconds_ago]

def check_for_consistent_detections(executor):
    """Check if there are enough recent detections to trigger an action."""
    global detection_timestamps
    if len(detection_timestamps) >= min_detections:
        print("Consistent container detections detected!")
        # Execute your action here
        
        # IF VL53L0X get its default max range, then there's no object, so stop dispensing.
        
        # EXECUTE only this TTS when initially detected the container, then resets
        # after there's no object detected from ToF Sensor.
        print("Range: {0}mm".format(vl53L0X.range))
        # ~ executor.submit(run_tts, "A container is detected. Please stand still.")
        detection_timestamps.clear()  # Reset detections to avoid repeated actions

def now():
    return round(time.time() * 1000)

def get_webcams():
    port_ids = []
    for port in range(5):
        print(f"Looking for a camera in port {port}:")
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print(f"Camera {backendName} ({h} x {w}) found in port {port}")
                port_ids.append(port)
            camera.release()
    return port_ids

def sigint_handler(sig, frame):
    print('Interrupted')
    if runner:
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def main():
    model = "modelfile.eim"

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL:', modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print(f'Loaded runner for "{model_info["project"]["owner"]} / {model_info["project"]["name"]}"')
            labels = model_info['model_parameters']['labels']
            port_ids = get_webcams()
            if not port_ids:
                raise Exception('Cannot find any webcams')
            if len(port_ids) > 1:
                raise Exception("Multiple cameras found. Specify the camera port ID as an argument.")
            videoCaptureDeviceId = int(port_ids[0])

            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print(f"Camera {backendName} ({h} x {w}) in port {videoCaptureDeviceId} selected.")
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")

            next_frame = 0  # Limit to ~10 fps here

            with concurrent.futures.ThreadPoolExecutor() as executor:
                for res, img in runner.classifier(videoCaptureDeviceId):
                    # Dispense Button
                    dispense_state = GPIO.input(DispenseSwitch)
                    automatic_state = GPIO.input(AutoSwitch)
                    
                    if automatic_state == 0:
                        if not dispense_state: GPIO.output(RelayPump,GPIO.LOW)
                        else: GPIO.output(RelayPump,GPIO.HIGH)
                    else:
                        if (next_frame > now()):
                            time.sleep((next_frame - now()) / 1000)
                            
                        current_time = now()  # Get current time in milliseconds
                        remove_old_detections(current_time)  # Remove old detections

                        # print('classification runner response', res)

                        if "classification" in res["result"].keys():
                            print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                            for label in labels:
                                score = res['result']['classification'][label]
                                print('%s: %.2f\t' % (label, score), end='')
                            print('', flush=True)

                        elif "bounding_boxes" in res["result"].keys():
                            # ~ print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                            for bb in res["result"]["bounding_boxes"]:
                                if bb['value'] < 0.90: continue
                                # ~ print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                                detection_timestamps.append(current_time)  # Record detection time
                                img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                        
                        check_for_consistent_detections(executor)  # Check for consistent detections
                        # ~ next_frame = now() + 100
                        next_frame = now() + 10
                        
                    if (show_camera):
                        cv2.imshow('Drinking Container Presence Detection', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                        if cv2.waitKey(1) == ord('q'):
                            break
        finally:
            if runner:
                runner.stop()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import device_patches  # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import sys, getopt
import signal
import time
import threading
import serial
import json
from edge_impulse_linux.image import ImageImpulseRunner

runner = None
show_camera = True
if sys.platform == 'linux' and not os.environ.get('DISPLAY'):
    show_camera = False

SERIAL_PORT = '/dev/ttyUSB0'  # Adjust based on your setup
BAUD_RATE = 115200

# Shared flag for joystick status
is_pushed = False

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
    global is_pushed
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        data = json.loads(line)
                        is_pushed = data.get("is_pushed", False)
                        print(f"Serial data received: is_pushed={is_pushed}")
                    except json.JSONDecodeError:
                        print(f"Failed to parse: {line}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()

    if len(args) == 0:
        help()
        sys.exit(2)

    model = "modelfile.eim"

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            if len(args) >= 2:
                videoCaptureDeviceId = int(args[1])
            else:
                port_ids = get_webcams()
                if len(port_ids) == 0:
                    raise Exception('Cannot find any webcams')
                if len(args) <= 1 and len(port_ids) > 1:
                    raise Exception("Multiple cameras found. Add the camera port ID as a second argument to use to this script")
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
                if next_frame > now():
                    time.sleep((next_frame - now()) / 1000)
                    
                if is_pushed:
                    color = (0, 255, 0)

                if "classification" in res["result"].keys():
                    print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                    for label in labels:
                        score = res['result']['classification'][label]
                        print('%s: %.2f\t' % (label, score), end='')
                    print('', flush=True)

                elif "bounding_boxes" in res["result"].keys():
                    print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                    for bb in res["result"]["bounding_boxes"]:
                        print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                        img = cv2.rectangle(img, (bb['x'], bb['y']),
                                            (bb['x'] + bb['width'], bb['y'] + bb['height']), color, 1)

                if show_camera:
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break

                next_frame = now() + 100
        finally:
            if runner:
                runner.stop()

if __name__ == "__main__":
    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    main(sys.argv[1:])

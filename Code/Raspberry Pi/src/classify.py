#!/usr/bin/env python

# build the setup through: "python setup.py build"
# install the setup through: "sudo python setup.py install"
# before installing all the dependencies follow the edge impulse sdk guide, and install pyAudio through apt-get.
# install all the dependencies: "sudo pip install -r requirements.txt --break-system-packages"
# use command to permit access to eim file: "chmod +x <modelfile.eim path>"
# run this python script.

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import sys
import signal
import time
import concurrent.futures
from edge_impulse_linux.image import ImageImpulseRunner
import psutil

runner = None
is_speaking = False

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
    try:
        process_name = "ttsDemo"
        if not is_process_running(process_name):
            print(f"Running TTS: {text}")
            process = subprocess.run(['./ttsDemo'], input=text, text=True, capture_output=True)
            myProcessIsRunning = poll() is None 
            if(not myProcessIsRunning):
                print("Terminate TTS");
            print(f"TTS Output: {process.stdout}")
            if process.returncode != 0: print(f"Error: {process.stderr}")            
        else:
            print("TTS process is already running.")
            return
    except Exception as e:
        print(f"Failed to run TTS: {e}")

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
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)

                    # print('classification runner response', res)

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
                            executor.submit(run_tts, "A container is detected. Please stand still.")
                            img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)

                    if (show_camera):
                        cv2.imshow('Drinking Container Presence Detection', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                        if cv2.waitKey(1) == ord('q'):
                            break

                    #next_frame = now() + 100
                    next_frame = now() + 10
        finally:
            if runner:
                runner.stop()

if __name__ == "__main__":
    main()

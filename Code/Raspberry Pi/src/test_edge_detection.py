#!/usr/bin/env python

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner

runner = None
# if you don't want to see a camera preview, set this to False
show_camera = True
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False

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

def main():
    model = "modelfile.eim"

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

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
                if next_frame > now():
                    time.sleep((next_frame - now()) / 1000)

                # Apply Canny edge detection to the entire frame by default
                edges = cv2.Canny(img, 50, 150)
                canny_frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

                if "bounding_boxes" in res["result"].keys() and len(res["result"]["bounding_boxes"]) > 0:
                    print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]),
                                                                res['timing']['dsp'] + res['timing']['classification']))
                    # Apply Gaussian blur to the entire image
                    blurred_img = cv2.GaussianBlur(img, (21, 21), 0)
                    
                    for bb in res["result"]["bounding_boxes"]:
                        print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (
                            bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))

                        # Configurable offset for ROI size
                        offset = 25

                        # Ensure the ROI remains within the image boundaries
                        roi_x1 = max(0, bb['x'] - offset)
                        roi_y1 = max(0, bb['y'] - offset)
                        roi_x2 = min(img.shape[1], bb['x'] + bb['width'] + offset)
                        roi_y2 = min(img.shape[0], bb['y'] + bb['height'] + offset)

                        # Extract the ROI from the original image (not blurred)
                        roi = img[roi_y1:roi_y2, roi_x1:roi_x2]

                        # Apply Canny edge detection on the ROI
                        edges = cv2.Canny(roi, 100, 250)

                        # Convert edges to a 3-channel image
                        canny_frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

                        # Overlay edges back onto the blurred image
                        blurred_img[roi_y1:roi_y2, roi_x1:roi_x2] = canny_frame

                        # Draw the bounding box on the blurred image
                        blurred_img = cv2.rectangle(blurred_img, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 1)

                    # Update the main display frame
                    img = blurred_img

                if show_camera:
                    # Resize the Canny frame to match the main image size
                    canny_frame_resized = cv2.resize(canny_frame, (img.shape[1], img.shape[0]))

                    # Combine original and Canny frames side by side
                    combined_frame = cv2.hconcat([cv2.cvtColor(img, cv2.COLOR_RGB2BGR), canny_frame_resized])
                    cv2.imshow('Original and Canny Edge Detection', combined_frame)

                    if cv2.waitKey(1) == ord('q'):
                        break

                next_frame = now() + 100


        finally:
            if runner:
                runner.stop()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import sys, getopt
import signal
import time
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner


# TOBEDELETED
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
WarmWater = 13
GPIO.setup(WarmWater,GPIO.OUT)
GPIO.output(WarmWater, GPIO.LOW);
is_pouring = False
def run_pump_thread(state):
    """
    Directly control the pump state with safe GPIO handling.
    """
    global is_pouring
    if state and not is_pouring:
        # Turn on the pump
        print("Turning pump ON")
        GPIO.output(WarmWater, GPIO.HIGH)
        is_pouring = True
    elif not state and is_pouring:
        # Turn off the pump
        print("Turning pump OFF")
        GPIO.output(WarmWater, GPIO.LOW)
        is_pouring = False

def turn_pump(state):
    """
    Trigger the pump state change safely.
    """
    global is_pouring
    if (state and not is_pouring) or (not state and is_pouring):
        run_pump_thread(state)

# TOBEDELETED

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

                # Make a copy of the original frame for processing
                processed_img = img.copy()

                if "bounding_boxes" in res["result"].keys() and len(res["result"]["bounding_boxes"]) > 0:
                    print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]),
                                                                res['timing']['dsp'] + res['timing']['classification']))
                    # Apply Gaussian blur to the entire image
                    blurred_img = cv2.GaussianBlur(processed_img, (21, 21), 0)
                    
                    for bb in res["result"]["bounding_boxes"]:
                        print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (
                            bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))

                        # Configurable offset for ROI size
                        offset = 45

                        # Ensure the ROI remains within the image boundaries
                        roi_x1 = max(0, bb['x'] - offset)
                        roi_y1 = max(0, bb['y'] - offset)
                        roi_x2 = min(processed_img.shape[1], bb['x'] + bb['width'] + offset)
                        roi_y2 = min(processed_img.shape[0], bb['y'] + bb['height'] + offset)

                        # Extract the ROI from the original image (not blurred)
                        roi = img[roi_y1:roi_y2, roi_x1:roi_x2]

                        # Apply Canny edge detection on the ROI only
                        edges_roi = cv2.Canny(roi, 100, 250)

                        # Convert the Canny edges of the ROI to BGR for display
                        canny_frame_roi = cv2.cvtColor(edges_roi, cv2.COLOR_GRAY2BGR)

                        # --- Hough Circle Transform for circle detection ---
                        # Detect circles using Hough Transform in the ROI
                        edges_roi_gray = np.uint8(edges_roi)
                        circles = cv2.HoughCircles(edges_roi_gray, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=100)

                        # Draw the detected circles (if any)
                        if circles is not None:
                            turn_pump(True)
                            circles = np.round(circles[0, :]).astype("int")
                            for (x, y, r) in circles:
                                # Draw the circle in green (thicker)
                                cv2.circle(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], (x, y), r, (0, 255, 0), 1)

                        # Find contours in the edges
                        contours, _ = cv2.findContours(edges_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                        farthest_contour = None
                        max_distance = 0

                        # Calculate the center of the ROI
                        roi_center_x = (roi_x1 + roi_x2) // 2
                        roi_center_y = (roi_y1 + roi_y2) // 2

                        # Iterate through all contours and filter out non-circular contours based on circularity
                        for contour in contours:
                            # Calculate the area and perimeter of the contour
                            area = cv2.contourArea(contour)
                            perimeter = cv2.arcLength(contour, True)

                            if perimeter == 0:  # Skip invalid contours
                                continue

                            # Calculate circularity
                            circularity = (4 * np.pi * area) / (perimeter ** 2)

                            # Only consider contours with high circularity (close to 1)
                            if circularity >= 0.8:
                                # Calculate the centroid of the contour
                                moments = cv2.moments(contour)
                                if moments['m00'] == 0:
                                    continue  # Skip if contour has no area

                                cx = int(moments['m10'] / moments['m00'])
                                cy = int(moments['m01'] / moments['m00'])

                                # Calculate the distance from the contour's centroid to the center of the ROI
                                distance = ((cx - roi_center_x) ** 2 + (cy - roi_center_y) ** 2) ** 0.5

                                # Check if this contour is the farthest
                                if distance > max_distance:
                                    max_distance = distance
                                    farthest_contour = contour

                        # Draw only the farthest contour and make it sparkle with a white glow
                        if farthest_contour is not None:
                            # Draw the contour in white with a thicker line (sparkling effect)
                            cv2.drawContours(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], [farthest_contour], -1, (255, 255, 255), 2)

                            # Draw a thinner green contour on top to make it more prominent
                            cv2.drawContours(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], [farthest_contour], -1, (0, 255, 0), 1)

                            # Calculate the centroid of the farthest contour
                            moments = cv2.moments(farthest_contour)
                            cx = int(moments['m10'] / moments['m00'])
                            cy = int(moments['m01'] / moments['m00'])

                            # Now calculate the distance to the edge of the detected circle
                            if circles is not None:
                                for (x, y, r) in circles:
                                    # Calculate the distance from the centroid of the farthest contour to the center of the circle
                                    distance_to_center = ((cx - x) ** 2 + (cy - y) ** 2) ** 0.5
                                    
                                    ## Get the average x and y coords from 10 readings of the contours
                                    ## Get the average distance based on the 10 readings after each 10 readings from avg x and y.

                                    # Calculate the distance to the edge of the circle (radius)
                                    distance_to_edge = distance_to_center - r
                                    if distance_to_edge >= -2:
                                        print("Container is full")
                                        turn_pump(False) # TOBEDELETED
                                        time.sleep(1)
                                        cv2.destroyAllWindows()
                                        sys.exit(0)
                                    print(f"Distance: {distance_to_edge} pixels")

                        # Draw the bounding box on the processed image
                        processed_img = cv2.rectangle(processed_img, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 1)

                    # Use the processed image (blurred with annotations)
                    # (No need to update blurred_img anymore, just use processed_img)

                    if show_camera:
                        # Combine the original and processed frames side by side
                        original_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        processed_frame = cv2.cvtColor(processed_img, cv2.COLOR_RGB2BGR)

                        # Resize the processed frame to match the size of the original frame
                        processed_frame_resized = cv2.resize(processed_frame, (original_frame.shape[1], original_frame.shape[0]))

                        # Combine the frames side by side (both frames should have the same dimensions now)
                        combined_frame = cv2.hconcat([original_frame, processed_frame_resized])

                        # Show the combined frame
                        cv2.imshow('Combined Frame (Original | Processed)', combined_frame)

                        # Resize the window to a specific width and height
                        cv2.resizeWindow('Combined Frame (Original | Processed)', 1200, 900)

                        # Wait for a key press
                        if cv2.waitKey(1) == ord('q'):
                            break


                next_frame = now() + 100

        finally:
            if runner:
                runner.stop()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import sys, getopt
import signal
import time
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner

runner = None
# if you don't want to see a camera preview, set this to False
show_camera = True
#if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
#    show_camera = False

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
            
            # Initialize variables to track total radius and area over 10 readings
            total_radius = 0
            total_x = 0
            total_y = 0
            valid_circle_count = 0
            frame_count = 0  # Counter for number of frames processed
            
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
                    
                    # Initialize variables to track the largest circle

                    # Loop through the bounding boxes to process each region

                    # Loop through the bounding boxes to process each region
                    for bb in res["result"]["bounding_boxes"]:
                        print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (
                            bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))

                        # Configurable offset for ROI size
                        offset = 70

                        # Ensure the ROI remains within the image boundaries
                        roi_x1 = max(0, bb['x'] - offset)
                        roi_y1 = max(0, bb['y'] - offset)
                        roi_x2 = min(processed_img.shape[1], bb['x'] + bb['width'] + offset)
                        roi_y2 = min(processed_img.shape[0], bb['y'] + bb['height'] + offset)

                        # Extract the ROI from the original image (not blurred)
                        roi = img[roi_y1:roi_y2, roi_x1:roi_x2]

                        # Apply Canny edge detection on the ROI only
                        edges_roi = cv2.Canny(roi, 100, 250)

                        # --- Hough Circle Transform for circle detection ---
                        # Detect circles using Hough Transform in the ROI
                        edges_roi_gray = np.uint8(edges_roi)
                        
                        cv2.imshow("Canny Edge", edges_roi)
                        circles = cv2.HoughCircles(edges_roi_gray, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=100)

                        # Track the largest circle for this frame
                        largest_circle = None
                        if circles is not None:
                            circles = np.round(circles[0, :]).astype("int")
                            for (x, y, r) in circles:
                                cv2.circle(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], (x, y), r, (0, 255, 0), 1)
                                # If the current circle has a larger radius than the previous largest, update
                                if largest_circle is None or r > largest_circle[2]:
                                    largest_circle = (x, y, r)

                        # If a largest circle is found, add its radius and area to the totals
                        if largest_circle is not None:
                            x, y, r = largest_circle
                            area = np.pi * r**2
                            #print(f"Largest Circle - Center: ({x}, {y}), Radius: {r}, Area: {area:.2f}, Frame: {frame_count:.2f}")

                            # Update totals
                            total_radius += r
                            total_x += x
                            total_y += y
                            valid_circle_count += 1

                        # Increment the frame count
                        frame_count += 1

                        # If we've processed 30 frames, calculate the averages
                        if frame_count >= 10:
                            if valid_circle_count > 0:
                                avg_radius = total_radius / valid_circle_count
                                avg_x = total_x / valid_circle_count
                                avg_y = total_y / valid_circle_count
                                
                                print(f"Average Radius: {avg_radius:.2f}")
                                print(f"Average X: {avg_x:.2f}, Average Y: {avg_y:.2f}")
                                
                                # Draw the average circle based on the calculated averages
                                cv2.circle(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], (int(avg_x), int(avg_y)), int(avg_radius), (255, 0, 0), 2)

                            # Reset the counters for the next batch of frames
                            total_radius = 0
                            total_x = 0  # Initialize total_x to accumulate x coordinates
                            total_y = 0  # Initialize total_y to accumulate y coordinates
                            valid_circle_count = 0
                            frame_count = 0  # Reset the frame counter after each 30-frame batch
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

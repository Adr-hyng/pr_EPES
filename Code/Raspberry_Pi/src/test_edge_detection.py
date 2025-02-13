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
            desired_width = 1280  # Example width
            desired_height = 720  # Example height
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
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

            # Hardcoded center, size, and offset
            offset = 40
            center = (48, 40)
            size = 40

            for res, img in runner.classifier(videoCaptureDeviceId):
                if next_frame > now():
                    time.sleep((next_frame - now()) / 1000)

                # Make a copy of the original frame for processing
                processed_img = img.copy()

                # Extract Region of Interest (ROI) based on hardcoded values
                roi_x1 = max(0, center[0] - offset)
                roi_y1 = max(0, center[1] - offset)
                roi_x2 = min(processed_img.shape[1], center[0] + size + offset)
                roi_y2 = min(processed_img.shape[0], center[1] + size + offset)

                # Extract the ROI from the original image (not blurred)
                roi = img[roi_y1:roi_y2, roi_x1:roi_x2]

                # Apply edge detection techniques
                # Apply Gaussian blur to the ROI
                blurred_img = cv2.GaussianBlur(roi, (7, 7), 0)
                
                # Apply Canny edge detection
                edges_roi = cv2.Canny(blurred_img, 50, 100)

                # Apply Sobel edge detection
                edges_sobel_x = cv2.Sobel(blurred_img, cv2.CV_64F, 1, 0, ksize=3)
                edges_sobel_y = cv2.Sobel(blurred_img, cv2.CV_64F, 0, 1, ksize=3)
                edges_sobel = cv2.magnitude(edges_sobel_x, edges_sobel_y)
                edges_sobel = np.uint8(np.clip(edges_sobel, 0, 255))

                # Apply Scharr edge detection
                edges_scharr_x = cv2.Scharr(blurred_img, cv2.CV_64F, 1, 0)
                edges_scharr_y = cv2.Scharr(blurred_img, cv2.CV_64F, 0, 1)
                edges_scharr = cv2.magnitude(edges_scharr_x, edges_scharr_y)
                edges_scharr = np.uint8(np.clip(edges_scharr, 0, 255))

                # Apply Laplacian edge detection
                edges_laplacian = cv2.Laplacian(blurred_img, cv2.CV_64F, ksize=3)
                edges_laplacian = np.uint8(np.clip(np.absolute(edges_laplacian), 0, 255))

                # Display edge detection results
                cv2.imshow("Canny Edge", edges_roi)
                cv2.imshow("Sobel Edge", edges_sobel)
                cv2.imshow("Scharr Edge", edges_scharr)
                cv2.imshow("Laplacian Edge", edges_laplacian)

                # --- Hough Circle Transform for circle detection ---
                # Detect circles using Hough Transform in the ROI
                edges_roi_gray = np.uint8(edges_roi)
                circles = cv2.HoughCircles(edges_roi_gray, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=100)

                # Track the largest circle for this frame
                largest_circle = None
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, r) in circles:
                        if largest_circle is None or (r > (largest_circle[2] * 0.7) and r <= (largest_circle[2] * 0.9)):
                            cv2.circle(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], (x, y), r, (255, 0, 0), 1)
                            largest_circle = (x, y, r)
                            
                            

                # If a largest circle is found, add its radius and area to the totals
                if largest_circle is not None:
                    x, y, r = largest_circle
                    area = np.pi * r**2
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
                        cv2.circle(processed_img[roi_y1:roi_y2, roi_x1:roi_x2], (int(avg_x), int(avg_y)), int(avg_radius), (255, 0, 0), 1)

                    # Reset the counters for the next batch of frames
                    total_radius = 0
                    total_x = 0
                    total_y = 0
                    valid_circle_count = 0
                    frame_count = 0  # Reset the frame counter after each 10-frame batch

                # Combine the original and processed frames side by side for display
                original_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                processed_frame = cv2.cvtColor(processed_img, cv2.COLOR_RGB2BGR)

                # Scale factor for enlarging the frames
                scale_factor = 3
                original_frame_resized = cv2.resize(original_frame, (int(original_frame.shape[1] * scale_factor), int(original_frame.shape[0] * scale_factor)))
                processed_frame_resized = cv2.resize(processed_frame, (original_frame_resized.shape[1], original_frame_resized.shape[0]))

                # Combine the frames side by side
                combined_frame = cv2.hconcat([original_frame_resized, processed_frame_resized])
                cv2.namedWindow('Combined Frame (Original | Processed)', cv2.WINDOW_NORMAL)

                # Show the combined frame
                cv2.imshow('Combined Frame (Original | Processed)', combined_frame)

                # Wait for a key press to quit
                if cv2.waitKey(1) == ord('q'):
                    break

                next_frame = now() + 100

        finally:
            if runner:
                runner.stop()


if __name__ == "__main__":
    main()

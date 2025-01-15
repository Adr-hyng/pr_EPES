import RPi.GPIO as GPIO
from time import sleep
import os
#Set warnings off (optional)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
#Set Button and LED pins

#Button Pins
Button_Pin = 14
Button1 = 14
Button2 = 15
Button3 = 18
Button4 = 17
Button5 = 23
Button6 = 24
Button7 = 10
LED1 = 25
LED2 = 8
LED3 = 7

Solenoid1 = 26
idk1 = 19
HotWater = 6
WarmWater = 13


#Setup Button and LED
GPIO.setup(Button1,GPIO.IN)
GPIO.setup(Button2,GPIO.IN)
GPIO.setup(Button3,GPIO.IN)
GPIO.setup(Button4,GPIO.IN)
#GPIO.setup(Button6,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
#GPIO.setup(Button7,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

#LED
#GPIO.setup(LED1,GPIO.OUT)
#GPIO.setup(LED2,GPIO.OUT)
#GPIO.setup(LED3,GPIO.OUT)

GPIO.setup(HotWater,GPIO.OUT)
GPIO.setup(WarmWater,GPIO.OUT)

#flag = 0
GPIO.output(WarmWater, GPIO.LOW);
GPIO.output(HotWater, GPIO.LOW);

try:
    while True: 
        GPIO.output(HotWater, GPIO.HIGH);
        sleep(1)
        GPIO.output(WarmWater, GPIO.HIGH);
        sleep(1)
        GPIO.output(WarmWater, GPIO.LOW);
        GPIO.output(HotWater, GPIO.LOW);
        sleep(2)
        #if(GPIO.input(Button4)) == GPIO.HIGH:
#            GPIO.output(idk2, GPIO.HIGH)
#        else:
#            GPIO.output(idk2, GPIO.LOW)
#        print(str(GPIO.input(Button1)) + " - "+str(GPIO.input(Button2)) + " - "+str(GPIO.input(Button3)) + " - "+str(GPIO.input(Button4)))
        #print("Button" + str(GPIO.input(Button3)))
        #print("Button 3 = " + str(GPIO.input(Button3)))
        #print("Button 4 = " + str(GPIO.input(Button4)))
        #print("Button 5 = " + str(GPIO.input(Button5)))
        #print("Button 6 = " + str(GPIO.input(Button6)))
        #print("Button 7 = " + str(GPIO.input(Button7)))
        sleep(0.2)
        os.system('clear')
except KeyboardInterrupt:
    print("exiting")
finally:
    GPIO.cleanup();

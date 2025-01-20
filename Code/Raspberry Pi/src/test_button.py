import RPi.GPIO as GPIO
from time import sleep
import os
#Set warnings off (optional)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
#Set Button and LED pins

#Button Pins
Button_Pin = 14

# Goods:
# 14, 2, 3

Button1 = 14 # Goods: 14, 2
Button2 = 4 # Change to other pin
Button3 = 2
Button4 = 3
Button5 = 25 # Hot
Button6 = 22 # Warm
Button7 = 9 # temp Lock
LED1 = 20
LED2 = 21
LED3 = 27

Solenoid1 = 26
Solenoid2 = 2
Solenoid3 = 3
Solenoid4 = 4
Solenoid5 = 10

idk1 = 6

Buzzer1 = 5
Buzzer2 = 6

Heater = 16
HotWater = 26
WarmWater = 19


#Setup Button and LED
GPIO.setup(Button1,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(Button2,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(Button3,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(Button4,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(Button5,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Button6,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Button7,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

#LED
GPIO.setup(LED1,GPIO.OUT)
GPIO.setup(LED2,GPIO.OUT)
GPIO.setup(LED3,GPIO.OUT)

#GPIO.setup(Solenoid1,GPIO.OUT)
#GPIO.setup(Solenoid2,GPIO.OUT)
#GPIO.setup(Solenoid3,GPIO.OUT)
#GPIO.setup(Solenoid4,GPIO.OUT)
#GPIO.setup(Solenoid5,GPIO.OUT)

GPIO.setup(HotWater,GPIO.OUT)
GPIO.setup(WarmWater,GPIO.OUT)
GPIO.setup(Heater,GPIO.OUT)

#flag = 0
#GPIO.output(Solenoid1, GPIO.LOW)
#GPIO.output(Solenoid2, GPIO.LOW)
#GPIO.output(Solenoid3, GPIO.LOW)
#GPIO.output(Solenoid4, GPIO.LOW)
#GPIO.output(Solenoid5, GPIO.LOW)

GPIO.output(WarmWater, GPIO.LOW)
GPIO.output(HotWater, GPIO.LOW)
GPIO.output(Heater, GPIO.LOW)

GPIO.output(LED1, GPIO.HIGH) # LOCK
GPIO.output(LED2, GPIO.HIGH)
GPIO.output(LED3, GPIO.HIGH) # HOT

try:
    while True:
#        GPIO.output(Heater, GPIO.HIGH)
#        sleep(4)
#        GPIO.output(Heater, GPIO.LOW)
#        sleep(4)
#        if(GPIO.input(Button4)) == GPIO.HIGH:
#            GPIO.output(idk2, GPIO.HIGH)
#        else:
#            GPIO.output(idk2, GPIO.LOW)
        print(str(GPIO.input(Button1)) + " - "+str(GPIO.input(Button2)) + " - "+str(GPIO.input(Button3)) + " - "+str(GPIO.input(Button4)) + " - " + str(int(not GPIO.input(Button5))) + " - " + str(int(not GPIO.input(Button6))) + " - " + str(int(not GPIO.input(Button7))))
#        print(str(GPIO.input(Button5)) + " - " + str(GPIO.input(Button6)) + " - " + str(GPIO.input(Button7)))

        #print("Button 5 = " + str(GPIO.input(Button5)))
        #print("Button 6 = " + str(GPIO.input(Button6)))
        #print("Button 7 = " + str(GPIO.input(Button7)))
        sleep(0.2)
        os.system('clear')
except KeyboardInterrupt:
    print("exiting")
finally:
    GPIO.cleanup();

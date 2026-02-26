# This file is used to make the light on the robot blink.
from gpiozero import LED
from time import sleep

def LED_Blink(pin, on_time=0.5, off_time=0.5):
    led = LED(pin)
    
    led.on()
    sleep(on_time)
    led.off()
    sleep(off_time)

        


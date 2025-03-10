from gpiozero import LED
from time import sleep

led = LED(27)
while True:
    print("it's on")
    led.on()
    sleep(1)
    led.off()
    print("it's off")
    sleep(1)    
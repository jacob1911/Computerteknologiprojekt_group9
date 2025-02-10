# coding: utf-8
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 18

print("Ultrasonic Measurement")

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output


# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

def measure():
    print(f"pin: {GPIO_TRIGECHO}\n")
  # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)

    print("GPIO.output works\n")

    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)

    print("GPIO.output works after false\n")
  #ensure start time is set in case of very quick return
    start = time.time()

    print("start is set\n")

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO)==0:
        start = time.time()

    print("while 1 exits\n")

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO)==1:
        stop = time.time()
  
    print("while 2 exits\n")

    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)

    print("Last weird thing has ran\n")

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    time.sleep(0.1)
    return distance

try:

    while True:
        print("in loop\n")
        distance = measure()
        print("  Distance : %.1f cm" % distance)
        time.sleep(1)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
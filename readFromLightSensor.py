import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here
        low_green = bus.read_i2c_block_data(0x44, 0x09)
        high_green = bus.read_i2c_block_data(0x44, 0x0A)
        # Convert the data to green, red and blue int values
        # Insert code here
        
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        # print("RGB(%d %d %d)" % (red, green, blue))
        print("\nlow green:")
        print(low_green)
        print("\nhigh green: ")
        print(high_green)
        
        time.sleep(2) 

getAndUpdateColour()
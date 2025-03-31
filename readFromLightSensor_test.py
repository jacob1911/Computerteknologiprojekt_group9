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

def convert_data(data, index):
    return data[index + 1] + data[index] / 256

threshold = 0.38

def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        # Convert the data to green, red and blue int values
        # Insert code here
        
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        # print("RGB(%d %d %d)" % (red, green, blue))
        print("\nNew reading:\n")
        green = convert_data(data, 0)
        red = convert_data(data, 2)
        blue = convert_data(data, 4)

        blue = blue * 2.35

        total = green + red + blue
        
        green = green / total
        red = red / total
        blue = blue / total

        print("\ncolor: ")

        if(green > threshold):
            print("green")
        elif(red > threshold):
            print("red")
        elif(blue > threshold):
            print("blue")
        else:
            print("no color detected")

        print(f"\ngreen: {green}")
        print(f"\nred: {red}")
        print(f"\nblue: {blue}")


        print(f"\nRaw data:{data}\n")


        
        time.sleep(2) 

getAndUpdateColour()
import json
import math
import time
import threading
import pygame

# Initialize Pygame for keyboard input
pygame.init()
screen = pygame.display.set_mode((300, 300))  # Small window to capture keyboard events
pygame.display.set_caption("TurtleBot Virtual Controller")

# Simulation variables
position = {"x": 0.0, "y": 0.0, "theta": 0.0}  # Initial position
linear_speed = 0.1  # Movement per step
angular_speed = 10  # Degrees per step
running = True

def update_position():
    """Update the robot's position based on keyboard inputs."""
    global position, running
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_w]:  # Move forward
            position["x"] += linear_speed * math.cos(math.radians(position["theta"]))
            position["y"] += linear_speed * math.sin(math.radians(position["theta"]))
        if keys[pygame.K_s]:  # Move backward
            position["x"] -= linear_speed * math.cos(math.radians(position["theta"]))
            position["y"] -= linear_speed * math.sin(math.radians(position["theta"]))
        if keys[pygame.K_a]:  # Turn left
            position["theta"] += angular_speed
        if keys[pygame.K_d]:  # Turn right
            position["theta"] -= angular_speed

        # Keep theta in range [0, 360)
        position["theta"] %= 360

        # Save position to JSON
        with open("turtlebot_position.json", "w") as f:
            json.dump(position, f, indent=4)

        # Print position (debugging)
        print(f"Position: x={position['x']:.2f}, y={position['y']:.2f}, θ={position['theta']:.1f}°")

        time.sleep(0.1)  # Small delay to prevent excessive CPU usage

# Start position update in a separate thread
threading.Thread(target=update_position, daemon=True).start()

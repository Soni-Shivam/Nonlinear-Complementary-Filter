import pygame
import numpy as np
from math import *
import serial
import time

# --- CONFIGURATION ---
WIDTH, HEIGHT = 1200, 600 # Widened window to fit 5 cubes
scale = 70                # Slightly smaller scale to fit them all
serial_port = '/dev/ttyUSB0' # Check your port!

# Filter labels matching the C++ print order
LABELS = ["Gyro Integration", "TRIAD (Meas)", "Direct Comp", "Passive Comp", "Explicit Comp"]

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (40, 40, 40)
YELLOW = (255, 255, 0)

# Define 5 positions on screen (2 rows: 3 on top, 2 on bottom)
# x, y coordinates
CUBE_CENTERS = [
    [200, 200],  # Top Left
    [600, 200],  # Top Middle
    [1000, 200], # Top Right
    [400, 500],  # Bottom Left
    [800, 500]   # Bottom Right
]

# --- GEOMETRY SETUP ---
points = []
# all the cube vertices (ENU frame)
points.append(np.matrix([-1, -1, .3]))
points.append(np.matrix([1, -1, .3]))
points.append(np.matrix([1,  1, .3]))
points.append(np.matrix([-1, 1, .3]))
points.append(np.matrix([-1, -1, -.3]))
points.append(np.matrix([1, -1, -.3]))
points.append(np.matrix([1, 1, -.3]))
points.append(np.matrix([-1, 1, -.3]))

# --- PYGAME SETUP ---
pygame.init()
pygame.display.set_caption("IMU Filter Comparison: 5-Way Split")
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
font = pygame.font.SysFont('Arial', 20, bold=True)

# Try connecting to serial
try:
    teensyPilot = serial.Serial(port=serial_port, baudrate=115200, timeout=.1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    print("Running in simulation mode (no data).")
    teensyPilot = None

# Global to store the latest 5 quaternions
# Default to Identity: w=1, x=0, y=0, z=0
current_quats = [[1, 0, 0, 0] for _ in range(5)] 

def connect_points(i, j, projected_points, color):
    pygame.draw.line(
        screen, color, (projected_points[i][0], projected_points[i][1]), (projected_points[j][0], projected_points[j][1]), 2)

def serial_read():
    global current_quats
    
    if teensyPilot is None:
        return

    while teensyPilot.in_waiting:
        try:
            line = teensyPilot.readline().decode("utf-8").strip()
            vals = line.split(',')
            
            # We expect 5 quaternions * 4 components = 20 values
            if len(vals) == 20:
                new_data = []
                # Parse 5 chunks of 4
                for i in range(5):
                    idx = i * 4
                    q = [float(vals[idx]), float(vals[idx+1]), float(vals[idx+2]), float(vals[idx+3])]
                    new_data.append(q)
                current_quats = new_data
            else:
                # partial line or garbage data
                pass
        except ValueError:
            pass

def get_rotation_matrix(q):
    """
    Convert quaternion [w, x, y, z] to 3x3 rotation matrix
    Using the formula provided in your original snippet
    """
    w, x, y, z = q

    q3q3 = y * y # q3 in your code was y? typically w,x,y,z -> 0,1,2,3
    q3q4 = y * z
    q2q2 = x * x
    q2q3 = x * y
    q2q4 = x * z
    q1q2 = w * x
    q1q3 = w * y
    q1q4 = w * z
    q4q4 = z * z

    # Based on your original math:
    # Note: q1=w, q2=x, q3=y, q4=z
    max_v = 1.0 - 2.0*(q3q3 + q4q4)
    may = 2.0*(q2q3 - q1q4)
    maz = 2.0*(q2q4 + q1q3)
    
    mbx = 2.0*(q2q3 + q1q4)
    mby = 1.0 - 2.0*(q2q2 + q4q4)
    mbz = 2.0*(q3q4 - q1q2)
    
    mcx = 2.0*(q2q4 - q1q3)
    mcy = 2.0*(q3q4 + q1q2)
    mcz = 1.0 - 2.0*(q2q2 + q3q3)

    DCM = np.matrix([
        [max_v, may, maz],
        [mbx, mby, mbz],
        [mcx, mcy, mcz],
    ])
    return DCM

def draw_cube(index, center_pos, label_text):
    # Get the specific quaternion for this index
    q = current_quats[index]
    
    # Compute Rotation Matrix
    DCM = get_rotation_matrix(q)
    
    projected_points = [[0, 0] for _ in range(len(points))]
    
    i = 0
    for point in points:
        # Rotate
        rotated2d = np.dot(DCM, point.reshape((3, 1)))
        
        # Project
        # Using your original projection logic
        distance = 10
        # Avoid divide by zero if z gets too close (clipping)
        denom = (distance - float(rotated2d[1][0]))
        if abs(denom) < 0.1: denom = 0.1
            
        z_factor = -1 / denom
        
        projection_matrix = np.matrix([
            [z_factor, 0, 0],
            [0, 0, -z_factor]
        ])
        
        projected2d = np.dot(projection_matrix, rotated2d)

        x = int(projected2d[0][0] * scale * distance) + center_pos[0]
        y = int(projected2d[1][0] * scale * distance) + center_pos[1]

        projected_points[i] = [x, y]

        # Draw vertices (Green/Red to indicate orientation)
        color = RED if float(point.reshape((3, 1))[1]) > 0 else GREEN
        pygame.draw.circle(screen, color, (x, y), 3)
        i += 1

    # Draw edges
    for p in range(4):
        connect_points(p, (p+1) % 4, projected_points, WHITE)
        connect_points(p+4, ((p+1) % 4) + 4, projected_points, WHITE)
        connect_points(p, (p+4), projected_points, WHITE)

    # Draw Label
    text_surface = font.render(label_text, True, YELLOW)
    text_rect = text_surface.get_rect(center=(center_pos[0], center_pos[1] + 100)) # Text below cube
    screen.blit(text_surface, text_rect)


# --- MAIN LOOP ---
while True:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                exit()

    # 1. Read latest data
    serial_read()

    # 2. Draw
    screen.fill(BLACK)
    
    # Iterate through the 5 filters and draw them at their respective positions
    for i in range(5):
        draw_cube(i, CUBE_CENTERS[i], LABELS[i])

    pygame.display.update()
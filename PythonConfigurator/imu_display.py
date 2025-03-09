import serial
import pygame
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import threading
import time

pygame.init()

width, height = 1200,1000
screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.OPENGL)

# Set up the perspective
gluPerspective(45, (width / height), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)

vertices=( (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1))

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )

yaw, pitch, roll = 0.0, 0.0, 0.0
new_yaw, new_pitch, new_roll = 0.0, 0.0, 0.0

def draw_cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def read_serial(stop_event):
    global new_yaw, new_pitch, new_roll
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip().split()
            # print(data)
            try:
                new_yaw = float(data[2])
                new_pitch = float(data[3])
                new_roll = float(data[4])
                
            except ValueError:
                print("Invalid data received, skipping...")

ser = serial.Serial(
    "COM5", 115200 
)

# Event to control when to stop the threads
stop_event = threading.Event()

# Create a thread for serial reading
thread1 = threading.Thread(target=read_serial, args=(stop_event,))

# Start the serial reading thread
thread1.start()

def render():
    global yaw, pitch, roll, new_yaw, new_pitch, new_roll
    setup = True
    while not stop_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_event.set()

        # Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Rotate the cube based on the new values
        offset_yaw = yaw - new_yaw
        yaw = new_yaw
        offset_pitch = pitch - new_pitch
        pitch = new_pitch
        offset_roll = roll - new_roll
        roll = new_roll
        # print(new_pitch, new_roll, new_yaw)

        glRotatef(offset_roll, 1, 0, 0)
        glRotatef(-offset_yaw, 0, 1, 0)
        glRotatef(-offset_pitch, 0, 0, 1)

        draw_cube()

        # Update the display
        pygame.display.flip()

        # Limit the frame rate (e.g., 60 FPS)
        pygame.time.wait(16)

    pygame.quit()

render()

thread1.join()

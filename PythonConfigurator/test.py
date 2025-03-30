import serial
import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
import time

# Initialize GLFW
glfw.init()

width, height = 1200, 1000
window = glfw.create_window(width, height, "3D Cube", None, None)
glfw.make_context_current(window)

# Set up the perspective
gluPerspective(45, (width / height), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)
glEnable(GL_DEPTH_TEST)  # Enable depth testing to make the cube solid

vertices = (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
)

edges = (
    (0, 1), (0, 3), (0, 4), (2, 1), (2, 3), (2, 7),
    (6, 3), (6, 4), (6, 7), (5, 1), (5, 4), (5, 7)
)

faces = (
    (0, 1, 2, 3),  # Back
    (3, 2, 7, 6),  # Left
    (6, 7, 5, 4),  # Front
    (4, 5, 1, 0),  # Right
    (1, 5, 7, 2),  # Top
    (4, 0, 3, 6)   # Bottom
)

colors = (
    (1, 0, 0),  # Red
    (0, 1, 0),  # Green
    (0, 0, 1),  # Blue
    (1, 1, 0),  # Yellow
    (1, 0, 1),  # Magenta
    (0, 1, 1)   # Cyan
)

new_yaw, new_pitch, new_roll = 0.0, 0.0, 0.0

def draw_cube():
    glBegin(GL_QUADS)
    for i, face in enumerate(faces):
        glColor3fv(colors[i])  # Set color for each face
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()

    glColor3f(1, 1, 1)  # White edges
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def read_serial(stop_event):
    global new_yaw, new_pitch, new_roll
    ser = serial.Serial("COM5", 115200)
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip().split()
            print(data)
            try:
                new_yaw = float(data[2])
                new_pitch = float(data[3])
                new_roll = float(data[4])
            except ValueError:
                print("Invalid data received, skipping...")

stop_event = threading.Event()
thread1 = threading.Thread(target=read_serial, args=(stop_event,))
thread1.start()

yaw, pitch, roll = 0.0, 0.0, 0.0
offset_yaw, offset_pitch, offset_roll = 0.0, 0.0, 0.0

def render():
    global new_yaw, new_pitch, new_roll
    while not glfw.window_should_close(window):
        glfw.poll_events()
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()  # Reset transformations
        
        glTranslatef(0.0, 0.0, -5)  # Move the camera back
        
        # Apply absolute rotations
        glRotatef(new_roll, 1, 0, 0)
        glRotatef(-new_yaw, 0, 1, 0)
        glRotatef(-new_pitch, 0, 0, 1)

        draw_cube()
        
        glfw.swap_buffers(window)
        time.sleep(0.016)  # ~60 FPS

    stop_event.set()
    thread1.join()
    glfw.terminate()

render()

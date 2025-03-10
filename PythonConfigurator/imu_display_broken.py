import serial
import pygame
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import threading
import numpy as np
from stl import mesh

pygame.init()

width, height = 1200, 1000
screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.OPENGL)

gluPerspective(45, (width / height), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)

font = pygame.font.SysFont("Arial", 20)

drone_mesh = mesh.Mesh.from_file('drone.stl')
print(f"Loaded {len(drone_mesh.vectors)} triangles from drone.stl")

yaw, pitch, roll = 0.0, 0.0, 0.0
new_yaw, new_pitch, new_roll = 0.0, 0.0, 0.0

def compute_centroid(model):
    all_vertices = np.concatenate(model.vectors)
    return np.mean(all_vertices, axis=0)

centroid = compute_centroid(drone_mesh)


def setup_lighting():
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, (1, 1, 1, 0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (1, 1, 1, 1))
    glLightfv(GL_LIGHT0, GL_SPECULAR, (1, 1, 1, 1))
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

def draw_model():
    glPushMatrix()
    
    for triangle in drone_mesh.vectors:
        glBegin(GL_TRIANGLES)
        for vertex in triangle:
            # vertex = vertex + (-centroid[0], -centroid[1], -centroid[2])
            glVertex3fv(vertex)
        glEnd()
    
    glPopMatrix()

def read_serial(stop_event):
    global new_yaw, new_pitch, new_roll
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip().split()
            try:
                new_yaw = float(data[2])
                new_pitch = float(data[3])
                new_roll = float(data[4])
            except:
                print("Invalid data received, skipping...")

ser = serial.Serial("COM5", 115200)
stop_event = threading.Event()
thread1 = threading.Thread(target=read_serial, args=(stop_event,))
thread1.start()

def render():
    global yaw, pitch, roll, new_yaw, new_pitch, new_roll
    setup_lighting()
    glScalef(0.01, 0.01, 0.01)
    # glRotatef(90, 1, 0, 1)

    while not stop_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_event.set()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glClearColor(0.0, 0.0, 0.0, 1.0)

        yaw_change = yaw - new_yaw

        pitch_change = pitch - new_pitch

        roll_change = roll - new_roll




        glPushMatrix()

        # Apply Pitch first (around the X-axis)
        glRotatef(roll_change, 1, 0, 0)  

        # Apply Yaw second (around the Y-axis)
        glRotatef(-yaw_change, 0, 1, 0)   

        # Apply Roll last (around the Z-axis)
        glRotatef(-pitch_change, 0, 0, 1)  

        glTranslatef(-centroid[0], -centroid[1], -centroid[2])

        draw_model()

        glPopMatrix()


        yaw_text = font.render(f"Yaw: {new_yaw:.2f}", True, (255, 255, 255))
        pitch_text = font.render(f"Pitch: {new_pitch:.2f}", True, (255, 255, 255))
        roll_text = font.render(f"Roll: {new_roll:.2f}", True, (255, 255, 255))
        screen.blit(yaw_text, (10, 10))
        screen.blit(pitch_text, (10, 40))
        screen.blit(roll_text, (10, 70))
        
        pygame.display.flip()
        pygame.time.wait(16)
    
    pygame.quit()

render()
thread1.join()

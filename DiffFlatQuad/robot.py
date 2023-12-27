import pygame
import numpy as np
from sympy import symbols, Matrix, sin, cos
import sympy as sp

class PlanerQuadrotor():
    def __init__(self, dt=0.01, rendering = True):
        self.x = np.array([0.0, 0.])  # Start at the center of the screen
        self.v = np.array([0., 0.])  # Start at the center of the screen
        self.omega = 0.
        self.theta = 0.0
        self.F = np.array([0., 0.])
        self.mass = 1.0
        self.J = 0.1
        self.L = 0.2
        self.gravity = np.array([0, 9.8])
        self.rendering = rendering
        self.dt = dt 
        self.symbolic_state = sp.Matrix([sp.Function(f'x{i}')(sp.symbols('t')) for i in range(1,7)])
        self.symbolic_input = sp.Matrix([sp.Function(f'u{i}')(sp.symbols('t')) for i in range(1,3)])
        if rendering:
            pygame.init()
            self.size = (1024, 768)
            self.screen = pygame.display.set_mode(self.size)
            pygame.display.set_caption('Quadrotor Simulator')
            self.clock = pygame.time.Clock()
            self.screen.fill((255, 255, 255))  # Clear screen
            self.color = (0, 0, 0)  # Red
            self.meter_to_pix = 100
            self.robot_img = pygame.image.load('DiffFlatQuad/quad.png')
            self.original_image = self.robot_img  # Store the original image to avoid degradation over time
            self.rect = self.robot_img.get_rect(center=self.x+np.array(self.size)//2)

    def reset(self):
        self.x = np.array([0., 0.])  # Start at the center of the screen
        self.v = np.array([0., 0.])  # Start at the center of the screen
        self.omega = 0.
        self.theta = 0.
        self.F = np.array([0., 0.])
        self.mass = 1.0
        self.gravity = np.array([0, 9.8])

    def step(self, T, F):
        self.x += self.v * self.dt
        self.v += self.dt*((F*np.array([-np.sin(self.theta), np.cos(self.theta)]))/self.mass-self.gravity)
        self.theta += self.omega*self.dt
        self.omega += self.dt*T/self.J
        if self.rendering:
            self.render()

    def render(self):
        self.screen.fill((255, 255, 255))  # Clear screen
        angle_degrees = np.degrees(self.theta) 
        self.robot_img = pygame.transform.rotate(self.original_image, angle_degrees)
        self.rect = self.robot_img.get_rect(center=np.array(self.size)/2 + np.diag([1,-1])@self.x*self.meter_to_pix)
        self.screen.blit(self.robot_img, self.rect)
        # pygame.display.flip()

    def running(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def getSymbolicF(self):
        """
        return the system dynamics f(x)
        x1 = x
        x2 = v_x
        x3 = y
        x4 = v_y
        x5 = theta
        x6 = omega
        """
        x = self.symbolic_state
        m, J, g = symbols('m,J, g')
        f = Matrix([[
            x[1],
            0,
            x[3],
            -g,
            x[5],
            0
        ]])
        return f.T
    
    def getSymbolicG(self):
        """
        return g(x). Not that u1 is thrust (F) and u2 is torque (T)
        """
        x = self.symbolic_state
        m, J, g = symbols('m,J, g')
        g = Matrix([[
            0,
            (-1/m)*sin(x[4]),
            0,
            (1/m)*cos(x[4]),
            0,
            0
        ],
        [
            0,
            0,
            0,
            0,
            0,
            1/J
        ]])
        return g.T
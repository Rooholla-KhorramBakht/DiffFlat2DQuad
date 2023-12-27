import time
import pygame

from DiffFlatQuad.robot import PlanerQuadrotor

def main():
    quadrotor = PlanerQuadrotor()
    while quadrotor.running():
        quadrotor.step(T=0.0000, F=9.8)
        time.sleep(0.01)

    pygame.quit()

if __name__ == '__main__':
    main()

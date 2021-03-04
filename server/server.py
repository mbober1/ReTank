import socket
import time
import pygame
from control import *



def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)         
    s.bind(('0.0.0.0', 8090 ))
    address = ('192.168.31.225', 8090);

    pygame.init()
    # logo = pygame.image.load("logo32x32.png")
    # pygame.display.set_icon(logo)
    pygame.display.set_caption("ReTank")
    screen = pygame.display.set_mode((240,180))

    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                s.close() 
                running = False
            
            elif event.type == pygame.KEYDOWN or event.type == pygame.KEYUP or event.type == pygame.JOYAXISMOTION:
                matrix = read_event(event)
                dupa = "L" + str(matrix[0]) + ";R" + str(matrix[1]);
                s.sendto(dupa.encode(), address)
                print(dupa)
                time.sleep(0.1)

if __name__=="__main__":
    main()
import pygame


def FindJoysticks():
    print("Found " + str(pygame.joystick.get_count()) + " devices")
    if pygame.joystick.get_count() > 1:
        for i in range(0, pygame.joystick.get_count()):
            print('[' + str(i) + ']' + pygame.joystick.Joystick(i).get_name())
        device = int(input("Choose one: "))
    else:
        device = 0

    return device


class Control_device():
    axis_data = None
    button_data = None
    power = None
    direction = None

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if(pygame.joystick.get_count()):
            try:
                self.keyboard = 0
                self.joy = pygame.joystick.Joystick(FindJoysticks())
                self.joy.init()
                print(self.joy.get_name(), ' connected')
                print(self.joy.get_numaxes(), 'Axes,',
                      self.joy.get_numbuttons(), 'Buttons')

            except pygame.error as message:
                self.keyboard = 1
                print("Keyboard control")

        else:
            self.keyboard = 1
            print("Keyboard control")




def read_event(event):
    l = 0
    r = 0
    if event.type == pygame.KEYDOWN:
        # print(event.key) #DEBUG
        if event.key == 97: #LEFT
            l = 0
            r = 255
        
        elif event.key == 100: #RIGHT
            l = 255
            r = 0
        
        elif event.key == 119: #UP
            l = 255
            r = 255
        
        elif event.key == 115: #DOWN
            l = -255
            r = -255

        elif event.key == 27: 
            exit()

        else: print(event.key)

    elif event.type == pygame.KEYUP:
        l = 0
        r = 0

    # else:
        # global steering
        # x = -int(steering.joy.get_axis(1)*255)
        # y = int(steering.joy.get_axis(0)*255)

        # if x > 255: x = 255
        # if x < -255: x = -255
        # if y > 255: y = 255
        # if y < -255: y = -255

        # if abs(x) < 10: x = 0
        # if abs(y) < 10: y = 0

    matrix = [l, r]
    # print('X: ', matrix[0], ' Y: ', matrix[1]) #DEBUG
    return matrix

steering = Control_device()

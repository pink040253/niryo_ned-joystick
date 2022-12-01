#!/usr/bin/env python

import pygame
import time
import math
import copy
import threading
from threading import *
from pygame.locals import *

from pyniryo import *

# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

robot_ip = "169.254.200.200"

client = NiryoRobot(robot_ip)
client.calibrate(CalibrateMode.AUTO)
client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
client.update_tool()
client.release_with_tool()
client.set_jog_control(True)

# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

# rendering thread of the program
class Rendering(threading.Thread):
    def __init__(self):
        Thread.__init__(self)  # init this class has a thread

        # Initialize the joysticks.
        # pygame.joystick.init()

    def run(self):
        print("pygame init")

        while 1:
            self.input()

    def input(self):
        global arm
        global tool_open
        global jog_failed
        global tool_update
        global to_origin
        global to_start

        # calculate jog value
        # jog_pose = [
        #     # arm[0] - robot_pose[0],
        #     # arm[1] - robot_pose[1],
        #     # arm[2] - robot_pose[2],
        #     arm[0],
        #     arm[1],
        #     arm[2],
        #     0,
        #     0,
        #     0
        # ]
        jog_pose = arm

        robot_pose = client.get_pose().to_list()
        # print(robot_pose)
        # print("----------", client.get_joints())
        # print(robot_pose[2], jog_pose[2])
        if jog_pose[2] < 0 and robot_pose[2] + jog_pose[2] < 0.29 :
            jog_pose[2] = 0

        # print(arm)
        # print(jog_pose)

        try:
            client.jog_pose(*jog_pose)
            jog_failed = 0
        except NiryoRobotException:
            jog_failed = 1

        if tool_update:
            # print("switch tool")
            tool_update = False
            client.set_jog_control(False)

            if tool_open:
                client.release_with_tool()
                print("open")
            else:
                client.grasp_with_tool()
                print("close")
            client.set_jog_control(True)

        if to_origin == True:
            to_origin = False
            client.set_jog_control(False)
            client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
            client.set_learning_mode(True)
            # client.set_jog_control(True)
        
        if to_start == True:
            to_start = False
            client.set_jog_control(False)
            client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
            client.set_learning_mode(True)
        
        if done == True:
            client.set_jog_control(False)
            client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
            # client.set_learning_mode(True)


pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set the width and height of the screen (width, height).
screen = pygame.display.set_mode((500, 700))
pygame.display.set_caption("AI fly")
# Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates.
clock = pygame.time.Clock()
# Initialize the joysticks.
pygame.joystick.init()
# Get ready to print.
textPrint = TextPrint()

arm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tool_update = False
jog_failed = False
tool_open = True
to_origin = False
to_start = False

# -------- Main Program Loop -----------
render_thread = Rendering()
render_thread.daemon = True
render_thread.start()

while not done:

    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT or joystick.get_button(8) == 1: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
            time.sleep(2.5)
            exit()
            # break
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
            if joystick.get_button(4) == 1:
                print("RB")
                tool_open = not tool_open
                tool_update = True
            if joystick.get_button(6) == 1:
                to_origin = True
            if joystick.get_button(7) == 1:
                to_start = True
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
    
    # arm[0] = round(-1 * joystick.get_axis(3), 2) if abs(round(-1 * joystick.get_axis(3), 2)) > 0.15 else 0
    arm[0] = 0.02 * (-1 * list(joystick.get_hat(0))[1])
    # arm[1] = round(-1 * joystick.get_axis(2), 2) if abs(round(-1 * joystick.get_axis(2), 2)) > 0.15 else 0
    arm[1] = 0.02 * (list(joystick.get_hat(0))[0])
    arm[2] = 0.02 * (round(-1 * joystick.get_axis(3), 2) if abs(round(-1 * joystick.get_axis(3), 2)) > 0.2 else 0)
    # arm[2] = list(joystick.get_hat(0))[1]
    arm[4] = 0.05 * (1 if joystick.get_button(3) == 1 else -1 if joystick.get_button(0) == 1 else 0)
    arm[5] = 0.05 * (-1 if joystick.get_button(2) == 1 else 1 if joystick.get_button(1) == 1 else 0)



    screen.fill(WHITE)
    textPrint.reset()
    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    # For each joystick:
    # for i in range(joystick_count):
    #     joystick = pygame.joystick.Joystick(i)
    #     joystick.init()

    textPrint.tprint(screen, "Joystick {}".format(0))
    textPrint.indent()

    # for i in range(axes):
        # axis = joystick.get_axis(i)
        # textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
    # textPrint.unindent()

    axis0 = joystick.get_axis(0)

    textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(0, axis0))

    axis1 = joystick.get_axis(1)
    textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(1, axis1))

    axis2 = joystick.get_axis(2)
    textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(2, axis2))

    axis3 = joystick.get_axis(3)
    textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(3, axis3))

    axis4 = joystick.get_axis(4)
    textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(4, axis4))

    axis5 = joystick.get_axis(5)
    textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(5, axis5))

    # axis6 = joystick.get_axis(6)
    # textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(6, axis6))

    # axis7 = joystick.get_axis(7)
    # textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(7, axis7))

    buttons = joystick.get_numbuttons()
    textPrint.tprint(screen, "Number of buttons: {}".format(buttons))
    textPrint.indent()

    for i in range(buttons):
        button = joystick.get_button(i)
        textPrint.tprint(screen,
                            "Button {:>2} value: {}".format(i, button))
    textPrint.unindent()

    hats = joystick.get_numhats()
    textPrint.tprint(screen, "Number of hats: {}".format(hats))
    textPrint.indent()

    # Hat position. All or nothing for direction, not a float like
    # get_axis(). Position is a tuple of int values (x, y).
    for i in range(hats):
        hat = joystick.get_hat(i)
        # print(type(list(hat)[1]))
        textPrint.tprint(screen, "Hat {} value: {}".format(i, str(hat)))
    textPrint.unindent()

    textPrint.unindent()

    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    #
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    # Limit to 20 frames per second.
    # clock.tick(20)
# client.set_jog_control(False)
client.set_learning_mode(True)
# client.close_connection()
pygame.quit()
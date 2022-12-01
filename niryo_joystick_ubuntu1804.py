#!/usr/bin/env python

import time
import threading
from threading import *
import pygame
from pygame.locals import *

from pyniryo import *

robot_ip = "169.254.200.200"

# range of the robot compare to his height
# This limit variable is used to calculate the radius of the white circle displayed in the graphic interface
# according to the current height of the end effector.
# Higher the end's effector is (Or low, if Ned can access negative height) (represented by the first element of each
# tuple), smaller the radius of the white circle will be (represented by the second element
# of each tuple). When the end effector reach is optimal height, the accessible area is the larger, so is the radius.

client = NiryoRobot(robot_ip)
client.calibrate(CalibrateMode.AUTO)
# client.set_arm_max_velocity(100)
# client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
client.update_tool()
client.release_with_tool()
client.set_jog_control(True)

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
# joystick.init()

# - Pygame methods

def exit_game():
    client.set_jog_control(False)
    client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
    client.set_learning_mode(True)
    client.close_connection()
    exit()


# rendering thread of the program
class Rendering(threading.Thread):
    def __init__(self):
        Thread.__init__(self)  # init this class has a thread

        # pygame init and variables definition
        # pygame.init()

        # Initialize the joysticks.
        # pygame.joystick.init()

    def run(self):
        print("pygame init")

        while 1:
            self.input()

    def input(self):
        global arm
        global tool_open
        global to_origin
        global to_start
        global running

        # pos = list(pygame.mouse.get_pos())
        # joystick = pygame.joystick.Joystick(0)
        # joystick.init()

        for event in pygame.event.get():
            if joystick.get_button(9) == 1:
                print("Bye~")
                running = False
                # time.sleep(2.5)
                exit()
            elif event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
                # print(event.type)
                if joystick.get_button(4) == 1:
                    print("RB")
                    global tool_update
                    tool_open = not tool_open
                    tool_update = True
                if joystick.get_button(6) == 1:
                    to_origin = True
                if joystick.get_button(7) == 1:
                    to_start = True
            # elif event.type == pygame.JOYBUTTONUP:
            #     print("Joystick button released.")

        # arm[0] = round(-1 * joystick.get_axis(3), 2) if abs(round(-1 * joystick.get_axis(3), 2)) > 0.15 else 0
        arm[0] = -1 * list(joystick.get_hat(0))[1]
        # arm[1] = round(-1 * joystick.get_axis(2), 2) if abs(round(-1 * joystick.get_axis(2), 2)) > 0.15 else 0
        arm[1] = list(joystick.get_hat(0))[0]
        arm[2] = round(-1 * joystick.get_axis(4), 2) if abs(round(-1 * joystick.get_axis(4), 2)) > 0.2 else 0
        # arm[2] = list(joystick.get_hat(0))[1]
        arm[4] = 1 if joystick.get_button(3) == 1 else -1 if joystick.get_button(0) == 1 else 0
        arm[5] = -1 if joystick.get_button(2) == 1 else 1 if joystick.get_button(1) == 1 else 0


# - Process methods

# arm = list(pose[0:3])
arm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# value share between the GUI and main Thread
jog_failed = False  # last jog failed ?
tool_open = True  # state of the tool
tool_update = False  # gui ask for tool action
running = True  # turn to fasle when the program is closed finished
tool_open = True
to_origin = False
to_start = False

# start the GUI thread
render_thread = Rendering()
render_thread.start()

while running:

    jog_pose = arm

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

    for x in range(len(jog_pose)):
        if x <= 4:
            jog_pose[x] *= 0.001
        elif x == 4:
            jog_pose[x] *= 0.0002
        elif x == 5:
            jog_pose[x] *= 0.001
    # robot_pose = client.get_pose().to_list()

    # print(robot_pose)
    # print("----------", client.get_joints())
    # print(robot_pose[2], jog_pose[2])
    # if jog_pose[2] < 0 and robot_pose[2] + jog_pose[2] < 0.29 :
    #     jog_pose[2] = 0

    # print(arm)
    print(jog_pose)

    try:
        client.jog_pose(*jog_pose)
        jog_failed = 0
    except NiryoRobotException:
        jog_failed = 1

    if tool_update:
        print("switch tool")
        tool_update = False
        client.set_jog_control(False)

        if tool_open:
            client.release_with_tool()
        else:
            client.grasp_with_tool()
        client.set_jog_control(True)

    if to_origin == True:
        to_origin = False
        client.set_jog_control(False)
        client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
        # client.set_learning_mode(True)
        client.set_jog_control(True)
    
    if to_start == True:
        to_start = False
        client.set_jog_control(False)
        client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
        # client.set_learning_mode(True)
        client.set_jog_control(True)
    
    if running == False:
        exit_game()
    
    time.sleep(0.05)

pygame.quit()
exit_game()
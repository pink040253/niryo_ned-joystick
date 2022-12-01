#!/usr/bin python3
A = 304         # BTN_A
B = 305         # BTN_B
X = 307         # BTN_X
Y = 308         # BTN_Y
LB = 310        # BTN_TL
RB = 311        # BTN_TR
SELECT = 314    # BTN_SELECT
START = 315     # BTN_START
MODE = 316      # BTN_MODE
BTN_THUMBL = 317

ABS_X = 0         # -32767 ~ 32767 (left ~ right)
ABS_Y = 1         # -32767 ~ 32767 (up ~ down)
ABS_Z = 2         # 0 ~ 1023 (free ~ push)
ABS_RX = 3        # -32767 ~ 32767 (left ~ right)
ABS_RY = 4        # -32767 ~ 32767 (up ~ down)
ABS_RZ = 5        # 0 ~ 1023 (free ~ push)
ABS_HAT0X = 16    # -1, 1 (left ~ right)
ABS_HAT0Y = 17    # -1, 1 (up ~ down)


from evdev import InputDevice, categorize, ecodes
import time
import threading
from threading import *
import paho.mqtt.client as mqtt
from pyniryo import *
import numpy as np

# Connect with niryo robot
# robot_ip = "10.10.10.110"
robot_ip = "10.10.10.10"
# robot_ip = "127.0.0.1"
client = NiryoRobot(robot_ip)

# mqtt_Broker = "10.10.10.110"
# mqtt_Port = 1883    # mqtt 1883
# mqtt_Alive = 60
# mqtt_Topic = "niryo"

# Initialize niryo robot
client.calibrate(CalibrateMode.AUTO)
# client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
client.move_joints(0, 0.25, -0.21, 0, -1.57, 0)
client.update_tool()
client.release_with_tool()
client.set_jog_control(True)


# (0, 0.25, -0.21, 0, -1.57, 0)

# Get joystick device
dev = InputDevice('/dev/input/event0')
# print(dev)
# print(dev.capabilities())
# print(dev.capabilities(absinfo=False))

# mqtt
class mqtt_niryo(threading.Thread):
    def __init__(self):
        Thread.__init__(self)
    
    def subscribe(self):
        client = mqtt.Client("PC")

        # client.username_pw_set("niryo","robotics")

        # client.loop_start()

        client.on_connect=on_connect
        client.on_message=on_message 
        client.connect(mqtt_Broker, mqtt_Port, mqtt_Alive)

        client.loop_forever()
    
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe("niryo")

    def on_message(self, client, userdata, message):
        received_msg = str(message.payload.decode("utf-8"))
        print("received message: ", received_msg)
        publisher(client, received_msg)

    def publisher(self, client, received_msg):
        if received_msg == "Hello":
            msg = "Hi"
            client.publish(mqtt_Topic, msg)
            print("Just published " + str(msg) + " to topic " + mqtt_Topic)
        elif received_msg == "Hoo":
            msg = "Ho"
            client.publish(mqtt_Topic, msg)
            print("Just published " + str(msg) + " to topic " + mqtt_Topic)


# Rendering thread of the program
class Rendering(threading.Thread):
    def __init__(self):
        Thread.__init__(self)  # init this class has a thread

        # Initialize the joysticks.
        # pygame.joystick.init()

    def run(self):
        print("ready")

        while 1:
            self.input()

    def input(self):
        global arm
        global tool_open
        global jog_failed
        global tool_update
        global to_origin
        global to_start
        global end_project

        if arm != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
            robot_pose = client.get_pose().to_list()
            # robot_joints = np.round(client.get_joints(), 2)
            # print(robot_pose)
            # print("----------", client.get_joints())
            # print(robot_pose[2], arm[2])
            if arm[2] < 0 and robot_pose[2] + arm[2] < 0.2 :
                arm[2] = 0

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
            # robot_pose_next = robot_pose

            # for i in range(len(robot_pose_next)):
            #     robot_pose_next[i] += jog_pose[i]
            
            # robot_joints_next = client.inverse_kinematics(*robot_pose_next)

            # jog_joints = robot_joints_next
            # for i in range(len(jog_joints)):
            #     jog_joints[i] -= robot_joints[i]
            # print(jog_joints)

            # max_d = 0.1
            # dxy = (jog_pose[0] ** 2 + jog_pose[1] ** 2 + jog_pose[2] ** 2) ** 0.5
            # if dxy > max_d:
            # #     jog_pose[0] /= dxy / max_d
            # #     jog_pose[1] /= dxy / max_d
            # #     jog_pose[2] /= dxy / max_d
            #     for i in range(len(jog_joints)):
            #         jog_pose[i] /= dxy / max_d

            # print(arm)
            # print(jog_pose)

            self.jog(*jog_pose)
            # try:
            #     client.jog_pose(*jog_pose)
            #     # client.jog_joints(*jog_joints)
            #     # jog_failed = 0
            # except NiryoRobotException:
            #     print("IK error")
            #     client.set_jog_control(False)
            #     time.sleep(0.3)
            #     client.set_jog_control(True)
            #     # jog_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #     # client.jog_pose()
            #     # jog_failed = 1
            
            time.sleep(0.1)

        if tool_update:
            # print("switch tool")
            tool_update = False
            client.set_jog_control(False)

            if tool_open == True:
                client.release_with_tool()
                print("open")
            else:
                client.grasp_with_tool()
                print("close")
            client.set_jog_control(True)
            self.jog(0, 0, 0.003, 0, 0, 0)
            # client.jog_pose(0, 0, 0.003, 0, 0, 0)

        # if to_origin == True:
        #     to_origin = False
        #     client.set_jog_control(False)
        #     client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
            # client.set_learning_mode(True)
            # client.set_jog_control(True)
        
        if to_start == True:
            to_start = False
            client.set_jog_control(False)
            client.move_joints(0, 0.25, -0.21, 0, -1.57, 0)
            client.release_with_tool()
            # client.set_learning_mode(True)
        
        if end_project == True:
            client.set_jog_control(False)
            client.move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)
            client.set_learning_mode(True)
            client.close_connection()
            exit()

    def jog(self, *pose):
        try:
            client.jog_pose(*pose)
            # client.jog_joints(*jog_joints)
            # jog_failed = 0
        except NiryoRobotException:
            print("IK error")
            client.set_jog_control(False)
            time.sleep(0.3)
            client.set_jog_control(True)
            # jog_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # client.jog_pose()
            # jog_failed = 1

# Parameters
arm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tool_update = False
jog_failed = False
tool_open = True
to_origin = False
to_start = False
end_project = False
abs_value = {ABS_X: 0.0, ABS_Y: 0.0, ABS_Z: 0.0, 
             ABS_RX: 0.0, ABS_RY: 0.0, ABS_RZ: 0.0, 
             ABS_HAT0X: 0.0, ABS_HAT0Y: 0.0}
btn_value = {A: 0, B: 0, X: 0, Y: 0,
             LB: 0, RB: 0, SELECT: 0, START: 0, MODE: 0}

# Start rendering thread
render_thread = Rendering()
render_thread.start()

# # Start mqtt thread
# mqtt_thread = mqtt_niryo()
# mqtt_niryo.start()

# Read joystick event data 
for event in dev.read_loop():
    # print(dev.read_loop())
    if event.type == ecodes.EV_KEY:
        # print("Joystick button pressed.")
        # print(categorize(event))
        data = repr(event).split(")")[0].split(", ")[-2:]
        if int(data[0]) in [A, B, X, Y]:
            btn_value[int(data[0])] = int(data[1])
        if int(data[0]) == LB and int(data[1]) == 1:
            # print("LB") # open
            tool_open = True
            tool_update = True
            time.sleep(0.5)
        if int(data[0]) == RB and int(data[1]) == 1:
            # print("LB") # close
            tool_open = False
            tool_update = True
            time.sleep(0.5)
        # if int(data[0]) == SELECT and int(data[1]) == 1:
        #     to_origin = True
        # if int(data[0]) == START and int(data[1]) == 1:
        if int(data[0]) == MODE and int(data[1]) == 1:
            to_start = True
        # if int(data[0]) == BTN_THUMBL and int(data[1]) ==1:
        #     end_project = True
        #     break
        # print(data[0], btn_value)
    if event.type == ecodes.EV_ABS:
        # print(categorize(event))
        data = repr(event).split(")")[0].split(", ")[-2:]
        if int(data[0]) in [ABS_RY]:    # ABS_X, ABS_Y, ABS_RX
            abs_value[int(data[0])] = round(int(data[1])/32767, 2)
            # print(int(data[1]), abs_value[int(data[0])])
        # elif int(data[0]) in [ABS_Z, ABS_RZ]:
        #     abs_value[int(data[0])] = round(int(data[1])/1023, 2)
        elif int(data[0]) in [ABS_HAT0X, ABS_HAT0Y]:
            abs_value[int(data[0])] = int(data[1])
        # print(data[0], abs_value)
    
    arm[0] = 0.02 * (abs_value[ABS_HAT0Y])
    arm[1] = 0.02 * (abs_value[ABS_HAT0X])
    arm[2] = 0.02 * (-1 * abs_value[ABS_RY] if abs(abs_value[ABS_RY]) > 0.2 else 0)
    arm[4] = 0.03 * (1 if btn_value[Y] == 1 else -1 if btn_value[A] == 1 else 0)
    arm[5] = 0.05 * (-1 if btn_value[X] == 1 else 1 if btn_value[B] == 1 else 0)

    time.sleep(0.001)

# return 0


class My_Ned():
    def __init__(self):
        self.base_hieght = 0.114
        self.L10 = 0.08
        self.L11 = -0.025 
        self.r1 = 0.21
        self.L30 = 0.03
        self.L31 = 0.041
        # self.r2 = (self.L30**2 + self.L31**2)**0.5
        self.L4 = 0.18 # m
        self.r2 = (self.L30**2 + (self.L31+self.L4)**2)**0.5
        self.L5 = 0.0237
        self.L_end = 0.232
        self.grap = np.array([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 1]])
        self.joint = [radians(0), radians(0), radians(0), 0, 0, 0]
    
    def rotation_X(self,rad):
        rotation = np.array([[1,     0       ,        0       ],
                             [0,cos(rad),-1*sin(rad)],
                             [0,sin(rad),   cos(rad)]])
        return rotation
    def rotation_Y(self,rad):
        rotation = np.array([[cos(rad), 0, -1*sin(rad)],
                             [  0     , 1,     0      ],
                             [sin(rad), 0,    cos(rad)]])
        return rotation
    def rotation_Z(self,rad):
        rotation = np.array([[cos(rad), -1*sin(rad),0],
                             [sin(rad),    cos(rad),0],
                             [0,0,1]])
        return rotation
    
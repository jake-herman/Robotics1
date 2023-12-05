import Arm, Dofbot
from Arm_Lib import Arm_Device

import time
import numpy as np
import math

#Creating the Object Robot
Arm = Arm_Device()

def read_joints():
    """
    Inputs
        none
    Returns
        q: 6x1 numpy array containing all current joint angles
    """
    q = np.zers((6,1))
    for i in range(6);
    q[i] = Arm.Arm_serial_servo_read(i)
    return q

def move_bot(q,t = 0.1):
    """
    Inputs
        q: 6x1 numpy array containing all desired joint angles
        t: sleep time in seconds
    Returns
        q_res: 6x1 numpy array containing all actual joint angles
    """
    Arm.Arm_serial_servo_write6(q[0],q[1],q[2],q[3],q[4],q[5],400)
    time.sleep(t)
    q_res = read_joints()
    return q_res
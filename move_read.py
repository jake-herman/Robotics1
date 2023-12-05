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
    q = np.zeros(6)
    for i in range(1,7):
        
        q[i-1] = Arm.Arm_serial_servo_read(i)
        if math.isnan(q[i-1]):
            q[i-1] = 90
    return q

def move_bot(q,t_move = 1,t_sleep = 0.05,disp = True):
    """
    Inputs
        q: 6x1 numpy array containing all desired joint angles
        t_move: move time in seconds
        t_sleep = sleep time in seconds
    Returns
        q_res: 6x1 numpy array containing all actual joint angles
    """
    t_sleep = t_move+t_sleep
    t_move = int(t_move*1000)
    Arm.Arm_serial_servo_write6(q[0],q[1],q[2],q[3],q[4],q[5],t_move)
    time.sleep(t_sleep)
    q_res = read_joints()
    if disp == True:
        return q_res
    

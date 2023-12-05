import numpy as np
import move_read
import math

#Generate Path Function
def gen_joint_space_path(start, end, num):
    path = []
    for i  in range(num+1):
        a = i / (num)
        interp = (1 - a) * start + a * end
        path.append(interp)
    return path

def path_follow(path,t,num):
    t_ind = t/num
    error = np.zeros((len(path),6))
    move_read.move_bot(path[1],.5,0)
    q_act = move_read.read_joints()
    error[0,:] = path[0] - q_act
    
    for i in range(1,len(path)):
        move_read.move_bot(path[i],t_ind,0)
        q_act = move_read.read_joints()
        error[i,:] = path[i] - q_act
    return(error)

def pid_path_follow(path, t, num, kp, ki, kd):
    path = np.array(path)
    t_ind = t / num
    error = np.zeros((len(path), 6))
    I = np.zeros(6)
    prev_error = np.zeros(6)
    path_new = path
    move_read.move_bot(path_new[1], 1, 0)
    error[0, :] = path[0] - move_read.read_joints()
    
    for i in range(1,len(path)):
        P = kp * error[i-1, :]
        I += ki * t_ind * error[i-1, :]
        D = kd * (error[i-1, :] - prev_error) / t_ind
        
        # PID control
        u = P + I + D
        prev_error = error[i, :]
        
        path_new[i,:] += u
        move_read.move_bot(path_new[i], t_ind, 0)
        q_act = move_read.read_joints()
        error[i, :] = path[i] - q_act
        # print(u)
        
    return error

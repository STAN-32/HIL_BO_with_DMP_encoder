import rospy
from std_msgs.msg import Float32
from sentry_msgs.msg import limb

import sys
print("Current Python interpreter path:", sys.executable)

import os

current_dir = os.path.dirname(os.path.realpath(__file__))

print("Current working directory:", current_dir)
if current_dir not in sys.path:
    sys.path.append(current_dir)

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as scio
from sklearn.metrics import mean_squared_error
import math as m
import threading
from bayes_opt import BayesianOptimization
import time
import scipy.io as sio
import pickle
import pdb


Num_opt =  40
qd_flag = 0

Start_Ready_flag = False
init_wait = True

class Cost_BO:
    def __init__(self):
        self.s = None
        self.q1 = None
        self.q2 = None
        self.q1_d = None
        self.q2_d = None
        self.Cost_all = 0
        self.lamda = 0.5
        self.counter = 0  
        self.receive_flag = 1  
        self.threshold = 2  

        self.q1_d_pre = None
        self.cost_counter = 0

    def update(self):
        if (self.receive_flag == 0):
            if self.q1_d is not None and self.q2_d is not None:
                self.Cost_all += (self.q1-self.q1_d)**2 + (self.q2-self.q2_d)**2 + self.lamda * (self.s**2) * 100
                self.cost_counter += 1
    
    def init_cost(self):
        self.Cost_all = 0
        self.cost_counter = 0
        self.receive_flag = 0
        self.counter = 0 

    def check_counter(self):
        if self.q1_d is None or self.q1_d_pre is None:
            return
        if self.q1_d < 0 and self.q1_d_pre > 0:
            if abs(self.q1_d) + abs(self.q1_d_pre) > 1:
                print('Zero Error: ', abs(self.q1_d) + abs(self.q1_d_pre))
                return
            self.counter += 1
            print(self.counter)
            if self.counter >= self.threshold:
                self.receive_flag = 1
                


cost_bo = Cost_BO()

filename = 'twojoint.mat'
filepath = current_dir + '/twojoint.mat'
try:
    mat_data = sio.loadmat(filepath)
    start_q2 = mat_data['start_q2'][0][0]
    start_q4 = mat_data['start_q4'][0][0]
    weight_q2 = mat_data['weight_q2'][0]
    weight_q4 = mat_data['weight_q4'][0]
except FileNotFoundError:
    print(filepath)
    print(f'File {filename} not exist')

def Matlab_range(start,step,end):
    return np.arange(start,end+step,step)



class DMP_class():
    def __init__(self,start,low,high) -> None:
        self.z = 0
        self.y = 0
        self.t_pre = None
        self.start = start
        self.lower_bound = low
        self.higher_bound = high
    
    def update(self, weight, t_crt):
        if self.t_pre == None:
            t_s = 0.001
            self.t_pre = t_crt
            self.t_start = t_crt
        else:
            t_s = t_crt - self.t_pre
            # t_s = 0.001
            self.t_pre = t_crt
        t_DMP = Matlab_range(0,t_s,1)*2*m.pi # to fulfill the slice like matlab: start,end+step,step

        N = 20
        alphaz = 30
        betaz = alphaz/4
        h = 2.5*N
        c = Matlab_range(0,2*m.pi/(N-1),2*m.pi)
        r = 1
        g = 0
        omiga = 2*m.pi

        T_per = 1

        phi = omiga*(t_crt-self.t_start)*0.25*1
        Phi_i = np.exp(h * (np.cos(phi - c) - 1))
        f_nolinear = sum((weight * Phi_i).squeeze()) * r / sum(Phi_i.squeeze())
        z_new = self.z + omiga*(alphaz*(betaz*(g-self.y)-self.z)+f_nolinear)*t_s
        y_new = self.y + omiga * self.z * t_s
        self.z = z_new
        self.y = y_new

        self.output = (self.y)*(self.higher_bound - self.lower_bound) + self.start*180/m.pi
joint0 = DMP_class(start_q2,-30,40)
joint1 = DMP_class(start_q4,0,80)

def target_function(**kwargs):
    global weight_q2, weight_q4, Start_Ready_flag
    
    x = np.array([kwargs["x"+str(i)] for i in range (Num_opt)])
    weight_q2 = x[:int(Num_opt/2)]
    weight_q4 = x[-int(Num_opt/2):]
    
    while True:
        if Start_Ready_flag:
            if cost_bo.receive_flag == 1:
                break
        rospy.sleep(0.01)
    print('Current Cost: ', cost_bo.Cost_all/cost_bo.cost_counter)
    res = -cost_bo.Cost_all/cost_bo.cost_counter
    return res

initial_values = np.concatenate((weight_q2, weight_q4))


def callback_float(data):
    cost_bo.s = data.data
    if init_wait == False:
        cost_bo.update()

def callback_limb(data):
    cost_bo.q1 = data.q2  
    cost_bo.q2 = data.q4  


class My_BO:
    def __init__(self) -> None:
        
        self.first_flag = 0
        self.total_iterations = 200
        self.no_improve_threshold = 10
        self.no_improve_count = 0
        self.best_target = None
        inintial_bound = 0.3
        pbounds = {f'x{i}': (val - inintial_bound * abs(val), val + inintial_bound * abs(val)) for i, val in enumerate(initial_values)}
        self.bound = inintial_bound
        self.rate = 0.5
        self.converged_flag = 0
        self.optimize_times = 0
        self.finish_time = None
        self.random_times = 5
        self.random_counter = 0

        self.optimizer = BayesianOptimization(
            f=target_function,
            pbounds=pbounds,
            random_state=1,
            )


    def First_search(self):
        probe_params = {f'x{i}': val for i, val in enumerate(initial_values)}# pre-set
        self.optimizer.probe(
            params=probe_params,
            lazy=True  
        )
        self.optimizer.maximize(init_points=0, n_iter=0)
    def self_test(self):
        current_target = self.optimizer.max['target']
        if abs(current_target) <1:
            self.converged_flag = 1
        if self.best_target is None or current_target > self.best_target:
            self.best_target = current_target
            self.no_improve_count = 0
        else:
            self.no_improve_count += 1

        if self.no_improve_count >= self.no_improve_threshold:
            if (self.bound<0.015 or self.optimize_times >= self.total_iterations):
                print('Bound: ', self.bound, '!!!!!!!!!!!!!')
                self.converged_flag = 1
            self.no_improve_count = 0
            best_params = self.optimizer.max['params']
            self.bound = self.bound*self.rate
            pbounds = {f'x{i}': (val - self.bound * abs(val), val + self.bound * abs(val)) for i, val in best_params.items()}
            self.optimizer.set_bounds(new_bounds=pbounds)
    
    def optimize_once(self):
        if self.first_flag == 0:
            self.First_search()
            self.first_flag = 1
        else:
            if self.converged_flag == 0:
                if (self.random_counter < self.random_times):
                    self.optimizer.maximize(init_points=1, n_iter=0)
                    self.random_counter += 1
                else:
                    self.optimizer.maximize(init_points=0, n_iter=1)
                    self.self_test()
                    self.optimize_times += 1

    def save_params(self):
        filepath_bo = current_dir + '/bo_res.pickle'
        with open(filepath_bo,'wb') as f:
            pickle.dump(self.optimizer.res, f)
            print('Optimization Result Saved!!!!')


my_bo = My_BO()

def bayesian_optimization_thread():
    global qd_flag
    running = True  
    while running and not rospy.is_shutdown():
        if qd_flag == 0:
            my_bo.optimize_once()
            qd_flag = 1
            my_bo.finish_time = time.time()
            save_flag = True
        if my_bo.optimize_times%10 == 0 and save_flag == True:
            my_bo.save_params()
            save_flag = False
        if my_bo.converged_flag:
            my_bo.save_params()
            running = False
            print('STOP!!!!!!!!!!!!!')
        rospy.sleep(0.01)

def listener():

    rospy.Subscriber("anomaly_topic", Float32, callback_float, queue_size=1)

    rospy.Subscriber("limb_topic", limb, callback_limb, queue_size=1)

def qd_update_thread(pub_qd):
    global qd_flag, Start_Ready_flag, init_wait
    rate = rospy.Rate(1000)
    msg = limb()
    flag_firstin = True
    while not rospy.is_shutdown():
        t = time.time() 
        if (flag_firstin):
            t_firstin = time.time()
            flag_firstin = False
        if(t - t_firstin >15) and init_wait and cost_bo.counter > 10:
            cost_bo.receive_flag = 0
            init_wait = False
            Start_Ready_flag = True
            cost_bo.init_cost()
        joint0.update(weight_q2,t)
        joint1.update(weight_q4,t)
        cost_bo.q1_d = joint0.output
        cost_bo.q2_d = joint1.output
        cost_bo.check_counter()
        cost_bo.q1_d_pre = cost_bo.q1_d
        msg.q2 = joint0.output
        msg.q4 = joint1.output
        if (my_bo.converged_flag == 0):
            pub_qd.publish(msg)
            pass
        # t_end = time.time()
        rate.sleep() 

        if qd_flag == 1 and cost_bo.counter >= cost_bo.threshold + 1:
            print(cost_bo.cost_counter)
            cost_bo.init_cost()
            qd_flag = 0

def init_ros():
    pub_qd = rospy.Publisher("qd_topic", limb, queue_size=1)  # 假设的消息类型和主题
    return pub_qd

if __name__ == '__main__':
    rospy.init_node('bayes_op', anonymous=True)
    pub_qd = init_ros()
    listener_thread = threading.Thread(target=listener)
    bo_thread = threading.Thread(target=bayesian_optimization_thread)
    qd_thread = threading.Thread(target=qd_update_thread, args=(pub_qd,))

    listener_thread.start()
    bo_thread.start()
    qd_thread.start()

    listener_thread.join()
    bo_thread.join()
    qd_thread.join()
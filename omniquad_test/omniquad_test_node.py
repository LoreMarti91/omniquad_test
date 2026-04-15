from typing import List
import rclpy # serve per il nodo
from rclpy.node import Node # adesso posso usare calsse Nodo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # Importa il gruppo di callback mutuamente esclusivo
from pi3hat_moteus_int_msgs.msg import JointsCommand,JointsStates,PacketPass,OmniMulinexCommand,DistributorsState,Counter
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
import numpy as np
import os
#import pandas as pd
from scipy.interpolate import interp1d
from datetime import datetime
import math
from omniquad_test.linear_traj import Trajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration



class OmniquadTest(Node):
    def __init__(self,
                node_name = "omniquad_test_node", # nome del nodo
                period = 0.45e-3, ## temporizzazione
                             
                # jnt_names = ['LF_WHEEL_JNT','RF_WHEEL_JNT','RH_WHEEL_JNT','LH_WHEEL_JNT'], # QUELLI DELLA CONFIG 
                test_name = 'prova_jump', 
                # n_joint = 4, # numero di giunti
                try_test = 3,   # [salti da eseguire]
                try_start = 1,
               
                height_rate = 0.0,
                height_rate_vel = 0.5,
                
                Vel_start = 0.0,
                Vel_linear = 0.5,
                Vel_angolar = 1,
                   
                t_pre_test = 10,   #[s] tempo prima di inizio test
                t_acc = 1,   # [s] Durata fase di accelerazione
                t_holding_vel = 2,   # [s] mantenimento velocità
                t_dec = 1,   # [s] Durata fase di decelerazione
                ):
                
                
        # pakage_path = '/home/lorenzo/Desktop/LORENZO_WS/softlegjump_ws/src/fatigue_test/' 
        super().__init__(node_name) #super chiama il costruttore della calsse Node e dà il nome ( node_name = omniquad_test_node)
        
        ### TEMPI FASE HOMING_CALLBACK ( AL TERMINE AZZEO COUNTER E QUINDI ANCHE DELTA TEMPO)
        self.t_pre_test = t_pre_test 
        self.t_acc = t_acc 
        self.t_holding_vel = self.t_acc + t_holding_vel
        self.t_dec = self.t_holding_vel + t_dec 

        # create publisher with timer 
        # self.pub = self.create_publisher(JointsCommand,                     # il nodo pubblica messaggi di tipo JointsCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
        #                                 "joint_controller/command",
        #                                 10)
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos.deadline = Duration(seconds=0, nanoseconds=3000000)  # 3 ms
        # self.pub_command_vel = self.create_publisher(OmniMulinexCommand, "omni_controller/command", qos)
        # qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )

        # qos.deadline = Duration(seconds=0, nanoseconds=3000000)  
        # # esempio: 2.5 ms = 400 Hz
        self.pub_command_vel = self.create_publisher(OmniMulinexCommand,                     # il nodo pubblica messaggi di tipo OmniMulinexCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
                                        "omni_controller/command",
                                        qos)
        
        
        self.pub_count = self.create_publisher(Counter,                     # il nodo pubblica messaggi di tipo JointsCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
                                       "Counter",
                                       10)
        

        self.period = period
        self.counter_try_test = try_start 
        # self.jnt_names = jnt_names
        # self.n_joint = n_joint
        self.Vel_start = Vel_start
        self.Vel_angolar = Vel_angolar
        self.Vel_linear = Vel_linear
        self.height_rate = height_rate
        self.height_rate_vel= height_rate_vel
        self.counter_move = 0    
        self.k_0 = 0.2
        self.k_1 = 1.0 
        self.v_x = [self.Vel_start,self.Vel_start]
        self.v_y = [self.Vel_start,self.Vel_start]
        self.omega = [self.Vel_start,self.Vel_start]
        self.v_height_rate = [self.height_rate_vel,self.height_rate_vel]
        self.v_kp_scale_leg = [self.k_1,self.k_1]
        self.v_kd_scale_leg = [self.k_1,self.k_1]
        self.v_x_pub = 0
        self.v_y_pub  = 0
        self.omega_pub  = 0
        self.height_rate_pub = 0
        self.kp_scale_leg_pub = 0
        self.kd_scale_leg_pub = 0
        self.kp_scale_wheel = 1.0
        self.kd_scale_wheel = 0.95
        self.state_moviment = 0
        self.phase_test = 0
          
        self.clock = self.get_clock()
        
      
        self.try_test = try_test
        
        
    
        self.traj_linear_ik = Trajectory() # Creiamo un'istanza della classe Trajectory

        
       
        
       
        self.start_node = self.time_to_s(self.clock.now(), 0.0)
       
        
        self.timer = self.create_timer(self.period, self.homing_callback)
        
        ######################## la callback crea messaggi e li pubblica
 
        
    def homing_callback(self):  
        self.t = self.time_to_s(self.clock.now(),self.start_node)    
        
        ### PRE TEST
        if(self.t < self.t_pre_test) and (self.state_moviment >= 0):
            
            self.phase_test = 0
            self.updates_velocities()        
            self.publish_msg(height_rate_init = self.v_height_rate[0],height_rate_fin = self.v_height_rate[1],
                             kp_scale_leg_init = self.v_kp_scale_leg[0],kp_scale_leg_fin = self.v_kp_scale_leg[1],
                             kd_scale_leg_init = self.v_kd_scale_leg[0],kd_scale_leg_fin = self.v_kd_scale_leg[1],
                             v_x_init = self.v_x[0],v_x_fin = self.v_x[1],
                             v_y_init = self.v_y[0],v_y_fin = self.v_y[1],
                             omega_init = self.omega[0],omega_fin = self.omega[1],
                             time_ini = 0.0,time_fin=self.t_pre_test,phase_test = self.phase_test,phase_moviment = self.state_moviment)

        else:     
            self.start_node = self.time_to_s(self.clock.now(), 0.0)  
            self.phase_test = self.phase_test + 1
            self.counter_move = 1
            self.timer.destroy()
            print("[INFO]\tend pretest") 
                    
            self.timer = self.create_timer(self.period, self.timer_callback_iteration) ## 

    def timer_callback_single_move(self):
        self.t = self.time_to_s(self.clock.now(),self.start_node)
                       
        #### PHASE 1 - V* Move forward acc
        if(self.t < self.t_acc): 
             self.phase_test = 1
             self.updates_velocities()
             self.publish_msg(height_rate_init = self.v_height_rate[0],height_rate_fin = self.v_height_rate[1],
                             kp_scale_leg_init = self.v_kp_scale_leg[0],kp_scale_leg_fin = self.v_kp_scale_leg[1],
                             kd_scale_leg_init = self.v_kd_scale_leg[0],kd_scale_leg_fin = self.v_kd_scale_leg[1],
                             v_x_init = self.v_x[0],v_x_fin = self.v_x[1],
                             v_y_init = self.v_y[0],v_y_fin = self.v_y[1],
                             omega_init = self.omega[0],omega_fin = self.omega[1],
                             time_ini = 0.0,time_fin=self.t_acc,phase_test = self.phase_test,phase_moviment=self.state_moviment)

            
         ### PHASE 2 - V* Move forward stay
        elif (self.t >= self.t_acc) and (self.t < self.t_holding_vel): 
            self.phase_test = 2
            self.updates_velocities()
            self.publish_msg(height_rate_init = self.v_height_rate[0],height_rate_fin = self.v_height_rate[1],
                             kp_scale_leg_init = self.v_kp_scale_leg[0],kp_scale_leg_fin = self.v_kp_scale_leg[1],
                             kd_scale_leg_init = self.v_kd_scale_leg[0],kd_scale_leg_fin = self.v_kd_scale_leg[1],
                             v_x_init = self.v_x[0],v_x_fin = self.v_x[1],
                             v_y_init = self.v_y[0],v_y_fin = self.v_y[1],
                             omega_init = self.omega[0],omega_fin = self.omega[1],
                             time_ini = self.t_acc,time_fin=self.t_holding_vel,phase_test = self.phase_test,phase_moviment=self.state_moviment)



         ## PHASE 3 - V* Move forward dec
        elif (self.t >= self.t_holding_vel) and (self.t < self.t_dec):
            self.phase_test = 3
            self.updates_velocities()
            self.publish_msg(height_rate_init = self.v_height_rate[0],height_rate_fin = self.v_height_rate[1],
                             kp_scale_leg_init = self.v_kp_scale_leg[0],kp_scale_leg_fin = self.v_kp_scale_leg[1],
                             kd_scale_leg_init = self.v_kd_scale_leg[0],kd_scale_leg_fin = self.v_kd_scale_leg[1],
                             v_x_init = self.v_x[0],v_x_fin = self.v_x[1],
                             v_y_init = self.v_y[0],v_y_fin = self.v_y[1],
                             omega_init = self.omega[0],omega_fin = self.omega[1],
                             time_ini = self.t_holding_vel,time_fin=self.t_dec,phase_test = self.phase_test,phase_moviment=self.state_moviment)

  
    
        else:
                      
            self.counter_move = self.counter_move + 1
            self.phase_test = 1
            
            self.start_node = self.time_to_s(self.clock.now(), 0.0)            
            #print(self.counter_move)
            print("[INFO]\tend ::::::: move")

    def timer_callback_moviment(self):

    
        if(self.counter_move > 0) and (self.counter_move <= 1):
            self.state_moviment = 1
            self.timer_callback_single_move()

        elif(self.counter_move > 1) and (self.counter_move <= 2):
            self.state_moviment = 2
            self.timer_callback_single_move()

        elif(self.counter_move > 2) and (self.counter_move <= 3):
            self.state_moviment = 3
            self.timer_callback_single_move()

        elif(self.counter_move > 3) and (self.counter_move <= 4):
            self.state_moviment = 4
            self.timer_callback_single_move()
        
        elif(self.counter_move > 4) and (self.counter_move <= 5):
            self.state_moviment = 5
            self.timer_callback_single_move()
        

        elif(self.counter_move > 5) and (self.counter_move <= 6):
            self.state_moviment = 6
            self.timer_callback_single_move()
       
        else:
            
            self.counter_try_test = self.counter_try_test + 1
            self.counter_move = 1 
            
            #self.start_node = self.time_to_s(self.clock.now(), 0.0)   
            #self.timer.destroy()
            #self.timer = self.create_timer(self.period, self.timer_callback_iteration)
            print(self.counter_try_test)
            print("[INFO]\tend ::::::: moviment")
            
            # if self.counter_try_test > self.try_test:
            #     self.timer.destroy()
            #     print('[INFO]: test completato')    
            #     rclpy.shutdown()  # termina pulitamente
            # else:
            #     # Qui resetti phase_test e lascia il timer attivo
            #     self.phase_test = 1
            #     # Non serve chiamare self.timer_callback_iteration() manualmente,
            #     # il timer periodico continuerà a chiamare questa funzione
    def timer_callback_iteration(self):

               
        if(self.counter_try_test <= self.try_test):
            
            self.timer_callback_moviment()

       
        else:
            self.timer.destroy()
            print('[INFO]:\tend test PDDD::::::: die')
            raise Exception() # 
        
          
  
    def publish_msg(self,height_rate_init,height_rate_fin,kp_scale_leg_init,kp_scale_leg_fin,kd_scale_leg_init,kd_scale_leg_fin,
                    v_x_init,v_x_fin,v_y_init,v_y_fin,omega_init,omega_fin,time_ini,time_fin,phase_test,phase_moviment): 
        msg = OmniMulinexCommand()
        msg_count = Counter()
        

        # self.phase_test = phase_test   
        # self.v_x = self.traj_linear_ik.linear_trajectory(pos_init = v_x_init,pos_fin =v_x_fin, time_ini = time_ini,time_fin = time_fin,time = self.t)
        # self.v_y = self.traj_linear_ik.linear_trajectory(pos_init = v_y_init,pos_fin =v_y_fin, time_ini = time_ini,time_fin = time_fin,time = self.t)
        # self.omega = self.traj_linear_ik.linear_trajectory(pos_init = omega_init,pos_fin = omega_fin, time_ini = time_ini,time_fin = time_fin,time = self.t)
         
        self.v_x_pub = self.traj_linear_ik.linear_trajectory(
            pos_init=v_x_init,
            pos_fin=v_x_fin,
            time_ini=time_ini,
            time_fin=time_fin,
            time=self.t
        )

        self.v_y_pub = self.traj_linear_ik.linear_trajectory(
            pos_init=v_y_init,
            pos_fin=v_y_fin,
            time_ini=time_ini,
            time_fin=time_fin,
            time=self.t
        )

        self.omega_pub = self.traj_linear_ik.linear_trajectory(
            pos_init=omega_init,
            pos_fin=omega_fin,
            time_ini=time_ini,
            time_fin=time_fin,
            time=self.t
        )
        self.height_rate_pub = self.traj_linear_ik.linear_trajectory(
            pos_init=height_rate_init,
            pos_fin=height_rate_fin,
            time_ini=time_ini,
            time_fin=time_fin,
            time=self.t
        )

        self.kp_scale_leg_pub = self.traj_linear_ik.linear_trajectory(
            pos_init=kp_scale_leg_init,
            pos_fin=kp_scale_leg_fin,
            time_ini=time_ini,
            time_fin=time_fin,
            time=self.t
        )

        self.kd_scale_leg_pub = self.traj_linear_ik.linear_trajectory(
            pos_init=kd_scale_leg_init,
            pos_fin=kd_scale_leg_fin,
            time_ini=time_ini,
            time_fin=time_fin,
            time=self.t
        )


        if not (math.isnan(self.v_x_pub) and math.isnan(self.v_y_pub) and math.isnan(self.omega_pub) and math.isnan(self.height_rate_pub) and math.isnan(self.kp_scale_leg_pub) and math.isnan(self.kd_scale_leg_pub)):

            msg.header.stamp = self.clock.now().to_msg()

            msg.v_x = float(self.v_x_pub)
            msg.v_y = float(self.v_y_pub)
            msg.omega = float(self.omega_pub)
            msg.height_rate = float (self.height_rate_pub)
            msg.kp_scale_leg = float (self.kp_scale_leg_pub)
            msg.kd_scale_leg = float (self.kd_scale_leg_pub)
            msg.kp_scale_wheel = float (self.kp_scale_wheel)
            msg.kd_scale_wheel = float (self.kd_scale_wheel)


            self.pub_command_vel.publish(msg)
        else:
            print("C'è un NaN nelle velocità")
        # if not (
        #     math.isnan(self.v_x) or 
        #     math.isnan(self.v_y) or 
        #     math.isnan(self.omega)
        # ):

        #     msg.header.stamp = self.clock.now().to_msg()

        #     msg.v_x.append(self.v_x)
        #     msg.v_y.append(self.v_y)
        #     msg.omega.append(self.omega)
        #     msg.height_rate.append(height_rate)

        #     self.pub_command_vel.publish(msg)

        # else:
        #     print("C'è un NaN nelle velocità")


        # if not any(math.isnan(x) for x in self.q_pos_IK):
            
        #         msg.header.stamp = self.clock.now().to_msg()  
        #         msg.v_x.append(self.v_x)
        #         msg.v_y.append(self.v_y)
        #         msg.omega.append(self.omega)
        #         msg.height_rate.append(height_rate)
               

                    
    
        #     self.pub.publish(msg)

        # else:
        #     print('C è un Nan PD')


       
        msg_count.header.stamp = self.clock.now().to_msg()
        msg_count.counter_try = [self.counter_try_test]
        msg_count.phase_test = [self.phase_test]
        msg_count.state_moviment = [self.state_moviment]
        
            
        self.pub_count.publish(msg_count)

  

    def time_to_s(self, time, start):
        [sec, ns] = time.seconds_nanoseconds()
        now = float(sec + ns/pow(10, 9))
        return (now - start)
  
 
    def updates_velocities(self):

        if (self.state_moviment == 0) and (self.phase_test == 0):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]


            
        elif (self.state_moviment == 1) and (self.phase_test == 1):
            self.v_x = [self.Vel_start,self.Vel_linear]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate_vel,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]            
        elif (self.state_moviment == 1) and (self.phase_test == 2):
            self.v_x = [self.Vel_linear,self.Vel_linear]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate_vel,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        elif (self.state_moviment == 1) and (self.phase_test == 3):
            self.v_x = [self.Vel_linear,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate_vel,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]


        elif (self.state_moviment ==2) and (self.phase_test == 1):
            self.v_x = [self.Vel_start,-self.Vel_linear]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,-self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        elif (self.state_moviment == 2) and (self.phase_test == 2):
            self.v_x = [-self.Vel_linear,-self.Vel_linear]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [-self.height_rate_vel,-self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        elif (self.state_moviment == 2) and (self.phase_test == 3):
            self.v_x = [-self.Vel_linear,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]


        elif (self.state_moviment == 3) and (self.phase_test == 1):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_linear]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]
        elif (self.state_moviment == 3) and (self.phase_test == 2):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_linear,self.Vel_linear]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]
        elif (self.state_moviment == 3) and (self.phase_test == 3):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_linear,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]


        elif (self.state_moviment == 4) and (self.phase_test == 1):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,-self.Vel_linear]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]
        elif (self.state_moviment == 4) and (self.phase_test == 2):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [-self.Vel_linear,-self.Vel_linear]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]
        elif (self.state_moviment == 4) and (self.phase_test ==3):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [-self.Vel_linear,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_0,self.k_0]
            self.v_kd_scale_leg = [self.k_0,self.k_0]


        elif (self.state_moviment == 5) and (self.phase_test == 1):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,self.Vel_angolar]
            self.v_height_rate = [self.height_rate,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_0,self.k_1]
            self.v_kd_scale_leg = [self.k_0,self.k_1]
        elif (self.state_moviment == 5) and (self.phase_test == 2):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_angolar,self.Vel_angolar]
            self.v_height_rate = [self.height_rate_vel,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        elif (self.state_moviment == 5) and (self.phase_test == 3):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_angolar,self.Vel_start]
            self.v_height_rate = [self.height_rate_vel, self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]


        elif (self.state_moviment == 6) and (self.phase_test == 1):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [self.Vel_start,-self.Vel_angolar]
            self.v_height_rate = [self.v_height_rate,self.height_rate_vel]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        elif (self.state_moviment == 6) and (self.phase_test == 2):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [-self.Vel_angolar,-self.Vel_angolar]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        elif (self.state_moviment == 6) and (self.phase_test == 3):
            self.v_x = [self.Vel_start,self.Vel_start]
            self.v_y = [self.Vel_start,self.Vel_start]
            self.omega = [-self.Vel_angolar,self.Vel_start]
            self.v_height_rate = [self.height_rate,self.height_rate]
            self.v_kp_scale_leg = [self.k_1,self.k_1]
            self.v_kd_scale_leg = [self.k_1,self.k_1]
        
        else :
            pass




    # def callback_state(self, msg):


         

    #      for i, joint_name in enumerate(msg.name):
    #          position = msg.position[i]
    #          sec_enc_pos = msg.sec_enc_pos[i] if i < len(msg.sec_enc_pos) else None

    #          # Controllo per position
    #          if math.isnan(position):
    #              #print(f"Attenzione: Valore NaN per la posizione del giunto {joint_name}")
    #              # Non aggiornare il campo se è NaN
    #              if joint_name in self.joint_data:
    #                  position = self.joint_data[joint_name].get("position")

    #          # Controllo per second encoder position
    #          if sec_enc_pos is not None and math.isnan(sec_enc_pos):
    #              #print(f"Attenzione: Valore NaN per la posizione del secondo encoder del giunto {joint_name}")
    #              # Non aggiornare il campo se è NaN
    #              if joint_name in self.joint_data:
    #                  sec_enc_pos = self.joint_data[joint_name].get("sec_enc_pos")

    #          # Aggiorna il dizionario solo se i valori non sono NaN
    #          if not math.isnan(position):
    #              if joint_name not in self.joint_data:
    #                  self.joint_data[joint_name] = {}
    #              self.joint_data[joint_name]["position"] = position

    #          if sec_enc_pos is not None and not math.isnan(sec_enc_pos):
    #              if joint_name not in self.joint_data:
    #                  self.joint_data[joint_name] = {}
    #              self.joint_data[joint_name]["sec_enc_pos"] = sec_enc_pos

    #          #print(self.joint_data)

    #      # Gestione specifica di KNEE e HIP
    #      if "KNEE" in self.joint_data:
    #          self.knee_position = self.joint_data["KNEE"]["position"]
    #          self.knee_sec_enc_pos = self.joint_data["KNEE"]["sec_enc_pos"]

    #      if "HIP" in self.joint_data:
    #          self.hip_position = self.joint_data["HIP"]["position"]
    #          self.hip_sec_enc_pos = self.joint_data["HIP"]["sec_enc_pos"]
         




#######################################################################################################
#######################################################################################################

## ToDo: save node_name using param from launch file 


########### DEFINIZIONE DELLA FUNZIONE main
def main(args=None):
    rclpy.init(args=args)  # Si inizializza la libreria rlcpy
    name_exp = "omniquad_test_" + datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    node = OmniquadTest( test_name=name_exp,period=1/450)    # VIENE CREATO IL NODO
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    
    try:
        ex.spin()   #esegue in loop quello che è nel nodo
        print("pass here")
    except Exception as e:  # Gestisce qualsiasi eccezione
        print(f"An error occurred: {e}") # quando viene chiamata esegue questo
        print("Closing ROS2 Node")
        ex.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ =="__main__":
    main()
            
    
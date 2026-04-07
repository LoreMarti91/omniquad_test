
import math
import numpy as np
import matplotlib.pyplot as plt
class Trajectory :
    def __init__(self):
                 self.last_valid_qf_IK = None
                 self.q_history = []
                 
        
    

    def Direct_Kinematic (self,q,l,offset):
        # print(l)
        # print(type(l))
        # print(type(q)) 
        # print(q)
        # print(type(offset)) 
        # print(offset)
        Pos_EE = [
                  l[0] * math.cos(q[0] + offset[0]) + l[1] * math.cos(q[0] + offset[0] + q[1]  + offset[1]), 
                  l[0] * math.sin(q[0] + offset[0]) + l[1] * math.sin(q[0] + offset[0] + q[1] + offset[1])
                  ]
        #print(Pos_EE)
        
        return Pos_EE
  


    def Inverse_Kinematic (self,p,l,k_quadrante,offset):
        r = math.sqrt((p[0]**2 + p[1]**2 ))
        
        cos_q_knee = (r**2 - l[0]**2 - l[1]**2) / (2 * l[0] * l[1])

        cos_q_knee = max(min(cos_q_knee, 1), -1)
        # if cos_q_knee > 1:
        #     print(f"Attenzione: coseno limitato da {cos_q_knee} a 1")
        #     cos_q_knee = 1
        # elif cos_q_knee < -1:
        #     print(f"Attenzione: coseno limitato da {cos_q_knee} a -1")
        #     cos_q_knee = -1


        sin_q_knee = k_quadrante*math.sqrt(1-cos_q_knee**2)

        #q_knee = math.acos(cos_q_knee)
        q_knee = math.atan2(sin_q_knee,cos_q_knee)
        # print('q_knee')
        # print (q_knee)
        
       

        k1 = l[0] + l[1] * math.cos(q_knee)
        k2 = l[1] * math.sin(q_knee)

        q_hip = math.atan2(p[1], p[0]) - math.atan2(k2, k1)
        # print('q_hip')
        # print (q_hip)
      
            
        qf_IK = [q_hip - offset[0] , q_knee - offset[1] ]
        # if (q_hip + offset[0] > 6.28):
        #     qf_IK = [0.0     , q_knee + offset[1] ]

        # else:
        #     qf_IK = [q_hip + offset[0] , q_knee + offset[1] ]

        
        # print('qf_IK')
        # print(qf_IK)

        # Check sui NaN ed eventuale rimozione
        if any(math.isnan(x) for x in qf_IK):
            return self.last_valid_qf_IK

        self.last_valid_qf_IK = qf_IK
        
        return qf_IK

    def linear_trajectory (self,pos_init,pos_fin,time_ini,time_fin,time):
        pos_init = np.array(pos_init)
        pos_fin = np.array(pos_fin)


        m = (pos_fin - pos_init) / (time_fin - time_ini)
        trajectory_linear = m * (time - time_ini) + pos_init
        return trajectory_linear
 


    def Trajectory_linear_IK (self,pos_init,pos_fin,time_fin,time,l,offset):
        pos_init = np.array(pos_init)
        pos_fin = np.array(pos_fin)
        

    # def Trajectory_linear_IK (self,pos_init,pos_fin,time_init,time_fin,time,l,offset):
    #     pos_init = np.array(pos_init)
    #     pos_fin = np.array(pos_fin)


        # m = (pos_fin - pos_init) / (time_fin - time_init)
        # trajectory_linear = m * (time - time_init) + pos_init
        # print('Trajectory',trajectory_linear)

        m = (pos_fin - pos_init) / (time_fin - time)
        trajectory_linear = m * time + pos_init
        # print('pos_init', pos_init)
        # print('pos_fin', pos_fin)
        # print('time_fin', time_fin)
        # print('time', time)
        # print('m', m)
        # print('Trajectory',trajectory_linear)

        def determina_quadrante(trajectory_linear):
            if trajectory_linear[0] <= 0 and trajectory_linear[1] <= 0:
                return 1  # Quadrante III
            elif trajectory_linear[0] <= 0 and trajectory_linear[1] > 0:
                return 1  # Quadrante II
            elif trajectory_linear[0] > 0 and trajectory_linear[1] <= 0:
                return -1  # Quadrante IV
            else:
                return -1  # Quadrante I

        k_quadrante = determina_quadrante(trajectory_linear)

        return self.Inverse_Kinematic(trajectory_linear,l,k_quadrante,offset)


    def Trajectory_parabolic (self,pos_init,pos_fin,time_ini,time_fin,time):
        # pos_init = np.array([0.37,0.0])
        pos_init = np.array(pos_init)
        pos_fin = np.array(pos_fin)
        # print('pos_init =', pos_init)
        # print('pos_fin =',pos_fin)
        c = pos_init
        b = c*0 
        a = (pos_fin - pos_init) / (time_fin-time_ini)**2
        trajectory_parabolic = a *(time - time_ini)**2 + b *(time - time_ini) + c
       
        return trajectory_parabolic

    def Trajectory_parabolic_IK (self,pos_init,pos_fin,time_ini,time_fin,time,l,offset,k_quadrante):
        

        trajectory = self.Trajectory_parabolic(pos_init,pos_fin,time_ini,time_fin,time)
        

        # def determina_quadrante(trajectory_parabolic):
        #     if trajectory_parabolic[0] <= 0 and trajectory_parabolic[1] <= 0:
        #         return 1  # Quadrante III
        #     elif trajectory_parabolic[0] <= 0 and trajectory_parabolic[1] > 0:
        #         return 1  # Quadrante II
        #     elif trajectory_parabolic[0] > 0 and trajectory_parabolic[1] <= 0:
        #         return -1  # Quadrante IV
        #     else:
        #         return -1  # Quadrante I

        # k_quadrante = determina_quadrante(trajectory_parabolic)

        return self.Inverse_Kinematic(trajectory,l,k_quadrante,offset)
        
        # # Dopo il calcolo di trajectory_parabolic nel tuo codice:
        # plt.figure(figsize=(10, 6))
        # plt.plot(time, trajectory_parabolic[0], label='X coordinate')
        # plt.plot(time, trajectory_parabolic[1], label='Y coordinate')
        # plt.xlabel('Time')
        # plt.ylabel('Position')
        # plt.title('Parabolic Trajectory vs Time')
        # plt.legend()
        # plt.grid(True)
        # plt.show()

    def Trajectory_sinusoidal (self,f,A,time):
       

        omega = 2 * math.pi * f
        Sin_Traj = A * math.sin (omega * time)
        
        # Check sui NaN ed eventuale rimozione
        if math.isnan(Sin_Traj):
            
            return self.last_valid_qf_IK

        self.last_valid_qf_IK = Sin_Traj
        
        
        return Sin_Traj 
    



    def Trajectory_sinusoidal_dot (self,f,A,time):
       

        omega = 2 * math.pi * f
        Cos_Traj = A *omega * math.cos (omega * time)

        # Check sui NaN ed eventuale rimozione
        if math.isnan(Cos_Traj):
            
            return self.last_valid_qf_IK

        self.last_valid_qf_IK = Cos_Traj
        
        
        return Cos_Traj
    
    def determina_quadrante(self,EE_pos_now):
        EE_pos_now = np.array(EE_pos_now)

        if EE_pos_now[0] <= 0 and EE_pos_now[1] <= 0:
            k_quadrante = 1  # Quadrante III
        elif EE_pos_now[0] <= 0 and EE_pos_now[1] > 0:
            k_quadrante = 1  # Quadrante II
        elif EE_pos_now[0] > 0 and EE_pos_now[1] <= 0:
            k_quadrante = -1  # Quadrante IV
        else:
            k_quadrante = -1  # Quadrante I


        return k_quadrante 

    def update_q(self,pos):
       

        if len(self.q_history) <2:
            self.q_history.append(pos)
        else:
            self.q_history.pop(0)
            self.q_history.append(pos)
    
        
        
    
    def calcolo_q_dot(self,pos,dt):
        self.update_q(pos)
        if len(self.q_history) == 2:
            q_dot = (np.array(self.q_history[1]) - np.array(self.q_history[0]))/dt
            return q_dot
        else:
            print('error velocity')
            return np.array([0.0,0.0])
            


if __name__ == "__main__":
    # Creiamo un'istanza della classe Trajectory
    robot = Trajectory()

    # Test Cinematica Diretta
    # Definiamo alcuni angoli dei giunti
    # q_test = [0.7811,1.295]
    q_test = [0.0,0.0]
    l = [0.19,0.19]
    offset = [math.pi,0.0]
    q_home_1 = [-0.7,2.84],    # [rad] Posizione di homing con hip verticale per jump 1
               
    q_home_2 = [3.84,-2.84]   # [rad] Posizione di homing con hip vertical per jump 2
              
    q_step_jump_1 = [2, -0.7],    # [rad] posizione finale di jump 1 e di atterraggio
    q_step_jump_2 = [1.14,0.7],    # [rad] posizione finale di jump 1 e di atterraggio
    k_quadrante = +1
    robot.q = q_test  # Impostiamo gli angoli
    robot.l = l 
    robot.k_quadrante = k_quadrante
    robot.offset = offset
    # Calcoliamo la posizione dell'end effector
    posizione_ee = robot.Direct_Kinematic(robot.q, robot.l, robot.offset)
    print("Angoli dei giunti:", q_test)
    print("Posizione End Effector:", posizione_ee)

    # Test Cinematica Inversa
    # Usiamo la posizione calcolata per verificare la cinematica inversa
    # NB se in quadante I (homing 2), k_quadrante = -1; se in quadrante II (homing 1) k_quadrante = +1
    # robot.p = [-0.0429, -0.30]  # Impostiamo la posizione come target questa è hmoming 2
    # #robot.p = [-0.042917340198145665, -0.03764142143580142]  # Impostiamo la posizione come target questa è hmoming 1
    # #robot.p = posizione_ee
    # angoli_giunti = robot.Inverse_Kinematic(robot.p,robot.l,robot.k_quadrante,robot.offset)
    # print("\nPosizione target:", robot.p)
    # print("Angoli dei giunti calcolati:", angoli_giunti)

    # # Verifica
    # # Ricalcoliamo la posizione con gli angoli trovati dalla cinematica inversa
    # robot.q0 = angoli_giunti
    # pos_verifica = robot.Direct_Kinematic(robot.q0, robot.l)
    # print("\nPosizione di verifica:", pos_verifica)
    durata = 5  # secondi
    frequenza_campionamento = 500  # Hz

# Crea il vettore da 0 a 5 secondi con passo 1/500 (frequenza di campionamento)
    time = np.arange(0, durata, 1 / frequenza_campionamento)
    print("time",time)
    pos_ini = posizione_ee
    pos_fin = [-0.043,-0.038]
    robot.time = time
    robot.pos_ini = pos_ini
    robot.pos_fin = pos_fin
    
    j = [1,2]
    joint_position = robot.Trajectory_linear_IK(
                    pos_init=robot.pos_ini,     # Definisci questo
                    pos_fin= robot.pos_ini,       # Definisci questo
                    time_init=0,      # Tempo iniziale
                    time_fin=5,     # Tempo finale
                    time=time,                # Tempo corrente
                    l=robot.l,                 # Lunghezze dei link
                    offset = robot.offset
          
                ) 



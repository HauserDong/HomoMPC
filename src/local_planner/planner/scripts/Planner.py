#!/usr/bin/env python
import rospy
from bring_up.msg import CommuTraj, ExecuteTraj, RefPath, PathPoint
from bring_up.srv import ReplanningCheck, ReplanningCheckRequest
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time
import copy
import numpy as np
import os
import SET
import matplotlib.pyplot as plt
from geometry import *
from math import exp
import concurrent.futures
import sys

class Neighbour():

    def __init__(self,index,j_index):

        self.state=0
        
        # own index
        self.index=index

        # the neighbour's index
        self.j_index=j_index

        # own sequence number
        self.sequence=0

        # own's beginning time of lastly planned trajectory    
        self.last_time=0

        # last protocol updating time
        self.update_time=time.time()

        # the communication delay with this neighbour
        self.delay=0.0

        self.gamma=0.0001

        # the protocol with this neighbour
        self.protocol=[]

        # the agreement list used for avoid a repeated agreement
        self.agreement_list=[]

        # the communicated trajectory with this neighbour
        self.CommuTraj=CommuTraj()

        # define subscriber and publisher with this neighbour
        self.sub=rospy.Subscriber('CommuTraj'+str(j_index)+'to'+str(self.index), CommuTraj, callback=self.response)
        self.pub=rospy.Publisher('CommuTraj'+str(self.index)+'to'+str(j_index), CommuTraj, queue_size=1)

        # for trajectory estimation
        self.traj_x = None
        self.traj_y = None
        self.passage_time_map = None
        self.traj_start_time = None

        self.traj_state_est = None
        self.traj_time_est = None
        self.h = None

        # communication radius
        self.i_com_radius = None

        self.out_of_range = False
        

    def update_delay(self,delay):

        self.delay=0.7*self.delay+0.3*delay

        print(self.delay)


    def update_protocol(self,OtherTraj):

        traj_start_time=time.time()-self.delay-OtherTraj.r_start
        self.traj_start_time = traj_start_time
        
        start_time=max(self.last_time+self.CommuTraj.validity,traj_start_time+OtherTraj.validity)
        end_time=float('inf')

        if self.protocol!=[]:

            self.protocol[-1]['end_time']=start_time

        shapei=np.array(self.CommuTraj.shape)
        if len(shapei)>1:
            shapei=shapei.reshape(int(len(shapei)/2),2)

        shapej=np.array(OtherTraj.shape)
        if len(shapej)>1:
            shapej=shapej.reshape(int(len(shapej)/2),2)
        
        self.protocol+=[
            {   
                'start_time': start_time,
                'end_time': end_time,
                'j':{
                    'start_time': traj_start_time,
                    'x': OtherTraj.traj_x,
                    'y': OtherTraj.traj_y,
                    'theta': OtherTraj.traj_theta,
                    'duration':OtherTraj.duration,
                    'shape': shapej
                },
                'i':{
                    'start_time': self.last_time,
                    'x': self.CommuTraj.traj_x,
                    'y': self.CommuTraj.traj_y,
                    'theta': self.CommuTraj.traj_theta,
                    'duration': self.CommuTraj.duration,
                    'shape': shapei
                }
            }
        ]

        del_list=[]
        i=0
        for agreement in self.protocol:

            # the very outdated agreenment is deleted
            if self.last_time+self.CommuTraj.validity > agreement['end_time']:
                del_list+=[i]
            
            # delete the overlap agreement
            if i < len(self.protocol)-1:
                if abs(agreement['start_time']-self.protocol[i+1]['start_time'])<1e-2:
                    self.protocol[i+1]['start_time']=agreement['start_time']
                    del_list+=[i]

            i+=1

        del_list.reverse()
        for i in del_list:
            del self.protocol[i]

        self.update_time=time.time()


    def update_gamma(self,gamma):

        # gamma is an important coefficient used for deadlock resolution

        if self.gamma is None:
            self.gamma=gamma 
        else:
            self.gamma+=0.7*self.gamma+0.3*gamma


    def initiate_agreement(self,CommuTraj):
        
        # used for measuring the time delay
        self.initiate_time=time.time()

        self.pub.publish(CommuTraj)

    def transform_path(self, path):
        path_res = []
        for i in range(len(path)):
            point = []
            point.append(path[i].x)
            point.append(path[i].y)
            path_res.append(point)

        if len(path_res) == 0:
            return None
        else:
            return np.array(path_res)
    
    def transform_passage_passing_info(self, ppi):
        ppi_res = []
        idx = 0
        while idx < len(ppi):
            node_idx = ppi[idx]
            passage_idx = ppi[idx+1]
            ppi_res.append([node_idx, passage_idx])
            idx += 2
        
        if len(ppi_res) == 0:
            return None
        else:
            return np.array(ppi_res)

    def response(self,OtherTraj):

        # Only receive the information from agents within the communication range
        time_now = time.time()
        self_start_time = time_now - self.last_time
        other_start_time = time_now - self.delay - OtherTraj.r_start
        self_r_t = time_now - self_start_time
        other_r_t = time_now - other_start_time

        if self_r_t>self.CommuTraj.duration*0.95:
            px=get_sample(self.CommuTraj.traj_x,self.CommuTraj.h,self.CommuTraj.duration*0.95)
            py=get_sample(self.CommuTraj.traj_y,self.CommuTraj.h,self.CommuTraj.duration*0.95)
        else:
            px=get_sample(self.CommuTraj.traj_x,self.CommuTraj.h,self_r_t)
            py=get_sample(self.CommuTraj.traj_y,self.CommuTraj.h,self_r_t)
        self_pos=np.array([px,py])

        if other_r_t>OtherTraj.duration*0.95:
            px=get_sample(OtherTraj.traj_x,OtherTraj.h,OtherTraj.duration*0.95)
            py=get_sample(OtherTraj.traj_y,OtherTraj.h,OtherTraj.duration*0.95)
        else:
            px=get_sample(OtherTraj.traj_x,OtherTraj.h,other_r_t)
            py=get_sample(OtherTraj.traj_y,OtherTraj.h,other_r_t)
        other_pos=np.array([px,py])

        dist = np.linalg.norm(other_pos-self_pos)
        if dist > self.i_com_radius:
            self.out_of_range = True
        else:
            self.out_of_range = False

        if self.state==1 and not self.out_of_range:

            flag=False

            if OtherTraj.initiator==False:

                agreement=str(self.index)+'-'+str(self.sequence)+' & '+str(OtherTraj.index)+'-'+str(OtherTraj.sequence)

                if not (agreement in self.agreement_list):

                    flag=True

                    self.agreement_list+=[agreement]
                    dual_agreement=str(OtherTraj.index)+'-'+str(OtherTraj.sequence)+' & '+str(self.index)+'-'+str(self.sequence)
                    self.agreement_list+=[dual_agreement] 

                    self.update_protocol(OtherTraj)

                    self.update_delay((time.time()-self.initiate_time)/2)

            else:

                agreement=str(OtherTraj.index)+'-'+str(OtherTraj.sequence)+' & '+str(self.index)+'-'+str(self.sequence)

                if not (agreement in self.agreement_list):

                    flag=True

                    self.agreement_list+=[agreement]
                    dual_agreement=str(self.index)+'-'+str(self.sequence)+' & '+str(OtherTraj.index)+'-'+str(OtherTraj.sequence)
                    self.agreement_list+=[dual_agreement]       

                    self.CommuTraj.initiator=False
                    self.CommuTraj.r_start=time.time()-self.last_time

                    self.pub.publish(self.CommuTraj)

                    self.update_protocol(OtherTraj)
            
            self.traj_x = OtherTraj.traj_x
            self.traj_y = OtherTraj.traj_y
            self.h = OtherTraj.duration / (len(OtherTraj.traj_x)-1)

            if flag:     
                print('-------------------------')
                print('Agent'+str(self.index)+' reach agreement: '+agreement)
                print(' ')    


        return None
  

class planner():

    def __init__(self, agent, obs_env_idx):

        self.index=agent['index']

        rospy.init_node('Agent'+str(self.index), anonymous=False)
        self.Traj_pub=rospy.Publisher('Traj'+str(self.index), ExecuteTraj, queue_size=10)
        self.Ref_traj_pub = rospy.Publisher('Ref_Traj'+str(self.index), ExecuteTraj, queue_size=3)
        self.Path_sub=rospy.Subscriber('/Path'+str(self.index), RefPath, callback=self.get_path,queue_size=1)
        # self.nav_goal_sub=rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback=self.get_target,queue_size=10)

        self.Replan_check = rospy.ServiceProxy('replanning_check', ReplanningCheck)

        # debugging
        self.tractive_pub = rospy.Publisher('tractive'+str(self.index), Marker, queue_size=1)

        self.t_w=agent['t_w']
        self.t_c=agent['t_c']
        self.h=agent['h']
        self.K=agent['K']
        self.type=agent['type']
        self.real_target=agent['state']
        self.target=agent['state']
        self.validity=self.t_c+self.t_w
        self.CommuTraj=CommuTraj()
        self.ExecuteTraj=ExecuteTraj()
        self.RefExecuteTraj=ExecuteTraj()
        self.sequence=0
        self.ini_state=agent['state']
        self.shape = agent['radius']
        self.com_radius = agent['com_radius']
        
        traj_x=self.ini_state[0]*np.ones(self.K+1)
        traj_y=self.ini_state[1]*np.ones(self.K+1)
        self.traj=[traj_x,traj_y]

        self.time_list=np.array([0.0])

        self.path_start_time = time.time()
        self.path = None
        self.path_vel = None
        self.path_time = None


        if agent['type']=='Markanem':
            from Dynamic.Omnidirection.Markanem import markanem
            self.dynamic=markanem(self.index,self.shape,self.K,self.h,self.ini_state,self.target,obs_env_idx)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=np.zeros(self.K+1)
        elif agent['type']=='Mini_mec':
            from Dynamic.Omnidirection.Mini_mec import mini_mec
            self.dynamic=mini_mec(self.index,self.shape,self.K,self.h,self.ini_state,self.target,obs_env_idx)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=np.zeros(self.K+1)
        elif agent['type']=='Mini_om':
            from Dynamic.Omnidirection.Mini_om import mini_om
            self.dynamic=mini_om(self.index,self.shape,self.K,self.h,self.ini_state,self.target,obs_env_idx)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=np.zeros(self.K+1)
        elif agent['type']=='Walle':
            from Dynamic.Unicycle.Walle import walle
            self.dynamic=walle(self.index,self.shape, self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Mini_4wd':
            from Dynamic.Unicycle.Mini_4wd import mini_4wd
            self.dynamic=mini_4wd(self.index,self.shape, self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Mini_tank':
            from Dynamic.Unicycle.Mini_tank import mini_tank
            self.dynamic=mini_tank(self.index, self.shape, self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Mini_ack':
            from Dynamic.Car.Mini_ack import mini_ack
            self.dynamic=mini_ack(self.index, self.shape,self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Formula1':
            from Dynamic.Car.Formula1 import formula1
            self.dynamic=formula1(self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Porsche':
            from Dynamic.Car.Porsche import porsche
            self.dynamic=porsche(self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        else:
            raise Exception('Do not have this type, please define or check it')
        
        self.traj+=[traj_theta]
        if self.ini_state.shape[0] > 2:
            self.ref_traj = np.array([[self.ini_state[0],self.ini_state[1],self.ini_state[2]]])
        else:
            self.ref_traj = np.array([[self.ini_state[0],self.ini_state[1]]])

        # the neighbors that requires to communicate with
        self.neighbor_list=[]
        self.neighbor_index=[]

        self.new_target_set = False

        ###############################
        #### self traj estimation #####
        ###############################

        self.initial_time = time.time()

        # self.passage_pairs = self.dynamic.extend_visibility_check_res

        # print passage_pairs
        # for i in range(len(self.passage_pairs)):
        #     passage = self.passage_pairs[i]
        #     print('Passage pair '+str(i)+': '+str(passage[0])+' '+str(passage[1]))
        #     print("Passage pair "+str(i)+" length: ", np.linalg.norm(np.array(passage[0])-np.array(passage[1])))

        # self.self_passage_time_map = [-1 for _ in range(len(self.passage_pairs))]   # record the time arrival at each passage pair, -1 means not arrived

        # update trajectory for commnication and execution
        self.update_traj()

        # publish traj for low-level controler
        self.execute_traj()

        self.traj_state_est = None
        self.traj_time_est = None

        # self.traj_est(1)

        self.V_aver = -1

        # visualize trajectory time stamp estimation
        if self.index == 1:
            self.traj_est_pub_list = []

            self.self_traj_est_pub = rospy.Publisher('Self_Traj_Est'+str(self.index), MarkerArray, queue_size=1)

    def disc_traj(self, traj, disc):
        traj_disc = []
        i = 1
        traj_disc.append(traj[0])
        while True:
            if i >= len(traj):
                break

            if i == len(traj)-1:
                traj_disc.append(traj[i])
                break
            
            last_state = traj_disc[-1]
            dist = np.linalg.norm(traj[i]-last_state)

            if dist >= disc:
                traj_disc.append(traj[i])

            i += 1
        return np.array(traj_disc)

    # scan avaliable neighbour in its own communication range
    def scan(self):

        topics=rospy.get_published_topics()

        for topic in topics:
            name=topic[0]
            if name[0:5]=='/Traj':
                j_index=int(name[5:])
                if not(j_index in self.neighbor_index) and j_index!=self.index:
                    self.neighbor_index+=[j_index]
                    self.neighbor_list+=[Neighbour(self.index,j_index)]
                    # self.neighbor_list[-1].passage_time_map = [-1 for _ in range(len(self.passage_pairs))]
                    self.neighbor_list[-1].i_com_radius = self.com_radius

                    # visualize trajectory time stamp estimation
                    if self.index == 1:
                        self.traj_est_pub_list += [rospy.Publisher('Other_Traj_Est'+str(j_index), MarkerArray, queue_size=1)]

        # delete outdated neighbors
        t=time.time()
        del_list=[]
        for i in range(len(self.neighbor_list)):
            if t-self.neighbor_list[i].update_time>2.0:
                del_list+=[i]
        del_list.reverse()
        for i in del_list:
            del self.neighbor_list[i]
            del self.neighbor_index[i]

    def decode_path(self, path):
        path_return = []

        for point in path:
            path_return.append([point.x, point.y])
        
        path_return = np.array(path_return)

        return path_return

    def get_path(self,msg):

        print("=====================================")
        print('Receive path from global planner')
        print("self.path_start_time: ", self.path_start_time)
        print("msg.start_time: ", msg.start_time)
        print("=====================================")
        
        self.path_start_time = msg.start_time

        self.path_vel = msg.path_vel

        self.path = self.decode_path(msg.path)
        # print(self.path)

        self.path_time = msg.path_time

        self.ref_traj = self.path

    # def get_target(self,msg):

    #     px=msg.pose.position.x
    #     py=msg.pose.position.y
    #     theta=np.arctan2(2*(msg.pose.orientation.w*msg.pose.orientation.z+msg.pose.orientation.x*msg.pose.orientation.y),\
    #     (1-2*(msg.pose.orientation.y*msg.pose.orientation.y+msg.pose.orientation.z*msg.pose.orientation.z)))

    #     if self.type=='Mini_mec' or self.type=='Mini_om' or self.type=='Walle':
    #         self.real_target=np.array([px,py])
    #     else:
    #         self.real_target=np.array([px,py,theta])

    # initiate agreements with others
    def initiate_agreement(self):
        
        self.CommuTraj.initiator=True 
        self.CommuTraj.r_start=time.time()-self.last_time

        for nei in self.neighbor_list:
            nei.initiate_agreement(self.CommuTraj)


    def plan_traj(self):

        from inter_aviodance import get_inter_cons
        from obstacle_avoidance import get_ob_cons

        start=time.time()
        inter_cons=get_inter_cons(self)
        print('Inter-agent avoidance constraints time cost:'+str(time.time()-start)+'s')

        start=time.time()
        ob_cons=get_ob_cons(self)
        print('Obstacle avoidance constraints time cost:'+str(time.time()-start)+'s')
        
        self.dynamic.trajectory_planning(inter_cons,ob_cons)

        self.dynamic.post_processing(self.validity)
        
        # 这个地方还需要改善，因为对邻居的定义还有待改变
        # if self.neighbor_list !=[]:
        #     for nei,gamma in zip(self.neighbor_list,self.dynamic.gamma):
        #         nei.update_gamma(gamma)

        return self.dynamic.traj

    def publish_homo_path(self, self_path):
        pathpoints = []
        if self_path is not None:
            for j in range(len(self_path)):
                pathpoints.append(PathPoint(x=self_path[j][0], y=self_path[j][1]))

        return pathpoints

    def publish_passage_passing_info(self, ppi):
        ppi_lst = []
        if ppi is not None:
            for i in range(len(ppi)):
                ppi_lst.append(ppi[i][0])
                ppi_lst.append(ppi[i][1])
        
        return ppi_lst


    def update_traj(self):
        
        # update communication trajectory
        self.CommuTraj.index=self.index 
        self.CommuTraj.duration=self.K*self.h
        self.CommuTraj.validity=self.validity
        self.CommuTraj.sequence=self.sequence
        self.CommuTraj.traj_x=self.traj[0].tolist()
        self.CommuTraj.traj_y=self.traj[1].tolist()
        if len(self.traj)>2:
            self.CommuTraj.traj_theta=self.traj[2].tolist()
        self.CommuTraj.shape=self.shape.tolist()
        self.CommuTraj.h = self.h

        # update execution trajectory
        self.ExecuteTraj.duration=self.K*self.h
        self.ExecuteTraj.sequence=self.sequence
        self.ExecuteTraj.start_time=time.time()
        self.ExecuteTraj.type=self.type
        self.ExecuteTraj.h=self.h
        self.ExecuteTraj.traj_x=self.traj[0].tolist()
        self.ExecuteTraj.traj_y=self.traj[1].tolist()
        if len(self.traj)>2:
            self.ExecuteTraj.traj_theta=self.traj[2].tolist()
        
        # update reference trajectory
        # print(self.ref_traj)
        if self.ref_traj is None:
            return
        self.RefExecuteTraj.start_time=time.time()
        self.RefExecuteTraj.type=self.type
        self.RefExecuteTraj.traj_x=self.ref_traj[:,0].tolist()
        self.RefExecuteTraj.traj_y=self.ref_traj[:,1].tolist()
        if self.ref_traj.shape[1]>2:
            self.RefExecuteTraj.traj_theta=self.ref_traj[:,2].tolist()

    def execute_traj(self):

        # update execution trajectory
        self.Traj_pub.publish(self.ExecuteTraj)

        self.Ref_traj_pub.publish(self.RefExecuteTraj)

    # update self trajectory estimation based on the mpc trajectory planned (For omnidirectional only)
    def traj_est(self, obj_flag, nei=None):
        # obj_flag:
        #   1 : self trajectory time stamp estimation
        #   2 : others  trajectory time stamp estimation
        #       when obj_flag = 2, nei contains the neighbor object

        traj_state_est = [] # save all state
        traj_time_est = []

        if obj_flag == 1:
            traj_x = self.traj[0]
            traj_y = self.traj[1]
            start_time = self.ExecuteTraj.start_time    # start time of estimation
            mpc_tractive_point = self.dynamic.tractive
            if self.dynamic.path is None:
                return 
            whole_path = self.dynamic.path
            if self.dynamic.passage_passing_info is None:
                return
            passage_passing_info = self.dynamic.passage_passing_info

            passage_time_map = copy.deepcopy(self.self_passage_time_map)
            for i in range(len(passage_time_map)):
                passage_time_map[i] = -1
            
            
        elif obj_flag == 2:
            if nei.traj_x is None or nei.traj_y is None:
                return
            traj_x = nei.traj_x
            traj_y = nei.traj_y
            if nei.traj_start_time is None:
                return
            start_time = nei.traj_start_time
            if nei.tractive is None:
                return
            mpc_tractive_point = nei.tractive
            if nei.path is None:
                return
            whole_path = nei.path
            if nei.passage_passing_info is None:
                return
            passage_passing_info = nei.passage_passing_info

            passage_time_map = copy.deepcopy(nei.passage_time_map)
            for i in range(len(passage_time_map)):
                passage_time_map[i] = -1
            
        else:
            print("obj_flag needs to be 1 or 2")
            return
        
        # calculate average speed for the mpc trajectory
        dist = 0.0
        # break_idx = len(traj_x) - 1
        for i in range(1,len(traj_x)):
            x0 = np.array([traj_x[i-1],traj_y[i-1]])
            x1 = np.array([traj_x[i],traj_y[i]])
            delta = np.linalg.norm(x1-x0)
            
            # if delta > 0.05:
            #     dist += delta
            # else:
            #     break_idx = i
            #     break

            dist += delta

        # time_duration = (break_idx-1)*self.h     # mpc time consumption
        time_duration = (len(traj_x)-1)*self.h     # mpc time consumption

        if time_duration == 0:
            V_aver = 0.0

            if obj_flag == 1:
                self.V_aver = -1
        else:
            V_aver = dist / time_duration
        
            if obj_flag == 1:
                self.V_aver = V_aver
                print("Self average speed: ", V_aver)

        # check the nearest point on the path from the break idx point in mpc trajectory
        # mpc_last_point = np.array([traj_x[break_idx-1],traj_y[break_idx-1]])
        mpc_last_point = np.array([traj_x[-1],traj_y[-1]])
        n_tractive = mpc_tractive_point - mpc_last_point
        min_dist = 1e6
        min_idx = -1
        for i in range(len(whole_path)):
            if np.linalg.norm(whole_path[i]-mpc_last_point) < min_dist:
                n_nearest = whole_path[i] - mpc_last_point
                if np.dot(n_tractive,n_nearest) > 0:
                    min_dist = np.linalg.norm(whole_path[i]-mpc_last_point)
                    min_idx = i
        
        ppi_start_idx = -1
        if min_idx != -1:
            for i in range(len(passage_passing_info)):
                if min_idx <= passage_passing_info[i][0]:
                    ppi_start_idx = i
                    break

        time_s = time.time()
        n = 0
        # update passage passing time stamps
        if len(whole_path) > 1:
            # has new target
            if V_aver > 0.2:
                dist = 0.0
                
                # for i in range(1,break_idx):
                for i in range(1,len(traj_x)):
                    x0 = np.array([traj_x[i-1],traj_y[i-1]])
                    x1 = np.array([traj_x[i],traj_y[i]])
                    delta = np.linalg.norm(x1-x0)
                    line_tmp = line(x0,x1)
                    check_result = detect_passage_passing(self.passage_pairs, line_tmp)
                    n += 1

                    if len(check_result) > 0:
                        # passage is passed
                        passage_idx_lst = check_result
                        for idx in passage_idx_lst:
                            passage = self.passage_pairs[idx]
                            passage_pt1 = np.array(passage[0])
                            passage_pt2 = np.array(passage[1])
                            mid_pt = (passage_pt1 + passage_pt2) / 2
                            dist_tmp = dist + np.linalg.norm(mid_pt-x0)
                            time_tmp = dist_tmp/V_aver
                            passage_time_map[idx] = start_time + time_tmp

                    dist += delta  
                    traj_state_est.append(x1)
                    traj_time_est.append(start_time + dist/V_aver)  

                dist = 0.0
                dist += np.linalg.norm(whole_path[min_idx]-mpc_last_point)

                traj_state_est.append(whole_path[min_idx])
                traj_time_est.append(start_time + time_duration + dist/V_aver) 

                dist_max = self.com_radius * 1.3  # maximum distance to be estimated, to avoid too much time consumption
                
                ppi_idx = ppi_start_idx
                for i in range(min_idx+1,len(whole_path)):
                    
                    if dist > dist_max:
                        break

                    x0 = whole_path[i-1]
                    x1 = whole_path[i]
                    delta = np.linalg.norm(x1-x0)
                    line_tmp = line(x0,x1)

                    passing_passage = False
                    passage_idx_lst = []
                    
                    if ppi_idx != -1:
                        for k in range(ppi_idx, len(passage_passing_info)):
                            if (i-1)==passage_passing_info[k][0]:
                                passage_idx_lst.append(passage_passing_info[k][1])
                            else:
                                ppi_idx = k
                                break
                    
                    if len(passage_idx_lst) > 0:
                        passing_passage = True

                    if passing_passage:
                        # passage is passed

                        for idx in passage_idx_lst:
                            passage = self.passage_pairs[idx]
                            passage_pt1 = np.array(passage[0])
                            passage_pt2 = np.array(passage[1])
                            mid_pt = (passage_pt1 + passage_pt2) / 2
                            dist_tmp = dist + np.linalg.norm(mid_pt-x0)
                            time_tmp = dist_tmp/V_aver
                            passage_time_map[idx] = start_time + time_duration + time_tmp
                    
                    dist += delta

                    traj_state_est.append(x1)
                    traj_time_est.append(start_time + time_duration + dist/V_aver)
                
                # print('Trajectory time stamp estimation time cost:'+str(time.time()-time_s)+'s')
            else:
                if obj_flag == 1:
                    passage_time_map = self.self_passage_time_map


                if obj_flag == 2:
                    print("Neighbour "+str(nei.j_index)+" average speed is too slow: ", V_aver)
                    # when discussing with others, if the average speed is too slow, then we try to block the nearest passage
                    other_pos = np.array([traj_x[0],traj_y[0]])
                    self_path = self.traj_state_est

                    if self_path is not None:
                        min_dist = 1e6
                        min_idx = -1
                        for i in range(len(self_path)):
                            if np.linalg.norm(self_path[i]-other_pos) < min_dist:
                                min_dist = np.linalg.norm(self_path[i]-other_pos)
                                min_idx = i
                        
                        other_target = whole_path[-1]
                        other_arrive_at_target = np.linalg.norm(other_target - other_pos) < 1e-2

                        self_dir = self.traj_state_est[1] - self.traj_state_est[0]
                        other_dir = other_pos - self.traj_state_est[0]
                        on_the_way = np.dot(self_dir,other_dir) > 0
                        

                        if not detect_line_collision(self.dynamic.obstacle_list, line(self_path[min_idx],other_pos)) and not other_arrive_at_target and on_the_way:
                            print("Neighbour "+str(nei.j_index)+" is near the self path, its idx: "+str(min_idx))

                            # indices of self passage passed
                            self_passage_idx = np.where(np.array(self.self_passage_time_map) > 0)[0]

                            print("Self passage idx: ", self_passage_idx)

                            nearest_passage_idx = -1
                            for i in range(1, min_idx+1,1):
                                x0 = self_path[i-1]
                                x1 = self_path[i]

                                for idx in self_passage_idx:
                                    line_path = line(x0,x1)

                                    passage = self.passage_pairs[idx]
                                    ext_r = 0.2
                                    p1 = np.array(passage[0])
                                    p2 = np.array(passage[1])
                                    line_p1 = p1 + ext_r * (p1 - p2)
                                    line_p2 = p2 + ext_r * (p2 - p1)
                                    line_passage = line(line_p1,line_p2)

                                    if detect_line_line_collision(line_path, line_passage):
                                        nearest_passage_idx = idx
                            
                            if nearest_passage_idx == -1:
                                for i in range(min_idx+1,len(self_path)):
                                    x0 = self_path[i-1]
                                    x1 = self_path[i]
                                    find_flag = False

                                    for idx in self_passage_idx:
                                        
                                        line_path = line(x0,x1)

                                        passage = self.passage_pairs[idx]
                                        ext_r = 0.2
                                        p1 = np.array(passage[0])
                                        p2 = np.array(passage[1])
                                        line_p1 = p1 + ext_r * (p1 - p2)
                                        line_p2 = p2 + ext_r * (p2 - p1)
                                        line_passage = line(line_p1,line_p2)

                                        if detect_line_line_collision(line_path, line_passage):
                                            nearest_passage_idx = idx
                                            find_flag = True
                                            break
                                    
                                    if find_flag:
                                        break
                            
                            if nearest_passage_idx != -1:
                                passage_time_map[nearest_passage_idx] = 0
                            
                            print("Neareast idx is: ", nearest_passage_idx)

                            passage = self.passage_pairs[nearest_passage_idx]
                            p1 = np.array(passage[0])
                            p2 = np.array(passage[1])
                            print("Nearest mid point: ", (p1+p2)/2)
                        else:
                            print("Neighbour "+str(nei.j_index)+" is not near the self path")

            if obj_flag == 1:
                self.self_passage_time_map = passage_time_map
                print("Self passage time map: ", np.where(np.array(self.self_passage_time_map) > 0)[0])
                if len(traj_state_est) > 0 and len(traj_time_est) > 0:
                    self.traj_state_est = np.array(traj_state_est)
                    self.traj_time_est = np.array(traj_time_est)
            elif obj_flag == 2:
                nei.passage_time_map = passage_time_map
                if len(traj_state_est) > 0 and len(traj_time_est) > 0:
                    nei.traj_state_est = np.array(traj_state_est)
                    nei.traj_time_est = np.array(traj_time_est)
        
        # if obj_flag == 2 and nei.j_index == 2:
        #     for i in range(len(nei.passage_time_map)):
        #         print('Passage pair '+str(i)+' passing time: '+str(nei.passage_time_map[i]))
        # print("n: ", n)
        
    def tra_est_markerarray(self, traj_state_est, traj_time_est, idx):

        scale = 0.05
        traj = traj_state_est
        time = traj_time_est
        # time_norm = (time - time[0])/(time[-1]-time[0])
        time_final = max(self.initial_time + 40, time[-1])
        time_norm = (time - self.initial_time)/(time_final - self.initial_time)
        cmap = plt.cm.jet  
        colors = [cmap(t) for t in time_norm]

        markerArray = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markerArray.markers.append(clear_marker)

        new_marker = Marker()
        new_marker.header.frame_id = "map"
        new_marker.header.stamp = rospy.Time.now()
        new_marker.ns = "traj_est"
        new_marker.id = idx
        new_marker.type = Marker.LINE_STRIP
        new_marker.action = Marker.ADD
        new_marker.scale.x = scale
        new_marker.pose.orientation.w = 1.0

        for i in range(len(traj)):
            p1 = Point()
            p1.x = traj[i][0]
            p1.y = traj[i][1]

            color1 = ColorRGBA()
            color1.r = colors[i][0]
            color1.g = colors[i][1]
            color1.b = colors[i][2]
            color1.a = 1.0
            
            new_marker.colors.append(color1)
            new_marker.points.append(p1)     

            # error_marker = Marker()
            # error_marker.header.frame_id = "map"
            # error_marker.header.stamp = rospy.Time.now()
            # error_marker.ns = "self_traj_est_error"
            # error_marker.id = i
            # error_marker.type = Marker.SPHERE
            # error_marker.action = Marker.ADD  

            # error_marker.pose.position.x = traj[i][0]
            # error_marker.pose.position.y = traj[i][1]
            # error_marker.pose.position.z = 0.0
            # error_marker.scale.x = max(self.traj_est_radius[i]*2, 0.01)
            # error_marker.scale.y = max(self.traj_est_radius[i]*2, 0.01)
            # error_marker.scale.z = 0.01

            # error_marker.pose.orientation.w = 1.0

            # error_marker.color.r = colors[i][0]
            # error_marker.color.g = colors[i][1]
            # error_marker.color.b = colors[i][2]
            # error_marker.color.a = 0.2

            # markerArray.markers.append(error_marker) 


        markerArray.markers.append(new_marker)

        return markerArray
    
    def replanning_check(self):

        if len(self.dynamic.delta_t_lst) > 10 and len(self.dynamic.aver_vel_lst) > 10:
            last_delta_t = self.dynamic.delta_t_lst[-1]
            aver_vel = self.dynamic.aver_vel_lst[-1]
        else:
            return 


        if last_delta_t > 1.0 and np.linalg.norm(self.dynamic.terminal_p-self.path[-1]) > 0.1:  # TODO: parameter setting
            rospy.wait_for_service('replanning_check')

            try:
                pos = PathPoint(x=self.dynamic.p[0],y=self.dynamic.p[1])
                request = ReplanningCheckRequest()
                request.index = self.index
                request.new_vel = aver_vel
                request.time = time.time() + self.validity
                request.pos = pos
                request.block = self.dynamic.tractive_block

                response = self.Replan_check(request)
                if response.success:
                    print('Receive replanning response')
                else:
                    print('Replanning is still under processing')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    def run(self,seq):


        self.sequence=seq

        # debugging
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tractive"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        points = []
        for i in range(len(self.dynamic.tractive_lst)):
            Point_tmp = Point()
            Point_tmp.x = self.dynamic.tractive_lst[i][0]
            Point_tmp.y = self.dynamic.tractive_lst[i][1]
            Point_tmp.z = 0.0
            points.append(Point_tmp)
        marker.points = points
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.tractive_pub.publish(marker)

        self.last_time=time.time()

        # update path information
        self.dynamic.set_path(self.path_start_time, self.path, self.path_vel, self.path_time)

        t_1 = time.time()
        self.dynamic.get_tractive_point(self.validity)
        print("Time for get_tractive_point: ", time.time()-t_1)

        # update trajectory for commnication and execution
        self.update_traj()

        # publish traj for low-level controler
        self.execute_traj()

        self.scan()

        # update communication trajectory for each neighbour
        for nei in self.neighbor_list:
            nei.sequence=seq
            nei.last_time=self.last_time
            nei.CommuTraj=copy.deepcopy(self.CommuTraj)

        # begin receiving information
        for nei in self.neighbor_list:
            nei.state=1

        # initiating agreements for others 
        self.initiate_agreement()

        # waiting time for receving neighbours' information
        time.sleep(self.t_w)

        # print('<=================================>')
        # stop receiving information
        for nei in self.neighbor_list:
            nei.state=0

        # print('<=================================>')
        # time_est_start = time.time()
        # self.traj_est(1)
        # print('Self trajectory estimation time cost:'+str(time.time()-time_est_start)+'s')
        # print('<=================================>')

        # time_est_start_2 = time.time()
        # # sequentially estimate the trajectory for each neighbour
        # for nei in self.neighbor_list:
        #     time_agent = time.time()

        #     if nei.out_of_range:
        #         continue

        #     self.traj_est(2,nei)
        #     # print('Agent'+str(nei.j_index)+' trajectory estimation time cost:'+str(time.time()-time_agent)+'s')
        
        # # for nei in self.neighbor_list:
        # #     for i in range(8):
        # #         result = other_traj_est(nei.j_index, [nei.traj_x, nei.traj_y], nei.traj_start_time, nei.tractive, nei.path, nei.passage_time_map, nei.h, self.passage_pairs)

        # #         if result is not None:
        # #             if len(result[1]) > 0 and len(result[2]) > 0:
        # #                 self.neighbor_list[0].passage_time_map = result[0]
        # #                 self.neighbor_list[0].traj_state_est = np.array(result[1])
        # #                 self.neighbor_list[0].traj_time_est = np.array(result[2])

        # # concurrent estimation for each neighbour
        # # with concurrent.futures.ProcessPoolExecutor() as executor:

        # #     futures = [executor.submit(other_traj_est, nei.j_index, [nei.traj_x, nei.traj_y], nei.traj_start_time, nei.tractive, nei.path, nei.passage_time_map, nei.h, self.passage_pairs) for nei in self.neighbor_list]

        # #     results = [future.result() for future in futures]
        # #     for i in range(len(results)):
        # #         if results[i] is not None:
        # #             if len(results[i][1]) > 0 and len(results[i][2]) > 0:
        # #                 self.neighbor_list[i].passage_time_map = results[i][0]
        # #                 self.neighbor_list[i].traj_state_est = np.array(results[i][1])
        # #                 self.neighbor_list[i].traj_time_est = np.array(results[i][2])

        # print('Others trajectory estimation time cost:'+str(time.time()-time_est_start_2)+'s')
        # print('<=================================>')
        
        # time_bias = time.time()
        # other_passage_passing_time = []
        # other_passage_passing_time_no_bias = []
        # other_passage_passing_time_no_bias_idx = []
        # for nei in self.neighbor_list:
        #     if nei.out_of_range:
        #         continue

        #     if nei.j_index > self.index:
        #         # smaller index, higher priority
        #         continue
        #     # passage_passing_time_relative = [time-time_bias for time in nei.passage_time_map]
        #     passage_passing_time_relative = []
        #     for t in nei.passage_time_map:
        #         if t > 0:
        #             t_relative = t - time_bias
        #             if t_relative > 0.05:
        #                 passage_passing_time_relative.append(t_relative)
        #             else:
        #                 passage_passing_time_relative.append(-1)
        #         elif t == 0:
        #             passage_passing_time_relative.append(0)
        #         else:
        #             passage_passing_time_relative.append(-1)

        #     other_passage_passing_time.append(passage_passing_time_relative)

        #     passage_passing_time = [time for time in nei.passage_time_map]
        #     other_passage_passing_time_no_bias.append(passage_passing_time)
        #     other_passage_passing_time_no_bias_idx.append(nei.j_index)
        
        # # evaluate the path taken now
        # self_score = 0.0
        # for i in range(len(self.self_passage_time_map)):
        #     self_time = self.self_passage_time_map[i]
        #     if self_time > 0:
        #         for j in range(len(other_passage_passing_time_no_bias)):
        #             other_time = other_passage_passing_time_no_bias[j][i]
        #             if other_time > 0:

        #                 cos = -1.0
        #                 j_index = other_passage_passing_time_no_bias_idx[j]
                        
        #                 try:
        #                     nei_j_index = self.neighbor_index.index(j_index)
        #                 except ValueError:
        #                     print("Error: Cannot find the neighbor index in the neighbor list")
                        
        #                 nei = self.neighbor_list[nei_j_index]
        #                 if nei.traj_x is not None and nei.traj_y is not None:
        #                     j_pos = np.array([nei.traj_x[0],nei.traj_y[0]])
        #                     passage_mid_pt = (np.array(self.passage_pairs[i][0]) + np.array(self.passage_pairs[i][1]))/ 2
        #                     self_pos = np.array([self.traj[0][0],self.traj[1][0]])
        #                     # print("passage_mid_pt: ", passage_mid_pt)
        #                     # print("j_pos: ", j_pos)
        #                     # print("self_pos: ", self_pos)

        #                     n_j = passage_mid_pt - j_pos
        #                     n_self = passage_mid_pt - self_pos
        #                     if np.linalg.norm(n_j) > 1e-2 and np.linalg.norm(n_self) > 1e-2:
        #                         cos = np.dot(n_j,n_self)/(np.linalg.norm(n_j)*np.linalg.norm(n_self))
        #                     # print("Cosine value: ", cos)
        #                 if cos > 0 :
        #                     continue

        #                 delta_time = abs(self_time - other_time)
        #                 discount_factor = 1.1 * exp(-0.1 * abs(self_time - time_bias))  # -0.2

        #                 passage_pt1 = np.array(self.passage_pairs[i][0])
        #                 passage_pt2 = np.array(self.passage_pairs[i][1])
        #                 passage_width = np.linalg.norm(passage_pt1-passage_pt2)

        #                 # passage_weight = - 0.1
        #                 self_score += exp(-0.5*delta_time) * discount_factor / passage_width
        #             elif other_time == 0:
        #                 self_score += 5.0

        # print("Self score: ", self_score)

        # # self.dynamic.set_target(self.real_target)
        # self.new_target_set = self.dynamic.set_target(self.real_target, other_passage_passing_time, self.V_aver)
        # if self.index == 1:
        #     if self.traj_state_est is not None and self.traj_time_est is not None:
        #         self.self_traj_est_pub.publish(self.tra_est_markerarray(self.traj_state_est, self.traj_time_est, self.index))
        #     for i in range(len(self.neighbor_list)):
        #         nei = self.neighbor_list[i]
        #         if nei.traj_state_est is not None and nei.traj_time_est is not None:
        #             self.traj_est_pub_list[i].publish(self.tra_est_markerarray(nei.traj_state_est, nei.traj_time_est, nei.j_index))
        
        # if not self.new_target_set and self_score > 3.0 and SET.test_mode == 4: # 1.5
        #     self.dynamic.path_plan(other_passage_passing_time, self.V_aver)
        #     self.dynamic.get_tractive_point()

        # print("Path Estimation and change time cost: "+str(time.time()-time_bias))
        # print('<=================================>')

        # print("Estimation Total Time Cost: "+str(time.time()-time_est_start)+'s')
        # print('<=================================>')

        # self.dynamic.set_path(self.path_start_time, self.path, self.path_vel, self.path_time)

        c_start=time.time()
        # trajectory planning
        self.traj=self.plan_traj()
        
        compute_time=time.time()-c_start
        self.time_list=np.append(self.time_list,compute_time)

        print('Computation uses: '+str(compute_time))
        print('Validity is: '+str(self.validity))
        print('<=================================>')
        print('Time left: ', (self.t_c+self.t_w)-(time.time()-self.last_time))
        print(' ')

        while True:
            if (time.time()-self.last_time>self.t_c+self.t_w):
                if seq>0:
                    self.validity=0.3*(time.time()-self.last_time)+0.7*self.validity
                break
            else:
                time.sleep(0.001)
        
        # check to replanning
        if SET.test_mode == 4 or SET.test_mode == 3 or (SET.test_mode == 0 and SET.replanning):
            replanning_time = time.time()
            self.replanning_check()
            print('<=================================>')
            print("Replanning time cost: ", time.time()-replanning_time)

def get_sample(P,h,t):

    if abs(h)<1e-5:
        return 0.0

    i=t/h
    i_c=int(np.ceil(i))
    l=len(P)

    if i_c>=l-1:
        p_c=P[-1]
    else:
        p_c=P[i_c]

    i_f=int(np.floor(i))

    if i_f>=l-1:
        return P[-1]
    else:
        p_f=P[i_f]

    return p_c*(i-i_f)+p_f*(i_c-i)
        
    
def main():

    import sys,SET
    import signal

    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)


    if len(sys.argv)==1:
        index=0
        obs_env_idx = 0
    elif len(sys.argv)==2:
        index=sys.argv[1] 
        obs_env_idx = 0
    else:
        index=sys.argv[1]
        obs_env_idx = sys.argv[2]

    i=0
    for agent in SET.agent_list:
        if agent['index']==int(index):
            break
        i+=1

    Agent=SET.agent_list[i]

    try:
        Planner = planner(Agent, int(obs_env_idx))
        i=0
        while True:
            try:
                Planner.run(i)
                i+=1
            except KeyboardInterrupt:
                break 
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    main()
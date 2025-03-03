#!/usr/bin/env python

################################################################
# 
# Author: Mike Chen, Hauser Dong
# From Peking university
# Last update: 2025.03.03
# 
###############################################################

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

        self.path_start_time = time.time()
        self.path = None
        self.path_vel = None
        self.path_time = None


        from Dynamic.Omnidirection.Mini_mec import mini_mec
        self.dynamic=mini_mec(self.index,self.shape,self.K,self.h,self.ini_state,self.target,obs_env_idx)
        self.shape=self.dynamic.shape.ravel()
        traj_theta=np.zeros(self.K+1)
          
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

        # update trajectory for commnication and execution
        self.update_traj()

        # publish traj for low-level controler
        self.execute_traj()

        self.traj_state_est = None
        self.traj_time_est = None

        self.V_aver = -1

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
                    self.neighbor_list[-1].i_com_radius = self.com_radius

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
    
    def replanning_check(self):

        if len(self.dynamic.delta_t_lst) > 10 and len(self.dynamic.aver_vel_lst) > 10:
            last_delta_t = self.dynamic.delta_t_lst[-1]
            aver_vel = self.dynamic.aver_vel_lst[-1]
        else:
            return 

        beta = rospy.get_param('beta', default=1.0)
        if last_delta_t > beta and np.linalg.norm(self.dynamic.terminal_p-self.path[-1]) > 0.1:
            rospy.wait_for_service('replanning_check')

            try:
                pos = PathPoint(x=self.dynamic.p[0],y=self.dynamic.p[1])
                request = ReplanningCheckRequest()
                request.index = self.index
                request.new_vel = aver_vel
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

        c_start=time.time()
        # trajectory planning
        self.traj=self.plan_traj()
        compute_time=time.time()-c_start

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
            # print('<=================================>')
            # print("Replanning time cost: ", time.time()-replanning_time)

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
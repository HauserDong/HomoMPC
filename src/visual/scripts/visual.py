#!/usr/bin/env

################################################################
# 
# Author: Mike Chen, Hauser Dong
# From Peking university
# Last update: 2025.03.03
# 
###############################################################

import rospy 
from bring_up.msg import  ExecuteTraj
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, PolygonStamped, Point32
import numpy as np
import time
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '../../local_planner/planner/scripts'))
import SET

class object():

    def __init__(self,name) -> None:
        
        self.name=name

        self.index=name[5:]

        self.sub=rospy.Subscriber(name,ExecuteTraj,callback=self.saveTraj)

        # self.ref_sub=rospy.Subscriber('Ref_Traj'+str(self.index),ExecuteTraj,callback=self.saveRefTraj)

        self.position_pub=rospy.Publisher('vis_p'+self.index,Marker,queue_size=1)

        self.traj_pub=rospy.Publisher('vis_traj'+self.index,Path,queue_size=1)

        self.past_traj_pub=rospy.Publisher('past_traj'+self.index,Path,queue_size=1)

        self.ref_traj_pub=rospy.Publisher('ref_traj'+self.index,Path,queue_size=1)

        self.traj=ExecuteTraj()

        self.ref = ExecuteTraj()

        self.start=time.time()

        self.vis_traj=Path()

        self.past_traj=Path()

        self.ref_traj=Path()

        self.vis_traj.header.frame_id="map"

        self.past_traj.header.frame_id="map"

        self.ref_traj.header.frame_id="map"

        self.update_time=time.time()

        self.type=self.traj.type

        self.obtain=False

        while True:
            if self.obtain:
                break
            else:
                time.sleep(0.1)

    def saveRefTraj(self,ref) -> None:

        self.ref=ref

    def saveTraj(self,traj) -> None:


        if hasattr(self, 'traj') and hasattr(self.traj, 'type') and hasattr(self, 'type') and self.type!=self.traj.type:

            self.obtain=True

            self.type=self.traj.type

            self.vis=Mini_mec(self.index)

        self.traj=traj

        self.start=time.time()

        self.update_time=time.time()


    def publish(self) -> None:


        import copy

        now=time.time()

        # long time no update, delete it
        if now-self.update_time>2.0:
            self.vis_traj.poses.clear()
            self.traj_pub.publish(self.vis_traj)
            self.past_traj.poses.clear()
            self.past_traj_pub.publish(self.past_traj)
            self.ref_traj.poses.clear()
            self.ref_traj_pub.publish(self.ref_traj)
            return None

        r_t=now-self.start
        h=self.traj.h

        if r_t>self.traj.duration*0.95:

            px=get_sample(self.traj.traj_x,h,self.traj.duration*0.95)
            py=get_sample(self.traj.traj_y,h,self.traj.duration*0.95)

            p_list=np.array([[px,py,0.0],[px,py,0.0]])

        else:

            px=np.zeros(30)
            py=np.zeros(30)
            i=0
            for t in np.linspace(r_t,self.traj.duration,30):
                px[i]=get_sample(self.traj.traj_x,h,t)
                py[i]=get_sample(self.traj.traj_y,h,t)
                i+=1

            p_list=np.zeros((30,3))

            p_list[:,0]=px 
            p_list[:,1]=py

        self.vis_traj.poses.clear()
    #    def push_point(self.vis_traj.points,p_list):
        pose=PoseStamped()
        for p in p_list:
            pose.pose.position.x=p[0]
            pose.pose.position.y=p[1]
            pose.pose.position.z=p[2]
            self.vis_traj.poses.append(copy.deepcopy(pose))

        self.ref_traj.poses.clear()
        pose=PoseStamped()
        traj_x_save = np.array(self.ref.traj_x).copy()
        traj_y_save = np.array(self.ref.traj_y).copy()
        for i in range(len(traj_x_save)):
            pose.pose.position.x=traj_x_save[i]
            pose.pose.position.y=traj_y_save[i]
            pose.pose.position.z=0.0
            self.ref_traj.poses.append(copy.deepcopy(pose))

        p=p_list[0]
        
        theta=get_sample(self.traj.traj_theta,self.traj.h,r_t)
        self.vis.update(p,theta)

        pose=PoseStamped()
        pose.pose.position.x=p[0]
        pose.pose.position.y=p[1]
        pose.pose.position.z=p[2]
        self.past_traj.poses.append(pose)

        self.past_traj_pub.publish(self.past_traj)
        self.traj_pub.publish(self.vis_traj)
        self.position_pub.publish(self.vis.vis_position)
        self.ref_traj_pub.publish(self.ref_traj)
        

class Mini_mec():

    def __init__(self, index) -> None:

        self.vis_position=Marker()

        self.vis_position.color.r=SET.rgb_color_list[int(index)][0]
        self.vis_position.color.g=SET.rgb_color_list[int(index)][1]
        self.vis_position.color.b=SET.rgb_color_list[int(index)][2]
        self.vis_position.color.a=0.95

        self.vis_position.header.frame_id="map" 

        self.vis_position.type=Marker.SPHERE

        self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0
        self.vis_position.pose.position.y=0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=SET.agent_list[int(index)]['radius'][0]*2
        self.vis_position.scale.y=SET.agent_list[int(index)]['radius'][0]*2
        self.vis_position.scale.z=SET.agent_list[int(index)]['radius'][0]*2

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2] + 1.0
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)



class rviz_pub():

    def __init__(self) -> None:

        rospy.init_node('vis_pub', anonymous=False)
        self.topic_list=[]
        self.object_list=[]
        self.obs_list = []


    def scan(self) -> None:
        
        topics=rospy.get_published_topics()

        for topic in topics:
            name=topic[0]
            if name[0:5]=='/Traj':
                if not(name in self.topic_list):
                    self.topic_list+=[name]
                    self.object_list+=[object(name)]


    def run(self) -> None:
        # rospy.loginfo("hello, in vis")

        self.scan()

        for ob in self.object_list:
            ob.publish()

        time.sleep(0.08)
        

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

    import signal
    signal.signal(signal.SIGINT, quit)  # keyboard interruption                              
    signal.signal(signal.SIGTERM, quit) # termination

    try:
        vis=rviz_pub()

        while True:
            # vis.obs_vis()
            vis.run()
            try:
                vis.run()
            except rospy.ROSInterruptException:
                break 
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':

    main()
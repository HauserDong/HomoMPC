import os  
import sys
import copy
import rospy
import numpy as np
import time
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from bring_up.msg import ExecuteTraj, RefPath, PathPoint
from bring_up.srv import StartPlanning, StartPlanningResponse, ReplanningCheck, ReplanningCheckResponse

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append( current_dir)
sys.path.append(os.path.join(current_dir, '../../../local_planner/planner/scripts'))
import SET
from geometry import *
import generate_homotopic_path as ghp

class HomoCoPathNode:
    def __init__(self):

        rospy.init_node('HomoCoPath', anonymous=True)
        rospy.loginfo('HomoCoPath node initialized')
    
        # subsriber
        self.targets_sub = {}
        self.trajs_sub = {}
        self.past_trajs_sub = {}

        # publisher
        self.paths_pub = {}

        # service
        self.co_path_srv = rospy.Service('start_planning', StartPlanning, self.co_path_planning)
        self.replanning_check_srv = rospy.Service('replanning_check', ReplanningCheck, self.replanning_check)

        # saved data
        self.targets = {}
        self.starts = {}
        self.paths_to_pub = {}
        self.past_trajs = {}

        self.pptm = {}      # records relative time, the start time refers to paths_to_pub[idx].start_time
        self.paths = {}
        self.pos = {}

        # parameters
        self.index_lst = []
        self.recal_flags = {}
        self.vels = {}
        self.path_pptm_scores = {}
        self.agent_block = {}
        self.score_threshold = 1.5

        # record data
        self.start_planning_time = None
        self.finish_times = {}
        self.total_path_lengths = {}
        self.all_reach_goal = False
        self.replan_num = 0

        # generate obstacle list
        self.index = 0
        shape = SET.agent_list[0]['radius']
        ExtendWidth = np.amax(shape)+0.01
        obs_env_idx = rospy.get_param('obs_env_idx', default=4)
        self.obstacle_list, self.obstacle_list_nest = Build_ExtensionZone(SET.ini_obstacle_list[obs_env_idx],ExtendWidth)
        
        # initialize path planner
        visualize = False
        self.alpha_eval = -1.0
        self.alpha_plan = -0.3
        self.homopathplanner = ghp.HomoPathPlanner(self.index, self.obstacle_list_nest, SET.map_range, SET.resolution/5, SET.test_mode, self.alpha_plan, visualize)
        self.whole_passage = self.homopathplanner.GetExtendedVisibilityCheck()
        
        # debug
        # for i in range(len(self.whole_passage)):
        #     passage = self.whole_passage[i]
        #     print('Passage pair '+str(int(i/2))+': '+str(passage[0])+' '+str(passage[1]))
        #     # print("Passage pair "+str(i)+" length: ", np.linalg.norm(np.array(passage[0])-np.array(passage[1])))

    def replanning_check(self, req):

        agent_idx = req.index
        new_vel = req.new_vel
        pos = [req.pos.x, req.pos.y]
        t = req.time
        block = req.block

        if self.recal_flags[agent_idx] == False:
            rospy.loginfo('Received replanning check request for agent %d, its new average velocity is %f' % (req.index, req.new_vel))

            self.recal_flags[agent_idx] = True
            self.vels[agent_idx] = new_vel
            # self.paths_to_pub[agent_idx].start_time = t
            # self.pos[agent_idx] = pos
            self.agent_block[agent_idx] = block

            return ReplanningCheckResponse(True)
        else:
            return ReplanningCheckResponse(False)
    
    def get_partial_path(self, agent_idx):
        # update path
        pos = copy.deepcopy(self.pos[agent_idx])
        new_path = []
        nearest_idx = -1
        second_nearest_idx = -1
        min_dist = 1e6
        for i in range(len(self.paths[agent_idx])):
            pos_path = np.array(self.paths[agent_idx][i])
            dist = np.linalg.norm(pos_path - np.array(pos))
            if dist < min_dist:
                min_dist = dist
                second_nearest_idx = nearest_idx
                nearest_idx = i
        
        # rospy.loginfo("Agent "+str(agent_idx)+" min distance before is "+str(min_dist))
        # print("nearest point: ", self.paths[agent_idx][nearest_idx])
        if nearest_idx != -1:
            min_dist = 1e6
            if second_nearest_idx != -1:
                # print("Agent "+str(agent_idx)+" second nearest index: "+str(second_nearest_idx))
                true_nearest_pt = None
                sec_min_pt = np.array(self.paths[agent_idx][second_nearest_idx])
                min_pt = np.array(self.paths[agent_idx][nearest_idx])
                n = 10
                for i in range(n):
                    p = sec_min_pt + (min_pt - sec_min_pt) * (i+1) / n
                    dist = np.linalg.norm(p - np.array(pos))
                    if dist < min_dist:
                        min_dist = dist
                        true_nearest_pt = p.tolist()
                if true_nearest_pt is not None:
                    new_path.append(true_nearest_pt)
                    # print("Agent "+str(agent_idx)+" true nearest point: ", true_nearest_pt)
            # else:
            #     print("Agent "+str(agent_idx)+" no second nearest index")

            new_path.extend(copy.deepcopy(self.paths[agent_idx][nearest_idx:]))
            # rospy.loginfo("Agent "+str(agent_idx)+" min distance after is "+str(min_dist))

        return new_path

    def recal_passage_time_map(self):

        HIGH_PRIORITY_CHANGE = False
        
        for agent_idx in self.index_lst:

            recal_flag = self.recal_flags[agent_idx]

            if SET.test_mode == 4:
                if recal_flag:

                    # new_path = self.get_partial_path(agent_idx)
                    path = self.paths[agent_idx]
                    
                    # recalculate passage time map
                    new_pptm = self.homopathplanner.CalculatePassageTimeMap(path, self.vels[agent_idx], self.pptm[agent_idx], self.starts[agent_idx])

                    # update passage time map to other agents
                    self.pptm[agent_idx] = new_pptm

                    # update new path time
                    path_time = self.homopathplanner.CalculatePathTime(path, self.vels[agent_idx], self.paths_to_pub[agent_idx].path_time, self.starts[agent_idx])

                    # debug
                    # passage_idx = 24
                    # if agent_idx == 6 or agent_idx == 2:
                    #     print("Agent "+str(agent_idx)+ " Passage " + str(passage_idx) + " time: ")
                    #     print(new_pptm[passage_idx*2])
                    #     print(new_pptm[passage_idx*2+1])

                    # reevaluate path
                    replan_flag = self.eval_path(agent_idx)

                    if replan_flag or self.agent_block[agent_idx]:
                        # print("Agent "+str(agent_idx)+" recalculated passage time map:")
                        # print(new_pptm[71])
                        print("***self velocity changes, replan for agent***")
                        print("block:" + str(self.agent_block[agent_idx]))
                        print("replan flag:" + str(replan_flag))
                        self.replan(agent_idx)
                    else:
                        self.paths_to_pub[agent_idx].path_time = path_time
                        self.paths_pub[agent_idx].publish(self.paths_to_pub[agent_idx])
                    
                elif HIGH_PRIORITY_CHANGE:
                    # reevaluate path
                    replan_flag = self.eval_path(agent_idx)

                    # print(self.agent_block[agent_idx])
                    if replan_flag or self.agent_block[agent_idx]:

                        print("***high priority agent replan for agent***")
                        self.replan(agent_idx)

                HIGH_PRIORITY_CHANGE = HIGH_PRIORITY_CHANGE or recal_flag
            
            elif SET.test_mode == 3:
                if recal_flag:
                    self.replan(agent_idx)
            elif SET.test_mode == 0 and SET.replanning:
                if recal_flag:
                    path = self.paths[agent_idx]

                    path_time = self.homopathplanner.CalculatePathTime(path, self.vels[agent_idx], self.paths_to_pub[agent_idx].path_time, self.starts[agent_idx])

                    self.paths_to_pub[agent_idx].path_time = path_time
                    self.paths_pub[agent_idx].publish(self.paths_to_pub[agent_idx])
        
        for agent_idx in self.index_lst:
            self.recal_flags[agent_idx] = False
    
    def replan(self, agent_idx):

        # TODO: check timestamp
        start = self.starts[agent_idx]
        target = self.targets[agent_idx]
        V = self.vels[agent_idx]

        rospy.logwarn('Replanning for agent %d, using velocity %f' % (agent_idx, V))
        self.replan_num += 1

        rel_time = time.time()
        agents_passage_passing_time = []
        for idx in self.index_lst:
            if idx < agent_idx:
                high_priority_agent_pptm = copy.deepcopy(self.pptm[idx])
                high_priority_start_time = self.paths_to_pub[idx].start_time
                for i in range(len(high_priority_agent_pptm)):
                    if high_priority_agent_pptm[i] > 0:
                        high_priority_agent_pptm[i] = high_priority_start_time + high_priority_agent_pptm[i] - rel_time

                agents_passage_passing_time.append(high_priority_agent_pptm)
        
        (path, path_vel, path_time, pptm) = self.homopathplanner.generate_homotopic_path(start, target, agents_passage_passing_time, V, True)

        # if agent_idx == 6:
        #     print("Agent "+str(agent_idx)+" replan passage time map:")
        #     print(pptm[24*2])
        #     print(pptm[24*2+1])

        #     print("Agent 2 passage time map:")
        #     print(agents_passage_passing_time[0][24*2])
        #     print(agents_passage_passing_time[0][24*2+1])

        path_pub_info = RefPath()
        path_pub_info.start_time = time.time()
        path_pub_info.path_vel = path_vel
        path_pub_info.path_time = path_time
        path_pub_info.path = self.path_transform(path)
        self.paths_to_pub[agent_idx] = path_pub_info

        # update this agent's passage time map to other agents
        self.pptm[agent_idx] = pptm
        self.eval_path(agent_idx)

        # record path
        self.paths[agent_idx] = path

        self.paths_pub[agent_idx].publish(path_pub_info)
        rospy.loginfo('Published path for agent %d, time stamp is %f' % (agent_idx, path_pub_info.start_time))

    def eval_path(self, agent_idx):

        eval_time = time.time()
        
        self_pptm = copy.deepcopy(self.pptm[agent_idx])
        self_start_time = self.paths_to_pub[agent_idx].start_time
        self_score = 0

        for i in self.index_lst:
            if i < agent_idx:
                # check for high priority agents
                other_pptm = copy.deepcopy(self.pptm[i])
                other_start_time = self.paths_to_pub[i].start_time

                for j in range(int(len(self_pptm)/2)):
                    if self_pptm[j*2] > 0 and self_pptm[j*2+1] > 0 and other_pptm[j*2] > 0 and other_pptm[j*2+1] > 0:

                        self_time_1 = self_start_time + self_pptm[j*2]
                        self_time_2 = self_start_time + self_pptm[j*2+1]
                        other_time_1 = other_start_time + other_pptm[j*2]
                        other_time_2 = other_start_time + other_pptm[j*2+1]

                        self_duration = [min(self_time_1, self_time_2), max(self_time_1, self_time_2)]
                        other_duration = [min(other_time_1, other_time_2), max(other_time_1, other_time_2)]

                        time_overlap_lower = max(self_duration[0], other_duration[0])
                        time_overlap_upper = min(self_duration[1], other_duration[1])

                        if time_overlap_lower < time_overlap_upper:
                            self_score += 1

                            # print("Agent "+str(agent_idx)+" Passage "+str(j)+" : " + str(self_time_1)+" "+str(self_time_2)+" / "+str(other_time_1)+" "+str(other_time_2))
                        else:
                            self_score += math.exp(self.alpha_eval*abs(time_overlap_upper - time_overlap_lower))

                            # print("Agent "+str(agent_idx)+" Passage "+str(j)+" : " + str(self_time_1)+" "+str(self_time_2)+" / "+str(other_time_1)+" "+str(other_time_2))

        if agent_idx not in self.path_pptm_scores:
            self.path_pptm_scores[agent_idx] = self_score
            print("Agent "+str(agent_idx)+" new score: "+str(self_score))
            return False
        else:
            old_score = self.path_pptm_scores[agent_idx]
            new_score = self_score

            self.path_pptm_scores[agent_idx] = new_score

            # print("Agent "+str(agent_idx)+" new score: "+str(new_score) + " old score: "+str(old_score))

            if new_score > self.score_threshold:
                print("self score: "+str(new_score))
                return True
            else:
                return False

    def get_target_callback(self, msg, index):
        
        px=msg.pose.position.x
        py=msg.pose.position.y
        theta=np.arctan2(2*(msg.pose.orientation.w*msg.pose.orientation.z+msg.pose.orientation.x*msg.pose.orientation.y),\
        (1-2*(msg.pose.orientation.y*msg.pose.orientation.y+msg.pose.orientation.z*msg.pose.orientation.z)))

        self.targets[index]=np.array([px,py])
        
        # rospy.loginfo('Received target for agent ' + str(index) + ': ' + str(self.targets[index]))

    def get_start_callback(self, msg, index):

        # TODO: need to check the exact time, use get_sample func
        traj_x = msg.traj_x
        traj_y = msg.traj_y
        start = np.array([traj_x[0], traj_y[0]])
        self.starts[index] = start

    def get_past_traj(self, msg, index):

        self.past_trajs[index] = msg.poses

    def co_path_planning(self, req):
        rospy.loginfo('Received request to start planning')
        self.finish_times.clear()
        self.total_path_lengths.clear()

        start_time = time.time()

        agents_passage_passing_time = []
        for idx in self.index_lst:
            start = self.starts[idx]
            target = self.targets[idx]
            V = 0.5
        
            (path, path_vel, path_time, pptm) = self.homopathplanner.generate_homotopic_path(start, target, agents_passage_passing_time, V, False)

            agents_passage_passing_time.append(pptm)

            # debug
            # passage_idx = 55
            # if idx == 8 or idx == 4:
            #     print("Agent "+str(idx)+ " Passage " + str(passage_idx) + " time: ")
            #     print(pptm[passage_idx*2])
            #     print(pptm[passage_idx*2+1])

            path_pub_info = RefPath()
            path_pub_info.path_vel = path_vel
            path_pub_info.path_time = path_time
            path_pub_info.path = self.path_transform(path)
            self.paths_to_pub[idx] = path_pub_info
            
            # record the average velocity
            self.vels[idx] = V

            # record passage time map
            self.pptm[idx] = pptm

            # record path
            self.paths[idx] = path
        
        rospy.loginfo('Homotopic Path Planning time: %f' % (time.time() - start_time))

        path_time_stamp = time.time()
        self.start_planning_time = path_time_stamp
        # publish path
        for idx, path in self.paths_to_pub.items():
            path.start_time = path_time_stamp
            self.eval_path(idx)
            self.paths_pub[idx].publish(path)
            rospy.loginfo('Published path for agent %d, its time stamp is %f' % (idx, path_time_stamp))

        return StartPlanningResponse(True)

    def path_transform(self, path):

        path_pub = []
        for p in path:
            path_point = PathPoint()
            path_point.x = p[0]
            path_point.y = p[1]
            path_pub.append(path_point)
        return path_pub
    
    def scan_topic(self):
        topics = rospy.get_published_topics()
        for topic, msg_type in topics:
            index = -1
            if len(topic) > 5 and topic[0:5] == '/goal':
                # subscribe to target
                index = int(topic[5:])
                if index not in self.targets_sub:
                    self.targets_sub[index] = rospy.Subscriber(topic, PoseStamped, self.get_target_callback, callback_args=index, queue_size=10)
                    rospy.loginfo('Subscribed to target for agent %d' % index)
            
            if len(topic) > 5 and topic[0:5] == '/Traj':
                # subscribe to agent's trajectory
                index = int(topic[5:])
                if index not in self.trajs_sub:
                    self.trajs_sub[index] = rospy.Subscriber(topic, ExecuteTraj, self.get_start_callback, callback_args=index, queue_size=10)
                    rospy.loginfo('Subscribed to trajectory for agent %d' % index)
                
            if len(topic) > 10 and topic[0:10] == '/past_traj': 
                # subscribe to agent's past trajectory
                index = int(topic[10:])
                if index not in self.past_trajs_sub:
                    self.past_trajs_sub[index] = rospy.Subscriber(topic, Path, self.get_past_traj, callback_args=index, queue_size=10)
                    rospy.loginfo('Subscribed to past trajectory for agent %d' % index)
            
            if index != -1 and index not in self.paths_pub:
                self.paths_pub[index] = rospy.Publisher('/Path'+str(index), RefPath, queue_size=1)
                self.recal_flags[index] = False
                self.agent_block[index] = False
                
                self.index_lst.append(index)
                self.index_lst.sort()
                
    def spin(self):

        import signal
        signal.signal(signal.SIGINT, quit)  # keyboard interruption                              
        signal.signal(signal.SIGTERM, quit) # termination

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                self.scan_topic()
                self.recal_passage_time_map()
                self.reach_goal_check()
                rate.sleep()
            except rospy.ROSInterruptException:
                break 
    
    def reach_goal_check(self):
        if self.start_planning_time is None:
            return

        all_reach_goal = True
        for idx in self.index_lst:
            agent_pos = self.starts[idx]
            agent_target = self.targets[idx]

            if np.linalg.norm(agent_pos - agent_target) > 0.1:
                all_reach_goal = False
            else:
                if idx not in self.finish_times and idx not in self.total_path_lengths:
                    self.finish_times[idx] = time.time() - self.start_planning_time

                    total_path = self.past_trajs[idx]
                    total_length = 0
                    for i in range(len(total_path)-1):
                        p1 = total_path[i].pose.position
                        p2 = total_path[i+1].pose.position
                        total_length += np.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)
                    self.total_path_lengths[idx] = total_length
        
        if all_reach_goal and not self.all_reach_goal:
            rospy.loginfo('All agents have reached their goals')
            for idx in self.index_lst:
                rospy.loginfo('Agent %d: Total path length: %f, Total time: %f' % (idx, self.total_path_lengths[idx], self.finish_times[idx]))
            self.all_reach_goal = True

            rospy.loginfo('Total length: %f' % np.sum(list(self.total_path_lengths.values())))
            rospy.loginfo('Total time: %f' % np.max(list(self.finish_times.values())))
            rospy.loginfo('Replan number: %d' % self.replan_num)


if __name__ == '__main__':
    n = HomoCoPathNode()
    n.spin()
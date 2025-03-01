################################################################
# 
# Author: Mike Chen, Hauser Dong 
# From Peking university
# Last update: 2023.5.17
# 
################################################################



import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosOcp
from casadi import SX, vertcat
import casadi as ca
import scipy.linalg
import copy
import sys
import os
from math import sqrt, isnan
import rospy
import time
from scipy.interpolate import BSpline, make_interp_spline

sys.path.append('../../')
from geometry import *
import SET
# import generate_homotopic_path
from enum import Enum

class omnidirection():

    # initialization
    def __init__(self,
        index,
        K,
        h,
        ini_p,
        target,
        shape,
        Vxmax,
        Vymax,
        buffer,
        Umax,
        obs_env_idx):

        ##############################
        ####### key cofficient #######
        ##############################

        self.index=index

        # the length of horizon
        self.K=K

        self.h=h

        self.Vxmax=Vxmax

        self.Vymax=Vymax

        self.Umax=Umax

        self.buffer=buffer

        self.shape=shape

        self.r_max=sqrt(self.Vxmax**2+self.Vymax**2)*self.h*self.K

        if len(shape)==1:
            self.circle=True
        else:
            self.circle=False
            self.shape=shape.reshape(int(len(shape)/2),2)

        self.buffer=buffer

        ExtendWidth = np.amax(self.shape) + 0.01
        self.obstacle_list, self.obstacle_list_nest = Build_ExtensionZone(SET.ini_obstacle_list[obs_env_idx],ExtendWidth)

        # load map
        # print(os.path.abspath('.'))
        # if SET.test_mode == 0:
        #     self.path_obstacle_list=np.loadtxt(os.path.abspath('.')+'/src/planner/scripts/map/forest_'+str(self.index)+'_'+str(obs_env_idx)+'.csv')   

        #####################
        ####### state #######
        #####################

        # initial position
        self.ini_p=ini_p.copy()

        # current position            
        self.p=ini_p.copy()

        self.v=np.zeros(2)

        # current state including position and velocity 
        self.state=np.block([self.p, self.v])


        ##########################
        ####### Trajectory #######
        ##########################

        # target position
        self.target=target.copy()

        # terminal position
        self.terminal_p=self.p.copy()

        # a coefficient related to the objective
        self.cost_index=K

        # the predetermined trajectory
        self.pre_traj=np.zeros((self.K+1,2))

        for i in range(self.K+1):
            self.pre_traj[i]=self.p.copy()

        self.acados_status = 0

        # get ABIT* path for safety corridor constructing, only getting once is enough
        # self.path=path_plan(self.path_obstacle_list,self.terminal_p,self.target)
        
        # if SET.test_mode == 0:
        #     # Hybrid A* path planning
        #     self.kino_path_finder = kino_astar.KinoAStar(self.path_obstacle_list,SET.map_range['x'],SET.map_range['y'],SET.resolution)
        
        ###############################
        ### Homotopic path planning ###
        ###############################

        self.explore_dist_max = 1.5

        self.Neighbor_path_dict = {}

        visualize = False
        # if self.index == 8:
        #     visualize = True
        # else:
        #     visualize = False
        
        # self.homopathplanner = generate_homotopic_path.HomoPathPlanner(self.index, self.obstacle_list_nest, SET.map_range, SET.resolution/5, SET.test_mode, visualize)

        # self.extend_visibility_check_res = self.homopathplanner.GetExtendedVisibilityCheck()

        # print extend_visibility_check_res
        # for i in range(len(self.extend_visibility_check_res)):
        #     passage = self.extend_visibility_check_res[i]
        #     pt1 = np.array(passage[0])
        #     pt2 = np.array(passage[1])
        #     print("Passage "+ str(i) + ":")
        #     print("pt1: ", pt1)
        #     print("pt2: ", pt2)
        #     print("length: ", np.linalg.norm(pt1-pt2)) 
            

        self.other_path_set = []
        
        self.other_path_idx = []   

        # self.path_plan()
        self.set_path(time.time())

        # tractive position list
        self.tractive_point = self.terminal_p.copy()    # for obstacle avoidance contraints
        self.tractive_lst = []
        self.tractive_time_now = None
        for i in range(K+1):
            tractive = np.block([self.terminal_p, np.zeros(2)])
            self.tractive_lst.append(tractive)
        self.tractive_vel = 0.0
        self.vel_lst = []
        self.aver_vel_lst = []
        self.err_lst = []
        self.aver_err_lst = []
        self.delta_t_lst = []
        self.aver_delta_t_lst = []

        self.tractive_block = False

        self.start_time = time.time()   # start getting new global path
        self.get_tractive_point()

        # self.first_ref_path = True

        # the list of all time's 
        self.pre_traj_list=[]

        # the list of all past position
        self.position=self.p.copy()


        #######################
        ####### Dealock #######
        #######################

        self.rho_0=20.0

        # warning band width
        self.epsilon=0.1

        self.eta=0.0

        self.term_overlap=False

        self.term_last_p=self.p.copy()

        self.term_overlap_again=False

        self.warning=None
        
        ###############################
        ####### data collection #######
        ###############################
        
        self.data=np.zeros(2)
        self.data[0:2]=np.array([0,1])
        self.data=np.block([[self.data],\
            [self.p],[-9999999*np.ones(2)]])

        ###############################
        ######### optimization ########
        ###############################

        self.setup_nlp()

        self.obs_cons = []

    # run convex program of each agents
    def trajectory_planning(self,Inter_cons,Ob_cons):

        ####### dynamic related #######

        state=self.state
        # tractive=self.tractive.copy()
        # tractive_vel = self.tractive_vel
        # print("tractive:",tractive)
        # buffer=self.buffer
        K=self.K
        next_state=self.next_state

        ####### functional constraints related #######
        inter_cons = Inter_cons[0]
        inter_cons_T = Inter_cons[1]
        ob_cons = Ob_cons
        self.obs_cons = ob_cons


        C_list=[np.zeros((10,4)) for _ in range(K)]
        lg_list=[-np.ones(10) for _ in range(K)]
        count_list=[0 for _ in range(K)]

        from inter_aviodance import compress
        total_cons = compress(inter_cons+ob_cons,self.K)
        # print("Const:",total_cons)

        for cons in total_cons:
            k=cons[0]
            count=count_list[k]
            if count<10:
                C_list[k][count]=np.block([cons[1],np.zeros(2)])
                lg_list[k][count]=cons[2] 
                count_list[k]+=1


        # dir=tractive[0:2]-next_state[K][0:2]
        # dir/=np.linalg.norm(dir)

        # theta=np.pi/2
        # R = np.array([
        #     [np.cos(theta), -np.sin(theta)],
        #     [np.sin(theta), np.cos(theta)]
        # ])

        # delta_yref=np.zeros(2)
        # for cons in inter_cons_T:

        #     outside=cons[1] @ next_state[K][0:2]-cons[2]

        #     if outside<self.buffer:
                
        #         sin=dir[0]*cons[1][1]-dir[1]*cons[1][0]

        #         dis=0.003/(0.001+max(0,outside))-0.003/(0.001+self.buffer)

        #         delta_yref -= 2.71828**sin*dis* R @ dir

        tractive_lst = self.tractive_lst.copy()

        theta=np.pi/2
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])

        delta_yref=np.zeros(4)
        
        dir = tractive_lst[K][0:2] - next_state[K][0:2]

        if np.linalg.norm(dir) > 1e-2:
            dir /= np.linalg.norm(dir)

            for cons in inter_cons_T:

                outside=cons[1] @ next_state[K][0:2]-cons[2]

                if outside<self.buffer:
                    
                    sin=dir[0]*cons[1][1]-dir[1]*cons[1][0]

                    dis=0.003/(0.001+max(0,outside))-0.003/(0.001+self.buffer)

                    delta_yref[0:2] -= 2.71828**sin*dis* R @ dir

            tractive_lst[K] += delta_yref
        else:
            if self.path is not None and len(self.path) > 2 and np.linalg.norm(self.p - self.path[-1]) < 1e-2:
                dir = self.path[-1] - self.path[-2]
                dir /= np.linalg.norm(dir)

                for cons in inter_cons_T:

                    outside=cons[1] @ next_state[K][0:2]-cons[2]

                    if outside<self.buffer:
                        
                        sin=dir[0]*cons[1][1]-dir[1]*cons[1][0]

                        dis=0.003/(0.001+max(0,outside))-0.003/(0.001+self.buffer)

                        delta_yref[0:2] -= 2.71828**sin*dis* R @ dir

                tractive_lst[K] += delta_yref
            
        ##############################
        ###### nolinear program ######
        ##############################

        # tractive[0:2]+=delta_yref

        # print("delta_yref is: "+str(delta_yref))

        # for i in range(K+1):
        #     p = np.block([tractive_lst[i], 10.0])
        #     self.acados_ocp_solver.set(i,"p",p)

        # # LINEAR_LS type cost objective
        # for i in range(K,0,-1):
        #     if (next_state[i][0:2]-tractive) @ (self.Q @ (next_state[i][0:2]-tractive))>0.1:
        #         break
        # cost_index=i

        # W1 = scipy.linalg.block_diag(self.Q,self.R)
        # W2 = scipy.linalg.block_diag(10*self.Q,self.R)

        # yref=np.block([tractive,np.zeros(2)])
        
        # for i in range(1,cost_index):
        #     self.acados_ocp_solver.cost_set(i,"yref",yref)
        #     self.acados_ocp_solver.cost_set(i,"W",W1)

        # for i in range(cost_index,K):
        #     self.acados_ocp_solver.cost_set(i,"yref",yref)
        #     self.acados_ocp_solver.cost_set(i,"W",W2)

        # self.acados_ocp_solver.cost_set(K,"yref",tractive)
        # self.acados_ocp_solver.cost_set(K,"W",10*self.Q)

        W = scipy.linalg.block_diag(10*self.Q,self.R)
        for i in range(K):
            yref=np.block([tractive_lst[i][0:2],np.zeros(2)])
            self.acados_ocp_solver.cost_set(i,"yref",yref)
            self.acados_ocp_solver.cost_set(i,"W",W)
        
        yref=tractive_lst[K][0:2]
        self.acados_ocp_solver.cost_set(K,"yref",yref)
        self.acados_ocp_solver.cost_set(K,"W",10*self.Q)

   
        # collision aviodance constriants
        if self.circle:
            for k in range(K):
                self.acados_ocp_solver.constraints_set(k+1,"C",C_list[k],api='new')
                self.acados_ocp_solver.constraints_set(k+1,"lg",lg_list[k])
                self.acados_ocp_solver.constraints_set(k+1,"ug",1e10*np.ones(10))
        else:
            pass
            # for S in self.S_list:
            #     cos_S=S[0]
            #     sin_S=S[1]
            #     opti.subject_to(buffer-buff+cons_B<=cons_A @ ( self.Phi @ opt_states + cos_S @ ca.cos(theta) + sin_S @ ca.sin(theta) ))

        # init_condition constraints
        self.acados_ocp_solver.set(0,"lbx",state)
        self.acados_ocp_solver.set(0,"ubx",state)

        # initial guess
        for k in range(K + 1):
            self.acados_ocp_solver.set(k, "x", self.next_state[k].copy())
        for k in range(K):
            self.acados_ocp_solver.set(k, "u", self.u0[k].copy())

        # solve
        status = self.acados_ocp_solver.solve()
        # self.acados_ocp_solver.print_statistics()
        print("cost:",self.acados_ocp_solver.get_cost())
        self.acados_status = status
        if status in [0,2]:
            opt_states=np.zeros((K+1,4))
            opt_controls=np.zeros((K,2))
            for k in range(K + 1):
                opt_states[k] = self.acados_ocp_solver.get(k, "x")
            for k in range(K):
                opt_controls[k] = self.acados_ocp_solver.get(k, "u")
        else:
            opt_states=self.next_state.copy()
            opt_controls=self.u0.copy()
            self.acados_ocp_solver.reset()

        self.cache = [opt_states,opt_controls]


    def setup_nlp(self):

        self.Q = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.R = np.array([[1.0, 0.0], [0.0, 1.0]])

        self.next_state=np.zeros((self.K+1, 4))
        for i in range(self.K+1):
            self.next_state[i]=self.state.copy()
        

        self.u0=np.zeros((self.K, 2))


        if not self.circle:
            self.S_list=[]
            for s in self.shape:
                cos_S=np.zeros((2*self.K,self.K))
                sin_S=np.zeros((2*self.K,self.K))
                for i in range(self.K):
                    cos_S[2*i,i]=s[0]
                    cos_S[2*i+1,i]=s[1]
                    sin_S[2*i,i]=-s[1]
                    sin_S[2*i+1,i]=s[0]
            
                self.S_list+=[[cos_S,sin_S]]
        
        # # create solvers
        ocp = self.create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            ocp, json_file="acados_code/agent"+str(self.index)+ "/configure.json"
        )


    @staticmethod
    def export_robot_model(index) -> AcadosModel:

        model_name = "agent"+str(index)

        # set up states & controls
        x = SX.sym("x")
        y = SX.sym("y")
        vx = SX.sym("vx")
        vy = SX.sym("vy")
        x = vertcat(x, y, vx, vy)

        ax = SX.sym("ax")
        ay = SX.sym("ay")
        u = vertcat(ax, ay)

        # xdot
        x_dot = SX.sym("x_dot")
        y_dot = SX.sym("y_dot")
        vx_dot = SX.sym("vx_dot")
        vy_dot = SX.sym("vy_dot")
        xdot = vertcat(x_dot, y_dot, vx_dot, vy_dot)

        # dynamics
        f_expl = vertcat(vx, vy, ax, ay)
        f_impl = xdot - f_expl

        # tractive point
        x_tractive = SX.sym("x_tractive")
        y_tractive = SX.sym("y_tractive")
        vx_tractive = SX.sym("vx_tractive")
        vy_tractive = SX.sym("vy_tractive")
        weight = SX.sym("weight")
        tractive = vertcat(x_tractive, y_tractive, vx_tractive, vy_tractive, weight)

        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        # model.p = tractive
        model.name = model_name

        return model   


    def create_ocp_solver_description(self) -> AcadosOcp:

        # create ocp object to formulate the OCP
        ocp = AcadosOcp()

        model = self.export_robot_model(self.index)
        ocp.model = model

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        ocp.dims.nx = nx
        ocp.dims.nu = nu
        ocp.dims.ny = ny

        # set cost
        # ocp.cost.cost_type = "EXTERNAL"
        # ocp.cost.cost_type_e = "EXTERNAL"

        # ocp.parameter_values = np.block([self.terminal_p, np.zeros(2),1.0])

        # # Q_terminal = self.Q.copy()
        # # Q_terminal[2:4,2:4] = np.zeros((2,2))
        # R = np.zeros((nu,nu))
        # R[:2,:2] = self.R

        # ocp.model.cost_expr_ext_cost = model.p[4] * (model.x - model.p[0:4]).T @ self.Q @ (model.x - model.p[0:4]) + model.u.T @ R @ model.u
        # ocp.model.cost_expr_ext_cost_e = 10 * (model.x - model.p[0:4]).T @ self.Q @ (model.x - model.p[0:4])

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        Vx = np.zeros((4,4))
        Vx[0:2,0:2] = np.eye(2)
        Vu = np.zeros((4,2))
        Vu[2:4,:] = np.eye(2)
        ocp.cost.Vx = Vx 
        ocp.cost.Vu = Vu
        ocp.cost.W = np.eye(4)
        ocp.cost.yref = np.zeros(4)

        ocp.cost.Vx_e = np.zeros((2,4))
        ocp.cost.Vx_e[0:2,0:2] = np.eye(2)
        ocp.cost.W_e = np.eye(2)
        ocp.cost.yref_e = np.zeros(2)



        # set constraints
        ocp.constraints.x0 = self.state

        ocp.constraints.lbu = -np.array([self.Umax,self.Umax])
        ocp.constraints.ubu = np.array([self.Umax,self.Umax])
        ocp.constraints.idxbu = np.array([0,1])

        ocp.constraints.lbx = -np.array([self.Vxmax,self.Vymax])
        ocp.constraints.ubx = np.array([self.Vxmax,self.Vymax])
        ocp.constraints.idxbx = np.array([2,3])     # the velocity in state is bounded
        
        # g_low <= D u + C x <= g_up
        ocp.constraints.C = np.zeros((10,4))
        ocp.constraints.C_e = np.zeros((10,4))
        ocp.constraints.D = np.zeros((10,2))

        ocp.constraints.lg = np.zeros(10)
        ocp.constraints.ug = np.zeros(10)
        ocp.constraints.lg_e = np.zeros(10)
        ocp.constraints.ug_e = np.zeros(10)

        # set options
        ocp.solver_options.N_horizon = self.K
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = "EXACT"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 50
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.solver_options.print_level = 0
        ocp.solver_options.tol = 1e-4

        # set prediction horizon
        ocp.solver_options.tf = self.K*self.h

        # export directory
        ocp.code_export_directory = "acados_code/agent"+str(self.index)

        return ocp


    def post_processing(self,vadility):

        self.output=self.cache[0]
        self.input=self.cache[1]

        # get predeterminted trajectory
        self.pre_traj=self.output[:,0:2]

        self.terminal_p = self.pre_traj[-1].copy()
        print("terminal position:",self.terminal_p)

        # get the execution trajectory
        self.traj=self.get_traj()

        sumv = 0
        sumerr = 0
        for i in range(len(self.output)):
            # print("state "+str(i)+": ", self.output[i])
            # print("velocity "+str(i)+": ", np.linalg.norm(self.output[i][2:4]))
            sumv += np.linalg.norm(self.output[i][2:4])
            tract = self.tractive_lst[i][0:2]
            # print("tractive "+str(i)+": ", tract)
            traj = np.array([self.traj[0][i], self.traj[1][i]])
            # print("error "+str(i)+": ", np.linalg.norm(tract-traj))
            sumerr += np.linalg.norm(tract-traj)
        # averv = sumv/len(self.output)
        # print("average velocity: ", averv)
        # print("average error: ", sumerr/len(self.output))

        # # check the terminal point safety
        # print("len of pre traj: ", len(self.pre_traj))
        # print("obs_cons len: ", len(self.obs_cons))
        # for i in range(len(self.obs_cons)):
        #     print("k : ", self.obs_cons[i][0])
        #     if self.obs_cons[i][0] == self.K-1:
        #         a = self.obs_cons[i][1]
        #         b = self.obs_cons[i][2]
        #         print(str(i)+ "th obstacle: ", a @ self.terminal_p - b)

        # if averv > 1e-2:
        #     self.vel_lst.append(averv)
        # if self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2:
        #     # paint the vel_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.vel_lst)
        #     # plot average velocity
        #     plt.axhline(y=np.mean(self.vel_lst), color='r', linestyle='--')
        #     plt.show()

        # get new state
        self.p=get_sample(P=self.pre_traj,h=self.h,t=vadility)
        self.v=get_sample(P=self.output[:,2:4],h=self.h,t=vadility)#*0.9
        self.state=np.block([self.p,self.v])
        
        # vel = np.linalg.norm(self.v)
        # print("velocity: ", vel)
        # if vel > 1e-2:
        #     self.vel_lst.append(vel)

        # if self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2:
        #     # paint the vel_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.vel_lst)
        #     plt.show()

        # update eta for deadlock resolution
        # self.update_eta()

        self.check_term_overlap()
        
        # data collection
        self.data=np.block([[self.data],[self.p],\
        [-7777777*np.ones(2)],[self.pre_traj],[-9999999*np.ones(2)]])

        next_state=np.zeros((self.K+1,4))
        self.u0=np.zeros((self.K,2))

        # after calculation time + waiting time, calculate the next state
        for i,t in zip(range(self.K+1),vadility+self.h*np.arange(self.K+1)):
            next_state[i]=get_sample(self.output,self.h,t)

        for i, t in zip(range(self.K),vadility+self.h*np.arange(self.K)):
            self.u0[i]=get_sample(self.input,self.h,t)

        self.next_state = next_state

        # t_1 = time.time()
        # self.get_tractive_point(vadility)
        # print("Time for get_tractive_point: ", time.time()-t_1)
        
        ##############################
        ########## velocity ##########
        ##############################
        v_next = self.v
        # print("velocity: ", np.linalg.norm(v_next))
        v = np.linalg.norm(v_next)
        if v > 0.01:
            self.vel_lst.append(v)
        # if (self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2 and self.index==5) or (np.linalg.norm(self.v) < 0.01 and self.path is not None and self.index==5):
        #     # paint the vel_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.vel_lst)
        #     # plot average velocity
        #     print("average velocity: ", np.mean(self.vel_lst))
        #     plt.axhline(y=np.mean(self.vel_lst), color='r', linestyle='--')
        #     plt.xlabel("iteration")
        #     plt.ylabel("velocity (m/s)")
        #     plt.show()

        #     import pickle
        #     with open('vel_only_obs.pkl', 'wb') as f:
        #         pickle.dump(self.vel_lst, f)
            
        
        aver_vel = -1
        if len(self.vel_lst) > 0:
            aver_vel = np.mean(self.vel_lst)
            if aver_vel > 0.01 and v > 0.01:
                # print("average velocity: ", aver_vel)
                self.aver_vel_lst.append(aver_vel)
        # if (self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2 and self.index==1) or (np.linalg.norm(self.v) < 0.01 and self.path is not None and self.index==1):
        #     # paint the vel_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.aver_vel_lst)
        #     plt.xlabel("iteration")
        #     plt.ylabel("aver velocity (m/s)")
        #     plt.show()
        
        ##############################
        ############ error ###########
        ##############################
        err = None
        if self.path is not None and self.tractive_time_now is not None:
            tractive_next_t = self.tractive_time_now + 2 * vadility - self.path_follow_start_time
            tractive_next = self.get_tractive_from_path(tractive_next_t)
            err = np.linalg.norm(tractive_next[0:2]-self.p)
            self.err_lst.append(err)

            self.tractive_block = detect_line_collision(self.obstacle_list, line(tractive_next[0:2], self.p))

        # if (self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2 and self.index==4) or (np.linalg.norm(self.v) < 0.01 and self.path is not None and self.index==4):
        #     # paint the err_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.err_lst)
        #     plt.axhline(y=np.mean(self.err_lst), color='r', linestyle='--')
        #     plt.xlabel("iteration")
        #     plt.ylabel("error (m)")
        #     plt.show()

        aver_err = -1
        if len(self.err_lst) > 0:
            aver_err = np.mean(self.err_lst)
            if aver_err > 1e-3:
                # print("average error: ", aver_err)
                self.aver_err_lst.append(aver_err)
        # if (self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2 and self.index==5) or (np.linalg.norm(self.v) < 0.01 and self.path is not None and self.index==5):
        #     # paint the err_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.aver_err_lst)
        #     plt.xlabel("iteration")
        #     plt.ylabel("aver error (m)")
        #     plt.show()
        
        ##############################
        ######### delta time #########
        ##############################
        if v > 0.01 and err is not None:
            delta_t = err / v
            self.delta_t_lst.append(delta_t)
        # if (self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2 and self.index==4) or (np.linalg.norm(self.v) < 0.01 and self.path is not None and self.index==4):
        #     # paint the delta_t_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.delta_t_lst)
        #     plt.ylim(0,1.0)
        #     plt.xlabel("iteration")
        #     plt.ylabel("delta time (s)")
        #     plt.show()

        if aver_vel > 0.01 and aver_err > 1e-3:
            aver_delta_t = aver_err / aver_vel
            self.aver_delta_t_lst.append(aver_delta_t)
        # if (self.path is not None and np.linalg.norm(self.p - self.path[-1]) < 1e-2 and self.index==5) or (np.linalg.norm(self.v) < 0.01 and self.path is not None and self.index==5):
        #     # paint the delta_t_lst
        #     import matplotlib.pyplot as plt
        #     plt.plot(self.aver_delta_t_lst)
        #     plt.xlabel("iteration")
        #     plt.ylabel("aver delta time (s)")
        #     plt.show()

        return None

    def check_term_overlap(self):

        term_p=self.pre_traj[-1].copy()
        tract_p=self.tractive_lst[-1][0:2].copy()
         
        term_second_p=self.pre_traj[-2].copy()

        if self.term_overlap:
            
            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.001
            condition_b=np.linalg.norm(term_p-term_second_p)<0.005
            condition_c=np.linalg.norm(term_p-tract_p)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap_again=True

        else:

            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.015
            condition_b=np.linalg.norm(term_p-term_second_p)<0.02
            condition_c=np.linalg.norm(term_p-tract_p)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap=True

        flag=False

        if np.linalg.norm(term_p-tract_p) < 0.05:
            flag = True
        
        if flag:
            self.term_overlap=False
            self.term_overlap_again=False
           
        self.term_last_p=term_p.copy()

    # transform the predetermined trajectory to the 7-th order polynomial
    def get_traj(self):    

        return [self.output[:,0],self.output[:,1]]


    # update eta for deadlock resolution
    def update_eta(self):

        term_p=self.pre_traj[-1].copy()
         
        term_second_p=self.pre_traj[-2].copy()

        if self.term_overlap:
            
            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.001
            condition_b=np.linalg.norm(term_p-term_second_p)<0.005
            condition_c=np.linalg.norm(term_p-self.tractive)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap_again=True

        else:

            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.015
            condition_b=np.linalg.norm(term_p-term_second_p)<0.02
            condition_c=np.linalg.norm(term_p-self.tractive)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap=True

        flag=False

        if self.warning is None:

            flag=True 
        else:
            if(type(self.warning) is np.ndarray):
                if (self.epsilon-self.warning < 1e-3).all():
                    flag=True
            elif self.epsilon-self.warning < 1e-3:
                    flag=True
        
        if flag:
            self.term_overlap=False
            self.term_overlap_again=False
            self.eta=0.0

        if self.term_overlap_again and self.eta < 4.0: 
            self.term_overlap_again=False
            self.eta += 0.3

        if self.term_overlap is True and self.eta <1.0:
            self.eta=1.0
           
        self.term_last_p=term_p.copy()

    # def set_target(self,target,other_passage_passing_time,V_aver):
        
    #     check_target_change_flag = not np.allclose(target[0:3],self.target[0:3],atol=0.1)
    #     if check_target_change_flag:
    #         self.target=target.copy()

    #         # self.other_path_set = []
    #         # self.other_path_idx = []

    #         # for i in range(len(other_passage_passing_time)):
    #         #     for j in range(len(other_passage_passing_time[i])):
    #         #         if other_passage_passing_time[i][j] > 0:
    #         #             print("!!!!Passage "+str(i)+": ", other_passage_passing_time[i][j])

    #         self.path_plan(other_passage_passing_time, V_aver)

    #         self.get_tractive_point()

    #         return True
    #     else:
    #         return False

    def set_path(self, path_start_time, path=None, path_vel=None, path_time=None):
        if path is None or path_vel is None or path_time is None:
            print("path is None or path_vel is None or path_time is None")
            self.path = None
            self.path_vel = None
            self.path_time = None
            self.path_start_time = path_start_time
        elif path is not None and path_vel is not None and path_time is not None and abs(path_start_time - self.path_start_time) > 0.01:
            print("==============Receive Path==================")
            print("path_start_time: ", path_start_time)
            print("self.path_start_time: ", self.path_start_time)
            self.path = path
            self.path_vel = path_vel
            self.path_time = path_time
            self.path_start_time = path_start_time
            self.path_follow_start_time = time.time()

            print("self.path_follow_start_time: ", self.path_follow_start_time)
            print("============================================")

            self.next_state=np.zeros((self.K+1, 4))
            for i in range(self.K+1):
                self.next_state[i]=self.state.copy()
            
            # self.vel_lst = []
            # self.aver_vel_lst = []
            # self.err_lst = []
            # self.aver_err_lst = []
            # self.delta_t_lst = []
            # self.aver_delta_t_lst = []
        else:
            print("==============Others==================")
            print("path_start_time: ", path_start_time)
            print("self.path_start_time: ", self.path_start_time)
            print("============================================")

            self.path_time = path_time



    def get_tractive_from_path(self, t):

        if np.linalg.norm(self.path[-1] - self.terminal_p) < 1.0:
            for i in range(len(self.path_time)-1):
                if self.path_time[i] <= t and t < self.path_time[i+1]:
                    t1 = self.path_time[i]
                    t2 = self.path_time[i+1]
                    p1 = self.path[i]
                    p2 = self.path[i+1]
                    tractive_point = p1 + (p2 - p1) * (t - t1) / (t2 - t1)
                    tractive_vel = np.zeros(2)

                    tractive_point = np.block([tractive_point, tractive_vel])
                    return tractive_point
            
            tractive_point = np.block([self.path[-1], np.zeros(2)])
            return tractive_point
        else:
            for i in range(len(self.path_time)-1):
                if self.path_time[i] <= t and t < self.path_time[i+1]:
                    t1 = self.path_time[i]
                    t2 = self.path_time[i+1]
                    p1 = self.path[i]
                    p2 = self.path[i+1]
                    v1 = self.path_vel[i]
                    v2 = self.path_vel[i+1]
                    tractive_point = p1 + (p2 - p1) * (t - t1) / (t2 - t1)
                    vel = v1 + (v2 - v1) * (t - t1) / (t2 - t1)
                    tractive_vel = vel * (p2 - p1) / np.linalg.norm(p2 - p1)

                    tractive_point = np.block([tractive_point, tractive_vel])
                    return tractive_point
            
            vel = self.path_vel[-1]
            tractive_vel = vel * (self.path[-1] - self.path[-2]) / np.linalg.norm(self.path[-1] - self.path[-2])
            tractive_point = np.block([self.path[-1], tractive_vel])
            return tractive_point

    # get the tractive point 
    def get_tractive_point(self, vadility = None):
            
        if self.path is None or self.path_vel is None or self.path_time is None or vadility is None:
            
            self.tractive_lst = []
            for i in range(self.K+1):
                tractive = np.block([self.terminal_p, np.zeros(2)])
                self.tractive_lst.append(tractive)
            self.tractive_point = self.terminal_p.copy()
        
        else:
            self.tractive_time_now = time.time()
            mpc_time_stamp_next = self.h*np.arange(self.K+1) + vadility
            mpc_time_stamp_next = mpc_time_stamp_next + self.tractive_time_now
            mpc_time_stamp = mpc_time_stamp_next - self.path_follow_start_time

            tractive_lst = []
            for i in range(len(mpc_time_stamp)):
                t = mpc_time_stamp[i]
                tractive_point = self.get_tractive_from_path(t)
                tractive_lst.append(tractive_point)
            
            self.tractive_lst = tractive_lst

            # last_tractive_point = tractive_lst[-1][0:2]

            # if not detect_line_collision(self.obstacle_list, line(self.terminal_p, last_tractive_point)):
            #     self.tractive_point = last_tractive_point
            # else:
            #     step = 0.1 * self.h
            #     tractive_time = mpc_time_stamp[-1]
            #     tractive_point_candidate = None

            #     while True:
            #         tractive_time -= step
            #         tractive_point_tmp = self.get_tractive_from_path(tractive_time)[0:2]
            #         if not detect_line_collision(self.obstacle_list, line(self.terminal_p, tractive_point_tmp)):
            #             tractive_point_candidate = tractive_point_tmp
            #             break
                
            #     if tractive_point_candidate is not None:
            #         self.tractive_point = tractive_point_candidate

    def path_plan(self,other_passage_passing_time=[], V_aver=-1):

        # plan predetermined path

        # if SET.test_mode != 0:
        #     # wait for higher priority agent to plan first
        #     while True:

        #         self.scan_homo_path()

        #         ready_to_plan = True

        #         for i in range(1,self.index):
        #             if i not in self.Neighbor_path_dict:
        #                 ready_to_plan = False
        #                 break
                    
        #             if self.Neighbor_path_dict[i].state == False:
        #                 ready_to_plan = False
        #                 break
                
        #         if ready_to_plan:
                    
        #             for i in range(1,self.index):
        #                 self.other_path_set.append(self.Neighbor_path_dict[i].path_set)
        #                 self.other_path_idx.append(self.Neighbor_path_dict[i].path_chosen_idx)

        #             break
                
        #         time.sleep(0.05)
            

        t_1 = time.time()
        
        # if SET.test_mode == 0:
        #     start = np.append(self.terminal_p.copy(),[0,0])
        #     end = np.append(self.target.copy(),[0,0])
        #     print("start:",start)
        #     print("end:",end)

        #     start = np.append(self.terminal_p.copy(),[0,0])
        #     end = np.append(self.target.copy(),[0,0])
        #     kino_stat = self.kino_path_finder.search(start,end,0)
        #     if kino_stat == Status.REACH_END.value:
        #         self.path = self.kino_path_finder.returnPath(0)
        #     else:
        #         self.path = None
        # else:

        start = self.terminal_p.copy()
        end = self.target.copy()
        print("start:",start)
        print("end:",end)

        if np.linalg.norm(start-end) < 0.05:
            self.path = np.expand_dims(self.target.copy(), axis=0)
            self.passage_passing_info = None
            return

        # (path_chosen_idx, path_set, passage_passing_info) = self.homopathplanner.generate_homotopic_path(start, end, other_passage_passing_time, V_aver)

        # if (path_chosen_idx >=0):
        #     self.path = np.array(path_set[path_chosen_idx])
        #     self.passage_passing_info = np.array(passage_passing_info[path_chosen_idx])

        #     # save the path to file
        #     # np.savetxt("path"+str(self.index),self.path)

        # else:
        #     self.path = None
        #     self.passage_passing_info = None
    
        # self.publish_homo_path(self.path)
            
        t_2 = time.time()
        print("Pre-trajectory planning time: ",t_2-t_1)

        # write the path to file
        # np.savetxt("path"+str(self.index),self.path)
    
    # def smooth_path_with_bspline(self, path, num_points = 100, degree = 3):
    #     if len(path) <= 3:
    #         return path

    #     points = np.array(path)

    #     x = points[:, 0]
    #     y = points[:, 1]

    #     n = len(points)
    #     k = degree
    #     t = np.linspace(0, 1, n-k+1)
    #     t = np.concatenate(([0]*k , t, [1]*k))

    #     spline_x = make_interp_spline(np.linspace(0,1,n), x, k=degree)
    #     spline_y = make_interp_spline(np.linspace(0,1,n), y, k=degree)

    #     new_t = np.linspace(0,1,num_points)
    #     smooth_x = spline_x(new_t)
    #     smooth_y = spline_y(new_t)

    #     smooth_points = np.vstack((smooth_x, smooth_y)).T

    #     return smooth_points.tolist()

        

    def interpolate_path(self,path):
        path_return = []

        for i in range(len(path)-1):
            path_return.append(path[i])

            start = np.array(path[i])
            end = np.array(path[i+1])
            dist = np.linalg.norm(start-end)
            dir = (end - start)/dist
            
            step_size = 2*SET.resolution
            num = int(np.floor(dist/step_size))

            for j in range(1,num+1):
                interp = start + dir * j * step_size
                path_return.append(interp.tolist())


        if len(path) > 0:
            path_return.append(path[-1])

        return path_return

def get_sample(P,h,t):

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
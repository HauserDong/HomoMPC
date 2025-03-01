################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.5.24
# 
################################################################



from sre_parse import State
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosOcp
from casadi import SX, vertcat, sin, cos
import scipy.linalg
import sys
import os
from math import sqrt,atan2

sys.path.append('../../')
from geometry import *
import SET
import kino_astar
from enum import Enum

class Status(Enum):
    REACH_HORIZON = 1
    REACH_END = 2
    NO_PATH = 3
    REACH_END_BUT_SHOT_FAILS = 4

class unicycle():

    # initialization
    def __init__(self,
        index,
        K,
        h,
        ini_state,
        target,
        Vmax,
        Amax,
        Omega_max,
        shape,
        buffer
        ):

        ##############################
        ####### key cofficient #######
        ##############################

        self.index=index

        # the length of horizon
        self.K=K

        self.h=h

        self.Vmax=Vmax

        self.Amax=Amax

        self.Omega_max=Omega_max

        self.shape=shape

        self.r_max=sqrt(self.Vmax**2+self.Vmax**2)*self.h*self.K    # only for deleting obstacles far away

        if len(shape)==1:
            self.circle=True
        else:
            self.circle=False
            self.shape=shape.reshape(int(len(shape)/2),2)

        self.buffer=buffer

        ExtendWidth = np.amax(self.shape)
        self.obstacle_list = Build_ExtensionZone(SET.ini_obstacle_list,ExtendWidth)

        # load map
        self.path_obstacle_list=np.loadtxt(os.path.abspath('.')+'/src/planner/scripts/map/forest_'+str(self.index)+'_1.csv')
        
        #####################
        ####### state #######
        #####################

        # initial postion
        self.ini_p=ini_state[0:2].copy()

        # position
        self.p=ini_state[0:2].copy()

        # velocity
        self.v=0.0

        # current state         
        self.state=np.append(ini_state,self.v) 

        ##########################
        ####### Trajectory #######
        ##########################

        # target position
        self.target=target.copy()

        # terminal position
        self.terminal_p=self.p.copy()

        # terminal state
        self.terminal_state = ini_state

        # the predetermined trajectory
        self.pre_traj=np.zeros((self.K+1,2))

        for i in range(self.K+1):
            self.pre_traj[i]=self.p.copy()

        # get ABIT* path for safety corridor constructing, only getting once is enough
        # self.path=path_plan(self.path_obstacle_list,self.terminal_p,self.target[0:2])
        
        # Hybride A* path planning
        self.kino_path_finder = kino_astar.KinoAStar(self.path_obstacle_list,SET.map_range['x'],SET.map_range['y'],SET.resolution)
        self.path_plan()

        # tractive position
        self.tractive = ini_state

        self.get_tractive_point()

        self.arrive_at_target = True

        G_p=[]
        # the tractive position list for objective
        for i in range(0,self.K):
            G_p=np.append(G_p,self.tractive)
        self.G_p=G_p

        # the list of all time's 
        self.pre_traj_list=[]


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
        
        self.data=np.zeros(len(self.state))
        self.data=np.block([[self.data],\
            [self.state],[np.append(self.target,0.0)],[-9999999*np.ones(len(self.state))]])

        
        ###############################
        ######### optimization ########
        ###############################

        self.setup_nlp()


    # run convex program of each agents
    def trajectory_planning(self,Inter_cons,Ob_cons):

        ####### dynamic related #######

        state=self.state
        tractive=self.tractive.copy()
        # buffer=self.buffer
        K=self.K
        next_state=self.next_state

        ####### functional constraints related #######
        inter_cons = Inter_cons[0]
        inter_cons_T = Inter_cons[1]
        ob_cons = Ob_cons


        C_list=[np.zeros((10,4)) for _ in range(K)]
        lg_list=[-np.ones(10) for _ in range(K)]
        count_list=[0 for _ in range(K)]

        delta_yref=np.zeros((K,3))
        # print(ob_cons)

        from inter_aviodance import compress
        total_cons = compress(inter_cons+ob_cons,self.K)
        # print(total_cons)

        for cons in total_cons:
            k=cons[0]
            count=count_list[k]
            if count<10:

                C_list[k][count]=np.block([cons[1],np.zeros(2)])
                lg_list[k][count]=cons[2] 
                count_list[k]+=1

        dir=tractive[0:2]-next_state[K][0:2]
        dir/=np.linalg.norm(dir)

        delta_yref=np.zeros(2)
        for cons in inter_cons_T:

            outside=cons[1] @ next_state[K][0:2]-cons[2]

            if outside<self.buffer:

                sin=dir[0]*cons[1][1]-dir[1]*cons[1][0]

                dis=0.004/(0.001+max(0,outside))-0.004/(0.001+self.buffer)
                delta_yref-=sin*dis*cons[1]
        
        
        ##############################
        ###### nolinear program ######
        ##############################


        # objective
        tractive[0:2]+=delta_yref
        print("delta_yref is: "+str(delta_yref))

        for i in range(K,0,-1):
            if (next_state[i][0:3]-tractive) @ (self.Q @ (next_state[i][0:3]-tractive))>0.5:
                break
        cost_index=i

        W1 = scipy.linalg.block_diag(self.Q,self.R)
        W2 = scipy.linalg.block_diag(10*self.Q,self.R)
        
        yref=np.block([tractive,np.zeros(2)])

        for i in range(1,cost_index):
            self.acados_ocp_solver.cost_set(i,"yref",yref)
            self.acados_ocp_solver.cost_set(i,"W",W1)

        for i in range(cost_index,K):
            self.acados_ocp_solver.cost_set(i,"yref",yref)
            self.acados_ocp_solver.cost_set(i,"W",W2)

        self.acados_ocp_solver.cost_set(K,"yref",tractive)
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

        self.Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, .1]])
        self.R = np.array([[0.5, 0.0], [0.0, 2.0]])

        self.next_state=np.zeros((self.K+1, 4))
        for i in range(self.K+1):
            self.next_state[i]=self.state.copy()
        
        self.u0=np.zeros((self.K, 2))


        if not self.circle:
            self.S_list=[]
            for s in self.shape:
            # for s in np.array([3.6,0.7,3.6,-0.7,-0.5,-0.7,-0.5,0.7]).reshape((4,2)):
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
        theta = SX.sym("theta")
        v = SX.sym("v")
        x = vertcat(x, y, theta, v)

        a = SX.sym("a")
        omega = SX.sym("omega")
        u = vertcat(a, omega)

        # xdot
        x_dot = SX.sym("x_dot")
        y_dot = SX.sym("y_dot")
        theta_dot = SX.sym("theta_dot")
        v_dot = SX.sym("v_dot")
        xdot = vertcat(x_dot, y_dot, theta_dot, v_dot)

        # dynamics
        f_expl = vertcat(v * cos(theta), v * sin(theta), omega, a)
        f_impl = xdot - f_expl

        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = model_name

        return model   


    def create_ocp_solver_description(self) -> AcadosOcp:

        # create ocp object to formulate the OCP
        ocp = AcadosOcp()

        model = self.export_robot_model(self.index)
        ocp.model = model

        # set dimensions
        ocp.dims.N = self.K

        # set cost
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        Vx = np.zeros((5,4))
        Vx[0:3,0:3] = np.eye(3)
        Vu = np.zeros((5,2))
        Vu[3:5,:] = np.eye(2)
        ocp.cost.Vx = Vx 
        ocp.cost.Vu = Vu
        ocp.cost.W = np.eye(5)
        ocp.cost.yref = np.zeros(5)

        Vx_e = np.zeros((3,4))
        Vx_e[0:3,0:3] = np.eye(3)
        ocp.cost.Vx_e = Vx_e
        ocp.cost.W_e = np.eye(3)
        ocp.cost.yref_e = np.zeros(3)

        # set constraints
        ocp.constraints.x0 = self.state

        ocp.constraints.lbu = -np.array([self.Amax,self.Omega_max])
        ocp.constraints.ubu = np.array([self.Amax,self.Omega_max])
        ocp.constraints.idxbu = np.array([0,1])

        ocp.constraints.lbx = -np.array([0.0])  # cannot back a car
        ocp.constraints.ubx = np.array([self.Vmax])
        ocp.constraints.idxbx = np.array([3])

        ocp.constraints.C = np.zeros((10,4))
        ocp.constraints.C_e = np.zeros((10,4))
        ocp.constraints.D = np.zeros((10,2))

        ocp.constraints.lg = np.zeros(10)
        ocp.constraints.ug = np.zeros(10)
        ocp.constraints.lg_e = np.zeros(10)
        ocp.constraints.ug_e = np.zeros(10)

        # set options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = "EXACT"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 10
        ocp.solver_options.qp_solver_iter_max = 10
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
        # print("terminal_p:",self.terminal_p)

        self.terminal_state = self.output[:,0:3][-1].copy()
        print("terminal state:",self.terminal_state)

        # get the execution trajectory
        self.traj=self.get_traj()  

        # get new state
        self.state=get_sample(P=self.output,h=self.h,t=vadility)
        self.p = self.state[0:2]


        # update eta for deadlock resolution
        # self.update_eta()

        # data collection
        self.data=np.block([[self.data],[np.append(self.tractive,0.0)],[self.state],\
        [-7777777*np.ones(len(self.state))],[self.output],[-9999999*np.ones( len(self.state) )]])

        next_state=np.zeros((self.K+1,4))
        self.u0=np.zeros((self.K,2))

        for i,t in zip(range(self.K+1),vadility+self.h*np.arange(self.K+1)):
            next_state[i]=get_sample(self.output,self.h,t)

        for i, t in zip(range(self.K),vadility+self.h*np.arange(self.K)):
            self.u0[i]=get_sample(self.input,self.h,t)

        self.next_state = next_state

        self.get_tractive_point()

        return None


    # transform the predetermined trajectory to the 7-th order polynomial
    def get_traj(self):    

        return [self.output[:,0],self.output[:,1],self.output[:,2]]


    # update eta for deadlock resolution
    def update_eta(self):

        term_p=self.pre_traj[-1].copy()
         
        term_second_p=self.pre_traj[-2].copy()

        if self.term_overlap:
            
            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.001
            condition_b=np.linalg.norm(term_p-term_second_p)<0.005
            condition_c=np.linalg.norm(term_p-self.tractive[0:2])>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap_again=True

        else:

            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.015
            condition_b=np.linalg.norm(term_p-term_second_p)<0.02
            condition_c=np.linalg.norm(term_p-self.tractive[0:2])>0.02

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

    def set_target(self,target):

        check_target_change_flag = not np.allclose(target[0:3],self.target[0:3],atol=0.1)
        if check_target_change_flag:
            self.target=target.copy()
            self.path_plan()

    # get the list of tractive point which is used for tracting the agent to the tractive point 
    def get_tractive_point_list(self):

        G_p=self.tractive
        for i in range(1,self.K):
            G_p=np.append(G_p,self.tractive)
        self.G_p=G_p

        return None
    
    # get the tractive point 
    def get_tractive_point(self):

        obstacle_list=self.obstacle_list
            
        # if the path is None, i.e, the search based planning doesn't find a feasible path, the tractive point will be chosen as the terminal point of predetermined trajectory
        if self.path is None:
            self.tractive=self.terminal_state.copy() 
        
        else:

            flag_no_tractive = False

            self.tractive=self.terminal_state.copy()
            
            # if a collision-free path exists, then we can find the tractive point
            for i in range(len(self.path)-1,-2,-1):

                if i == -1:

                    flag_no_tractive = True

                if not detect_line_collision(obstacle_list,line(self.path[i][0:2],self.terminal_p[0:2])):

                    term_angle = self.terminal_state[2]
                    tract_angle = self.path[i][2]

                    # calculate the angle difference between the tractive angle and the terminal angle
                    cos_delta = np.cos(tract_angle)*np.cos(term_angle)+np.sin(tract_angle)*np.sin(term_angle)
                    sin_delta = np.sin(tract_angle)*np.cos(term_angle)-np.cos(tract_angle)*np.sin(term_angle)
                    new_angle = term_angle + np.arctan2(sin_delta,cos_delta)

                    self.tractive = np.array([self.path[i][0],self.path[i][1],new_angle])

                    break
            
            if flag_no_tractive:
                self.path_plan()
                self.tractive=self.terminal_state.copy() 
                self.get_tractive_point()


        # No matter whether path exists or doesn't, there needs a tractive list for convex programming    
        self.get_tractive_point_list()
        print("tractive:",self.tractive)
        
        return None
    
    def angle_diff(self, angle_a, angle_b):
        # input: two angle in the same 2 pi range, use process_angle function first
        # output: the real difference between those 2 angles

        diff = abs(angle_a - angle_b)
        if diff > np.pi:
            return 2 * np.pi - diff
        else:
            return diff

    def normalize_angle(self,angle):
        angle_tmp = angle
        # angle_tmp += (angle < -np.pi) * 2 * np.pi
        # angle_tmp -= (angle >= np.pi) * 2 * np.pi
        while True:
            if angle_tmp >= np.pi:
                angle_tmp -= 2*np.pi
            if angle_tmp < -np.pi:
                angle_tmp += 2*np.pi
            
            if angle_tmp <np.pi and angle_tmp >= -np.pi:
                break

        return angle_tmp

    def process_angle(self,angle_a,angle_b):
        # make angle_a in the same range with angle_b
        # eg. angle_b in [-3pi,-pi), angle_a in [-pi,pi), then this function helps to make angle_a change to [-3pi,-pi)

        sgn_a = 1 if angle_a > 0 else -1
        sgn_b = 1 if angle_b > 0 else -1

        b_range_idx = 1
        while True:
            if angle_b >= (b_range_idx-2)*np.pi and angle_b < b_range_idx*np.pi:
                break
            else:
                b_range_idx += sgn_b * 2
            
        angle_a_ = self.normalize_angle(angle_a)
        while True:
            if angle_a_ >= (b_range_idx-2)*np.pi and angle_a_ < b_range_idx*np.pi:
                break
            else:
                angle_a_ += sgn_b * 2 * np.pi
        
        return angle_a_


    def path_plan(self):
        # plan predetermined path
        
        t_1 = time.time()
        
        start = np.append(self.state.copy()[0:3],0)
        end = np.append(self.target.copy(),0)

        kino_stat = self.kino_path_finder.search(start,end,1)
        if kino_stat == Status.REACH_END.value:
            self.path = self.kino_path_finder.returnPath(1)
        else:
            self.path = None
        
        t_2 = time.time()
        print("Pre-trajectory planning time: ",t_2-t_1)

    
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
import os
import SET
import sys
import rospy

session_name = 'ASSP'

# os.system("tmux kill-session -t " + session_name)
# os.system('tmux kill-server')

os.system("tmux new -d -s "+session_name+" -x 500 -y 500")

# if len(sys.argv) == 1:
#     obs_env_dix = 0
# else:
#     obs_env_dix = int(sys.argv[1])
 
agent_list=SET.agent_list
Num=len(agent_list)

for i in range(Num):
    os.system('tmux split-window ; tmux select-layout tiled')

for i in range(Num):

    # os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "conda activate $conda-env$" C-m')

    # single simulation
    os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "source devel/setup.bash" C-m')
    index=agent_list[i]['index']

    # os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "python3 src/local_planner/planner/scripts/Planner.py '+str(index)+' '+str(obs_env_dix)+'" '+' C-m')
    
    obs_env_idx = rospy.get_param('obs_env_idx', default=0)
    os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "python3 src/local_planner/planner/scripts/Planner.py '+str(index)+' '+str(obs_env_idx)+'" '+' C-m')

    # random simulation
    # os.system('tmux send-keys -t '+session_name+':1.'+str(i) +' "source devel/setup.bash" C-m')
    # index=agent_list[i]['index']
    # os.system('tmux send-keys -t '+session_name+':1.'+str(i) +' "python3 src/planner/scripts/Planner.py '+str(index)+' '+str(obs_env_dix)+'" '+' C-m')

os.system('tmux attach -t '+session_name)
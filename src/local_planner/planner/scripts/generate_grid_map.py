#!/usr/bin/env python3

import os 
import glob

import SET
from geometry import *

if __name__ == "__main__":
    
    agent_list = SET.agent_list

    ini_obstacle_list = SET.ini_obstacle_list 

    resolution = SET.resolution

    map_range = SET.map_range 

    # delete old grid maps
    folder_path = 'src/planner/scripts/map/*.csv'
    files = glob.glob(folder_path)
    for f in files:
        os.remove(f)

    for agent in agent_list:
        print("Agent "+str(agent['index'])+":")
        
        # generating map
        ExtendWidth = np.amax(agent['radius'])+0.1
        print("ExtendWidth:",ExtendWidth)
        
        for i in range(len(ini_obstacle_list)):
        
            path_obstacle_list, _ =Build_ExtensionZone(ini_obstacle_list[i],ExtendWidth)

            path_obstacle_list=grid(path_obstacle_list,resolution,map_range)

            np.savetxt('src/planner/scripts/map/forest_'+str(agent['index'])+'_'+str(i)+'.csv',path_obstacle_list)

            print("Map "+str(i)+" is generated.")
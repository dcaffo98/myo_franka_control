#!/usr/bin/env python3

import roslaunch
import rospy
from constants import NODE_MYO_LAUNCH, PCKG_NAME
from utils import wait_gazebo
import sys
import os
import subprocess



if __name__ == '__main__':
    rospy.init_node(NODE_MYO_LAUNCH)
    basepath = subprocess.run(f"rospack find {PCKG_NAME}".split(), stdout=subprocess.PIPE).stdout[:-1].decode('utf-8') 
    fullpath = os.path.join(basepath, 'launch/myo_raw.launch')                                                               
    # sys.argv.append('/dev/ttyACM0')
    cli_args = [fullpath, 'port:=' + sys.argv[1]]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)                                                                
    roslaunch.configure_logging(uuid)                                                                                        
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)                                                          
    wait_gazebo()
    rospy.loginfo('Myo node - Gazebo (and MoveIt) awaited. Starting node...')
    launch.start()                                                     
    rospy.loginfo('Myo node started!')                               
    launch.spin()                                                      


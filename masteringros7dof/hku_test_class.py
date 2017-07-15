#!/usr/bin/env python

from copy import deepcopy
import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
#from geometry_msgs.msg import Pose


class MoveItDemoHKU:
    def __init__(self,node_name,move_group_name):

        self.node_name=node_name
        self.move_group_name=move_group_name

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node(self.node_name, anonymous=True)

        self.cartesian = rospy.get_param('~cartesian', True)
        
        # Connect to the right_arm move group
        self.right_arm = MoveGroupCommander(self.move_group_name)

        # Allow replanning to increase the odds of a solution
        self.right_arm.allow_replanning(True)

        print '\nset_pose_reference_frame\n'
        # Set the right arm reference frame
        self.right_arm.set_pose_reference_frame('base_link')

        print '\nset_goal_position_tolerance and orientation_tolerance\n'
        # Allow some leeway in position(meters) and orientation (radians)
        self.right_arm.set_goal_position_tolerance(0.01)
        self.right_arm.set_goal_orientation_tolerance(0.1)

        print '\nget the name of the end-effector link\n'
        # Get the name of the end-effector link
        self.end_effector_link = self.right_arm.get_end_effector_link()
        self.initial_joint_value=self.right_arm.get_current_joint_values()
        # Start in the "straight_forward" configuration stored in the SRDF file
#        right_arm.set_named_target('straight_forward')

        # Plan and execute a trajectory to the goal configuration
#        right_arm.go()
        #initial_pose=right_arm.get_current_pose(end_effector_link).pose
        print '\nprepare for forward kinematics\n'
        # BEND_ELBOW = [0.0, 0.1238, -0.452, 0.9418, 1.1729, 0.5562, 0.9672]
        BEND_ELBOW = [0.5654, 0.4546, 0, 0.3471, 0.0, 0.0, 0.0]
        self.right_arm.set_joint_value_target(BEND_ELBOW)
        self.right_arm.go()
        print '\ndone with forward kinematics\n'

        # Get the current pose so we can add it as a waypoint
        start_pose = self.right_arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints = []
                
        # Set the first waypoint to be the starting pose
        # if cartesian:
        #     # Append the pose to the waypoints list
        #     waypoints.append(start_pose) 
        self.wpose = deepcopy(start_pose)

    def one_move(self):
        if self.cartesian:
            # Append the pose to the waypoints list
            self.waypoints.append(deepcopy(self.wpose))
        else:
            self.right_arm.set_pose_target(self.wpose)
            self.right_arm.go()
            self.rospy.sleep(1)

    def cartesian_plan_execution(self):
        if self.cartesian:
            fraction = 0.0
            maxtries = 100
            attempts = 0
            
            # Set the internal state to the current state
            self.right_arm.set_start_state_to_current_state()
    
            # Plan the Cartesian path connecting the waypoints
            while fraction < 0.98 and attempts < maxtries:
                (plan, fraction) = self.right_arm.compute_cartesian_path (
                                        self.waypoints,   # waypoint poses
                                        0.01,        # eef_step
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions
                
                # Increment the number of attempts 
                attempts += 1
                
                # Print out a progress message
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                        
            # If we have a complete plan, execute the trajectory
            if fraction >=0.95:
                rospy.loginfo("Path computed successfully.fraction is "+str(fraction)+" Moving the arm.")
    
                self.right_arm.execute(plan)
                            
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

    def terminate(self):
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        hku=MoveItDemoHKU('hku_test_class_demo','arm')
    except rospy.ROSInterruptException:
        pass
    
    # wait for gazebo and rviz to be well prepared
    rospy.sleep(2)

    print 'preprare for the first letter H'
    hku.wpose.position.z -= 0.2
    hku.one_move()
    hku.wpose.position.z +=0.1
    hku.one_move()
    hku.wpose.position.y -=0.05
    hku.one_move()
    hku.wpose.position.z -=0.1
    hku.one_move()
    hku.wpose.position.z +=0.2
    hku.one_move()
    print 'moveing H'
    hku.cartesian_plan_execution()
    print 'done with the first letter H'


    print 'move to the start position of k'
    hku.wpose.position.y -=0.05
    hku.right_arm.set_pose_target(hku.wpose)
    hku.right_arm.go()
    rospy.sleep(2)
    print 'prepare for the second letter k'
    del hku.waypoints[:]
    start_pose=hku.right_arm.get_current_pose(hku.end_effector_link).pose
    hku.wpose=deepcopy(start_pose)
    hku.wpose.position.z -=0.2
    hku.one_move()
    hku.wpose.position.z +=0.1
    hku.one_move()
    hku.wpose.position.z +=0.1
    hku.wpose.position.y -=0.05
    hku.one_move()
    hku.wpose.position.z -=0.1
    hku.wpose.position.y +=0.05
    hku.one_move()
    hku.wpose.position.z -=0.1
    hku.wpose.position.y -=0.05
    hku.one_move()
    print "moving k"
    hku.cartesian_plan_execution()
    print 'done with the second letter K'

    print 'move to the start position of U'
    hku.wpose.position.y -=0.01
    hku.wpose.position.z +=0.2
    hku.right_arm.set_pose_target(hku.wpose)
    hku.right_arm.go()
    rospy.sleep(2)
    print 'prepare for the third letter U'
    del hku.waypoints[:]
    start_pose=hku.right_arm.get_current_pose(hku.end_effector_link).pose
    hku.wpose=deepcopy(start_pose)
    hku.wpose.position.z -=0.2
    hku.one_move()
    hku.wpose.position.y -=0.05
    hku.one_move()
    hku.wpose.position.z +=0.2
    hku.one_move()
    print 'moving U'
    hku.cartesian_plan_execution()
    print 'done with the third letter U'

    hku.right_arm.set_joint_value_target(hku.initial_joint_value)
    hku.right_arm.go()
    rospy.sleep(1)

    hku.terminate()
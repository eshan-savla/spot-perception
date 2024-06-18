from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import NavigateThroughPoses

class NavigateToPoseActionClient(Node):

    def __init__(self):
            super().__init__('navigate_to_pose')
            self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, position):
         
         goal_msg = NavigateToPose.Goal()
         pose = PoseStamped()
         pose.header.frame_id =  'map'
         pose.pose.position.x =  position[0]  # Set your desired X coordinate
         pose.pose.position.y = position[1]  # Set your desired Y coordinate
         goal_msg.pose = pose
         self._action_client.wait_for_server()
         return self._action_client.send_goal_async(goal_msg)


class NavigateThroughPosesActionClient(Node):

    def __init__(self):
        super().__init__('navigate_through_poses')
        self._action_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')  # Änderung des Action-Namens

    def send_positions(self, poses):
        goal_msg = NavigateThroughPoses.Goal()  # Änderung des Goal-Typs
        pose_list=[]
        for i in range(len(poses)):
            pose = PoseStamped()
            pose.header.frame_id =  'map'
            pose.pose.position.x = poses[i,0]  # Set your desired X coordinate
            pose.pose.position.y = poses[i,1]  # Set your desired Y coordinate
            pose_list.append(pose) 
        goal_msg = pose_list
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
             


def main(args=None):
    rclpy.init(args=args)
    position = [0.4, 1.8]   
    action_client = NavigateToPoseActionClient()
    future = action_client.send_goal(position)
    rclpy.spin_until_future_complete(action_client, future)


    #action_client = NavigateThroughPosesActionClient()
    #pos1 = [-1.68, -0.611 ]
    #pos2 = [-1.56, 1.411]
    #pos3 = [1.486, 1.637]
    #pos4 = [1.54, -1.70]
    #positions = np.array([pos1, pos2, pos3, pos4])
    #future= action_client.send_positions(positions)
    #rclpy.spin_until_future_complete(action_client, future)



    #if future.result() is not None:
     #   result = future.result().result
      #  print('Goal was successfully achieved!')
       # print('Result:', result)
    #else:
     #   print('Goal failed to complete')

    #action_client.destroy()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()




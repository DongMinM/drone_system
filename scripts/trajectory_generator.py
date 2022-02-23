<<<<<<< HEAD
=======
#!/usr/bin/python3
>>>>>>> main
import numpy as np
import rospy

from drone_system.msg import Status
<<<<<<< HEAD
from std_msgs.msg import String
=======
from std_msgs.msg import String,Float32MultiArray
>>>>>>> main

class TrajectoryGenerator:

    def __init__(self):
        
        rospy.init_node("trajectory_generator")
<<<<<<< HEAD
=======
        rospy.Subscriber("action_msgs", String, self.action_update)
        self.pub2motion_motion = rospy.Publisher("motion_msgs", String,queue_size=1)
        self.pub2motion_trajec = rospy.Publisher("trajec_msgs", Float32MultiArray,queue_size=1)
>>>>>>> main

        self.action = None


<<<<<<< HEAD

    def action_update(self,action_msgs):

        self.action = action_msgs
=======
    def action_update(self,action_msgs):

        self.action = action_msgs.data
>>>>>>> main



    def run(self):

<<<<<<< HEAD
        rospy.Subscriber("action_msgs", String, self.action_update)

        while not rospy.is_shutdown():
            
            pass
=======
        while not rospy.is_shutdown():
            if self.action != None:
                if self.action != "park" or "search":
                    
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    # if the mission is simple, pass the mission to motion controller
                    self.action = None


                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass


if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()
>>>>>>> main

#!/usr/bin/env python

"""
  Based on email conversation with Charel Van Hoof 
  and voice_nav.py - Version 1.1 2013-12-20
"""

import rospy
import actionlib
import datetime
from std_msgs.msg import String
from math import copysign
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sound_play.libsoundplay import SoundClient

class VoiceNav:
    #i = 0
    def __init__(self):
        rospy.init_node('voice_nav_move')
        
        rospy.on_shutdown(self.cleanup)

        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
 
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'stop': ['stop'],
                                    'living room': ['living'],
                                    'dining room': ['dining'],
                                    'kitchen': ['kitchen'],
                                    'bed room': ['bed'],
                                    'bath room': ['bath'],
                                    'introduce': ['introduce'],
                                    'duck': ['duck'],
                                    'thank': ['thank', 'thank you'],
                                    'yes': ['yes'],
                                    'no': ['no'],
                                    'ok': ['ok'],
                                    'many': ['many'],
                                    'one': ['one'] }

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                            'SUCCEEDED', 'ABORTED', 'REJECTED',
                            'PREEMPTING', 'RECALLING', 'RECALLED',
                            'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()
        
        self.locations['kitchen'] = Pose(Point(1.952, 0.533, 0.000), Quaternion(0.000, 0.000, 0.181, 0.983))
        self.locations['dining room'] = Pose(Point(-0.611, -0.242, 0.000), Quaternion(0.000, 0.000, 0.988, -0.157))
        self.locations['living room'] = Pose(Point(-7.362, 0.299, 0.000), Quaternion(0.000, 0.000, 0.990, -0.140))
        self.locations['bath room'] = Pose(Point(0.965, 2.661, 0.000), Quaternion(0.000, 0.000, -0.599, 0.801))
        self.locations['bed room'] = Pose(Point(-0.460, 2.378, 0.000), Quaternion(0.000, 0.000, 0.811, 0.585))


        #self.locationsl = list()
        
        #self.locationsl.append(Pose(Point(1.952, 0.533, 0.000), Quaternion(0.000, 0.000, 0.181, 0.983)))
        #self.locationsl.append(Pose(Point(-0.611, -0.242, 0.000), Quaternion(0.000, 0.000, 0.988, -0.157)))
        #self.locationsl.append(Pose(Point(-7.362, 0.299, 0.000), Quaternion(0.000, 0.000, 0.990, -0.140)))
        #self.locationsl.append(Pose(Point(0.965, 2.661, 0.000), Quaternion(0.000, 0.000, -0.599, 0.801)))
        #self.locationsl.append(Pose(Point(-0.460, 2.378, 0.000), Quaternion(0.000, 0.000, 0.811, 0.585)))

         # For Check
        self.paused = False
        self.state = 0
        self.memlocation = ""
        self.mode = 0
        self.loopy = True
        self.numlist_l = []
        self.places = []
        self.numlist_d = dict()
        self.l_location = "sssss"
        self.c_location = ""
        self.n_location = len(self.locations)
        self.g_location = ""
        self.i = 0
        self.value = ""

        rospy.loginfo("Ready to receive voice commands " + str(self.n_location))
        self.soundhandle.say("Hello Everyone")

    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speech_callback(self, msg):
        #dont react upon anything if speech recognition is paused
        #if self.paused:
        #   return
    
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
            #if command == 'living':
            #rospy.loginfo("Command: " + str(command))
            #value = str.split(str(command))
        #self.numlist.append(command)
        rospy.loginfo("Command: " + str(command))
        
        # Log the command to the screen
        #rospy.loginfo("Command: " + str(numlist))
        if self.state == 0:
            if command == 'thank':
                self.soundhandle.say("How many destination you want to go one or more", self.voice)
                self.state = 1
            elif command == 'introduce':
                self.soundhandle.say("My name is duck nice to meet you friend", self.voice)
        
        elif self.state == 1:
            #self.paused = False
            if command == 'one':
                #self.mode = 1
                self.state = 2
            elif command == 'many':
                self.state = 3

        elif self.state == 2:
            rospy.loginfo("come to state: "+ str(self.state))
            if command == 'kitchen':
                self.memlocation = command
                self.soundhandle.say("Are you going to " + self.memlocation + "Yes or No", self.voice)
                self.state = 4
            elif command == 'bath room':
                self.memlocation = command
                self.soundhandle.say("Are you going to " + self.memlocation + "Yes or No", self.voice)
                self.state = 4
            elif command == 'bed room':
                self.memlocation = command
                self.soundhandle.say("Are you going to " + self.memlocation + "Yes or No", self.voice)
                self.state = 4
            elif command == 'dining room':
                self.memlocation = command
                self.soundhandle.say("Are you going to " + self.memlocation + "Yes or No", self.voice)
                self.state = 4
            elif command == 'living room':
                self.memlocation = command
                self.soundhandle.say("Are you going to " + self.memlocation + "Yes or No", self.voice)
                self.state = 4
            else:
                self.move_base.cancel_all_goals()
                self.msg = Twist()
                self.cmd_vel_pub.publish(self.msg)
        
        elif self.state == 3:
                if command in self.locations or self.keywords_to_command:
                    self.numlist_l.append(command)
                    for findok in self.numlist_l:
                        if findok == 'ok':
                            for findoks in range(0 , len(numlist_l) - 1):
                                self.places = numlist_l[findoks]
                                
                                for place in self.places:

                                    self.goal = MoveBaseGoal()

                                    self.goal.target_pose.pose = self.locations[self.places]
                                    self.goal.target_pose.header.frame_id = 'map'
                                    self.goal.target_pose.header.stamp = rospy.Time.now()

                                    # Start the robot toward the next location
                                    self.move_base.send_goal(self.goal)

                                    # Allow 3 minutes to get there
                                    finished_within_time = self.move_base.wait_for_result(rospy.Duration(180))

                                    # Check for success or failure
                                    if not finished_within_time:
                                        self.move_base.cancel_goal()
                                        #rospy.loginfo("Timed out achieving goal")
                                        self.soundhandle.say("Timed out achieving goal", self.voice)
                                    else:
                                        state = self.move_base.get_state()
                                        if state == GoalStatus.SUCCEEDED:
                                            #rospy.loginfo("Goal succeeded!")
                                            self.soundhandle.say("Goal succeeded!", self.voice)
                                            #if findoks <= self.numlist_l:
                                            #    self.numlist_l = []
                                            #    self.state = 0
                                        else:
                                            rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
            
        elif self.state == 4:
            if command == 'yes':
                self.paused = True
                self.soundhandle.say("OK I'm going to " + self.memlocation, self.voice)
                self.paused = False
                    
                    # Set up the next goal location
                self.goal = MoveBaseGoal()

                self.goal.target_pose.pose = self.locations[self.memlocation]
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()

                    # Start the robot toward the next location
                self.move_base.send_goal(self.goal)

                    # Allow 3 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(180))

                    # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                        #rospy.loginfo("Timed out achieving goal")
                    self.soundhandle.say("Timed out achieving goal", self.voice)
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                            #rospy.loginfo("Goal succeeded!")
                        self.soundhandle.say("Goal succeeded!", self.voice)
                        self.memlocation = ""
                        self.state = 0
                    else:
                        rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

            elif command == 'no':
                self.soundhandle.say("Tell me where you want to go again Please", self.voice)
                rospy.sleep(3)
                self.memlocation = ""
                self.state = 1

        elif self.state == 5:
            rospy.loginfo("come to state: " + str(self.state))
        else:
            return

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")
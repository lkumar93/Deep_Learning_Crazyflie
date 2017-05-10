#
# THIS IS AN IMPLEMENTATION OF Q LEARNING FOR POSITION CONTROL OF QUADROTORS
#
# COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE
#
# AUTHOR : LAKSHMAN KUMAR
# AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
# EMAIL : LKUMAR93@UMD.EDU
# LINKEDIN : WWW.LINKEDIN.COM/IN/LAKSHMANKUMAR1993
#
# THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THE MIT LICENSE
# THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF
# THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
# 
# BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
# BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
# CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
# CONDITIONS.#

###########################################
##
##	LIBRARIES
##
###########################################

import pygame, sys, getopt, os
import rospy
import time
import traceback
import matplotlib.pyplot as plotter
import numpy

from QLearning import QLearner
from rospy.exceptions import ROSException
from deep_learning_crazyflie.srv import *
from geometry_msgs.msg import Twist

###########################################
##
##	VARIABLES
##
###########################################

FPS = 50
EPOCHS = (FPS*60*2) #2 Minutes
EPOCH_RANGE = range(0,EPOCHS)
#MODE = 'behavioral cloning'
MODE = 'reinforcement learning'
TRAIN_FLAG = True
SETPOINT_X = 0.0
SETPOINT_Y = 0.0
SETPOINT_Z = 0.25

###########################################
##
##	HELPER FUNCTIONS
##
###########################################

def init_controllers() :
	
	EPSILON = 0.0

	if TRAIN_FLAG is True :
		EPSILON = 0.0

	position_z_controller = QLearner(param = 'Z', mode = MODE, action_limits = [20000,55000], action_step_size = 175, epsilon = EPSILON, kp=5000, ki=3000, kd=6500, kp_range = [1000,10000], ki_range = [1000,6000], kd_range = [1000,10000])
	position_y_controller = QLearner(param = 'Y', mode = MODE, action_limits = [-15,15], action_step_size = 0.15, epsilon = EPSILON, kp=40, ki=3, kd=20.0, kp_range = [10,100], ki_range = [1,20], kd_range = [10,100])
	position_x_controller = QLearner(param = 'X', mode = MODE, action_limits = [-15,15], action_step_size = 0.15, epsilon = EPSILON, kp=40, ki=3, kd=20.0, kp_range = [10,100], ki_range = [1,20], kd_range = [10,100])

	return [position_z_controller,position_y_controller,position_x_controller]

def run(controllers, cmd_publisher) :

	cmd = Twist()

	for controller in controllers:

		if controller.param == 'Z' :

			cmd.linear.z = controller.run()
			print "Thrust = " + str(cmd.linear.z)

		elif controller.param == 'Y' :

			cmd.linear.y = controller.run()
			print "Roll = " + str(cmd.linear.y)

		elif controller.param == 'X' :

			cmd.linear.x = controller.run()
			print "Pitch = " + str(cmd.linear.x)

	cmd_publisher.publish(cmd)

def epsilon_decay(controllers) :
	
	for controller in controllers :

		controller.decrement_epsilon(EPOCHS)


def update(controllers) :
	
	reward = 0.0

	for controller in controllers:

		reward+=controller.update_policy()

	return reward


def randomize_target(controllers ,min_value ,max_value ,cmd_publisher) :

	cmd = Twist()

	cmd.angular.x = 1

	cmd.linear.z = round(numpy.random.uniform(0,max_value),max_value)
	cmd.linear.y = round(numpy.random.uniform(min_value,max_value),max_value)
	cmd.linear.x = round(numpy.random.uniform(min_value,max_value),max_value)

	for i in range(0,100) :
		cmd_publisher.publish(cmd)

def randomize_pid(controllers) :
	for controller in controllers:
		controller.randomize_pid()

def get_status() :

    rospy.wait_for_service('crazyflie/request_status')

    try:
        status_client = rospy.ServiceProxy('crazyflie/request_status', Status)
        return status_client(True).status

    except rospy.ServiceException, e:
        print "request_status service call failed: %s"%e
	return "ERROR"


###########################################
##
##	MAIN FUNCTION
##
###########################################

if __name__ == '__main__':

	#Initialize ros node
	rospy.init_node('crazyflie_qlearning_node')

	#Parse command line arguments to check if the user wants to enable training and whether to activate dqn or normal q learning
	argv = sys.argv[1:]

   	try:
      		opts, args = getopt.getopt(argv,"b:t:",["behavioral_cloning=","train="])

   	except getopt.GetoptError:	
      		print 'Usage: python Train.py -b <bool> -t <bool>'
      		sys.exit(2)
   	
	for opt, arg in opts:

     		if opt in ("-b", "--behavioral_cloning"):

			if arg == 'True' :
				MODE = 'behavioral cloning'

			elif arg == 'False' :
				MODE = 'reinforcement learning'

			else :
				print 'Usage: python Train.py -b <bool> -t <bool>'
			
				sys.exit(2)

		elif opt in ("-t", "--train") :

			if arg == 'True' :
				TRAIN_FLAG = True

			elif arg == 'False' :
				TRAIN_FLAG = False

			else :
				print 'Usage: python Train.py -b <bool> -t <bool>'

				sys.exit(2)


	if MODE == 'behavioral cloning' :

		#Initialize position controllers as Behavior Cloning Agents
		controllers = init_controllers()


	elif MODE == 'reinforcement learning':

		cmd_vel_topic = 'crazyflie/deep_learning/cmd_vel'

		cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 1)

		cmd_pos_topic = 'crazyflie/deep_learning/cmd_pos'

		cmd_pos_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 1)

		rate = rospy.Rate(FPS)

		rewards_per_episode = []
		count = 0

		#Initialize position controllers as Q Learning Agents
		controllers = init_controllers()

		print "initialized"
	
		#If the user wants to train
		if TRAIN_FLAG :
		
			for i in EPOCH_RANGE:

				status = get_status()
			
				if status == 'REINFORCEMENT LEARNING':

					total_reward_per_episode = 0.0			

					run(controllers,cmd_vel_publisher)

					rate.sleep()

					total_reward_per_episode += update(controllers)

					epsilon_decay(controllers)

					count += 1

					if count%100 == 0 :
						randomize_pid(controllers)						

					print " Count = " + str(count) +" ,Epsilon = " + str(controllers[0].epsilon)

					rewards_per_episode.append(total_reward_per_episode)

					print '\n \n \n rewards =' +str(total_reward_per_episode) + " ,epoch = "+str(i)	

				else :
					print status
					cmd = Twist()
					print "status is not REINFORCEMENT LEARNING"
					print "stopping the reinforcement learning module"

					for i in range(0,1000) :
						cmd.linear.z = 0.0
						cmd.linear.x = 0.0
						cmd.linear.y = 0.0
						cmd_vel_publisher.publish(cmd)

					break
						
			if count > 100 :

				plotter.figure()		

				plotter.plot(EPOCH_RANGE, rewards_per_episode,'g',label='Rewards' )

				plotter.xlabel('Episode')
				plotter.ylabel('Rewards')

				plotter.title('Learning Curve ')

				plotter.savefig('../figures/LearningCurve.png')

				plotter.show()

		else :
	
			count = 0
			states = []
			while not rospy.is_shutdown():

				count = count + 1
				run(controllers,cmd_publisher)
				rate.sleep()
				if count > 500 :
					break

	else :
		print "Unknown Mode : not available"
		

	rospy.spin()

	


		



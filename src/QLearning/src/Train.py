
#
# THIS IS AN IMPLEMENTATION OF DEEP Q LEARNING FOR POSITION CONTROL OF QUADROTORS
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

###########################################
##
##	VARIABLES
##
###########################################

FPS = 13
STEPS = FPS*15
EPOCHS = 10*(FPS*60*60)
STEP_RANGE = range(0,STEPS)
EPOCH_RANGE = range(0,EPOCHS)
MODE = 'behavioral cloning'
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
		EPSILON = 1.0

	position_z_controller = QLearner(param = 'Z',setpoint=SETPOINT_Z, mode = MODE, action_limits = [20000,55000], action_step_size = 175, epsilon = EPSILON)
	position_y_controller = QLearner(param = 'Y',setpoint=SETPOINT_Y, mode = MODE, action_limits = [-15,15], action_step_size = 0.15, epsilon = EPSILON)
	position_x_controller = QLearner(param = 'X',setpoint=SETPOINT_X, mode = MODE, action_limits = [-15,15], action_step_size = 0.15, epsilon = EPSILON)

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

		controller.decrement_epsilon(STEPS*EPOCHS)


def update(controllers) :
	
	reward = 0.0

	for controller in controllers:

		reward+=controller.update_policy()

	return reward


def extract_state(controllers,param) :

	for controller in controllers :

		if controller.param == param :

			return controller.state[0]

def randomize_target(controllers,min_value,max_value) :

	for controller in controllers :
		
		controller.setpoint = round(numpy.random.uniform(min_value,max_value),max_value)


def get_status() :

    rospy.wait_for_service('crazyflie/request_status')

    try:
        status_client = rospy.ServiceProxy('crazyflie/request_status', Status)
        return status_client(true).status

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

		cmd_topic = 'crazyflie/deep_learning/cmd_vel'

		cmd_publisher = rospy.Publisher(cmd_topic, Twist, queue_size = 1)

		rate = rospy.Rate(FPS)

		pygame.init()
		pygame.display.set_mode((20, 20))
		clock = pygame.time.Clock()
		rewards_per_episode = []
		count = 0

		#Initialize position controllers as Q Learning Agents
		controllers = init_controllers()

		print "initialized"

		status = WAITING
		start = False

		#If the user wants to train
		if TRAIN_FLAG :
		
			while True:

				status = get_status()
			
				if status is 'REINFORCEMENT LEARNING' and start is True:

					total_reward_per_episode = 0.0			

					run(controllers,cmd_publisher)

					rate.sleep()

					total_reward_per_episode += update(controllers)

					epsilon_decay(controllers)


				for event in pygame.event.get():

					if event.type == pygame.QUIT:
					     start = False
						cmd = Twist()
						for i in range(0,1000) :
							cmd.linear.z = 0.0
							cmd.linear.x = 0.0
							cmd.linear.y = 0.0
							cmd_publisher.publish(cmd)

					    pygame.quit(); 
					    sys.exit() 

					if event.type == pygame.KEYDOWN :

					    if event.key == pygame.K_UP:
						pass 
	
					    elif event.key == pygame.K_DOWN:
						pass 

					    elif event.key == pygame.K_LEFT:
						pass

					    elif event.key == pygame.K_RIGHT:
						pass

					    elif event.key == pygame.K_b:
						print "starting the reinforcement learning module"
						start = True
						time.sleep(2)

				    	    if event.key == pygame.K_q:
						start = False
						cmd = Twist()
						for i in range(0,1000) :
							cmd.linear.z = 0.0
							cmd.linear.x = 0.0
							cmd.linear.y = 0.0
							cmd_publisher.publish(cmd)
						print "stopping the reinforcement learning module"
						time.sleep(2)
						break;

						count += 1


						print " Count = " + str(count) +" ,Epsilon = " + str(controllers[0].epsilon)
	
						rewards_per_episode.append(total_reward_per_episode)
	
						print '\n \n \n rewards =' +str(total_reward_per_episode) + " ,epoch = "+str(i)	

		
						clock.tick(FPS*2)

			plotter.figure()		

			plotter.plot(EPOCH_RANGE, rewards_per_episode,'g',label='Rewards' )

			plotter.xlabel('Episode')
			plotter.ylabel('Rewards')

			plotter.title('Learning Curve ')

			plotter.savefig('../figures/LearningCurve.png')

			plotter.show()

		else :

			#simulator_reset(controllers,cmd_publisher,gazebo_publisher)
		
			count = 0
			states = []
			while not rospy.is_shutdown():

				count = count + 1
				run(controllers,cmd_publisher)
				rate.sleep()
			
				#if check_validity(controllers) is False :
					#simulator_reset(controllers,cmd_publisher,gazebo_publisher)

				states.append(extract_state(controllers,'Thrust'))

				if count > 500 :
					break

			plotter.figure()		
			plotter.plot(range(0,len(states)), states ,'g',label='Error' )
			plotter.xlabel('Time')
			plotter.ylabel('Error')
			plotter.title('Learning Curve - PID ')
			plotter.savefig('../figures/LearningCurvePID.png')
			plotter.show()

	else :
		print "Unknown Mode : not available"
		

	rospy.spin()

	


		



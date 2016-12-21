
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

from QLearning import QLearner
from rospy.exceptions import ROSException

###########################################
##
##	VARIABLES
##
###########################################

EPOCHS = 500
FPS = 20
STEPS = FPS*20
STEP_RANGE = range(0,STEPS)
EPOCH_RANGE = range(0,EPOCHS)

FUNC_APPROX_FLAG = False
EXP_REPLAY_FLAG = None
TRAIN_FLAG = True

###########################################
##
##	MAIN FUNCTION
##
###########################################

if __name__ == '__main__':

	#Initialize ros node
	rospy.init_node('QLearner', anonymous=True)

	#Parse command line arguments to check if the user wants to enable function approximation, experience replay or random opponent

	argv = sys.argv[1:]

   	try:
      		opts, args = getopt.getopt(argv,"f:e:",["func_approx=","exp_replay="])

   	except getopt.GetoptError:	
      		print 'Usage: python Train.py -f <bool> -e <bool>'
      		sys.exit(2)
   	
	for opt, arg in opts:

     		if opt in ("-f", "--func_approx"):

			if arg == 'True' :
				FUNC_APPROX_FLAG = True

			elif arg == 'False' :
				FUNC_APPROX_FLAG = False

			else :
				print 'Usage: python Train.py -f <bool> -e <bool>'
			
				sys.exit(2)

		elif opt in ("-e", "--exp_replay") :

			if arg == 'True' :
				exp_replay_flag = True

			elif arg == 'False' :
				exp_replay_flag = False

			else :
				print 'Usage: python Train.py -f <bool> -e <bool>'

				sys.exit(2)

		
	
	print "initialized"

	rate = rospy.Rate(12)
	epsilon_decay = []
	pygame.init()
	pygame.display.set_mode((20, 20))
	clock = pygame.time.Clock()
	rewards_per_episode = []
	count = 0
	total_reward_per_episode = 0

	if TRAIN_FLAG :

		#Initialize thrust controller as a DQN Agent
		thrust_controller = QLearner(param = 'Thrust', controller = 'PID', setpoint = 1.0)

	 	while True :
		
			if thrust_controller.reset_flag is False and thrust_controller.initialized  is True :
				thrust_controller.play()
				rate.sleep()
				total_reward_per_episode += thrust_controller.update_policy()
				thrust_controller.decrement_epsilon(STEPS*EPOCHS)

			if abs(thrust_controller.state[0]) > 1.5* abs(thrust_controller.initial_state[0]) or  abs(thrust_controller.state[1]) > 5  :

				thrust_controller.reset()
				rate.sleep()
				print "Resetting and breaking out of limits"
				break

			for event in pygame.event.get():

				if event.type == pygame.QUIT:
				    thrust_controller.reset()
				    pygame.quit(); 
				    sys.exit() 

				if event.type == pygame.KEYDOWN and thrust_controller.controller == 'PID':

				    if event.key == pygame.K_UP:
					thrust_controller.kp += 500
			
				    if event.key == pygame.K_DOWN:
					thrust_controller.kp -= 500

				    if event.key == pygame.K_LEFT:
					thrust_controller.kd += 500

				    if event.key == pygame.K_RIGHT:
					thrust_controller.kd -= 500

			    	    if event.key == pygame.K_r:
					thrust_controller.reset()
					time.sleep(2)

			    	    if event.key == pygame.K_e:
					thrust_controller.reset_flag = False

			    	    if event.key == pygame.K_q:
					thrust_controller.reset()
					time.sleep(2)
					break;


				    print "Kd Value = " + str(thrust_controller.kd)
				    print "Kp Value = " + str(thrust_controller.kp)
			
				    print " Count = " + str(count) +" ,Epsilon = " + str(thrust_controller.epsilon)

	else :
		thrust_controller = QLearner( param = 'Thrust', controller = 'PID', setpoint = 0.5, epsilon = -1)
		thrust_controller.reset()
		
		count = 0
		states = []
		while not rospy.is_shutdown():

			count = count + 1

			thrust_controller.play()
			rate.sleep()
			#thrust_controller.update_policy()

			if abs(thrust_controller.state[0]) > 2* abs(thrust_controller.initial_state[0]) or  abs(thrust_controller.state[1]) > 10  :
				thrust_controller.reset()

			states.append(thrust_controller.state[0])

			if count > 500 :
				break

		plotter.figure()		
		plotter.plot(range(0,len(states)), states ,'g',label='Error' )
		plotter.xlabel('Time')
		plotter.ylabel('Error')
		plotter.title('Pitch Learning Curve - PID ')
		plotter.savefig('../figures/PitchLearningCurvePID2.png')
		plotter.show()
		

	rospy.spin()

	


		


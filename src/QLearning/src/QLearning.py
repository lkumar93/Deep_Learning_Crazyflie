#
# THIS IS AN IMPLEMENTATION OF Q LEARNING ALGORITHM
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
# CONDITIONS.
#

###########################################
##
##	LIBRARIES
##
###########################################
import random
import json
import time
import numpy
import rospy
import cPickle
import math
import curses	

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

###########################################
##
##	CLASSES
##
###########################################

#Create a framework for finding the optimal winning policy using Q Learning Algorithm
class QLearner:

    #Initialize the QLearner
    def __init__(self, param = 'Z',mode='behavioral cloning', setpoint = 0.4, action_limits = [20000, 55000], action_step_size = 1000, state_variance = [0.5,0.25], learning_rate = 0.05, discount_factor = 0.5, epsilon = 0.5) :

	self.param = param
	self.learning_rate = learning_rate
	self.discount_factor = discount_factor
	self.setpoint = setpoint
	self.epsilon = epsilon
	self.initialized = False
	self.initial_state = (0.0,0.0)
	self.current_state = (0.0,0.0)
	self.state = (0.0,0.0)
	self.min_value = action_limits[0]
	self.max_value = action_limits[1]
	self.step_size = action_step_size
	self.actions = list(numpy.arange(self.min_value,self.max_value,self.step_size))
	self.actions = random.sample(self.actions,len(self.actions))
	self.reset_flag = False
	self.epochs = 0 
	self.cmd = Twist()
	self.mode = mode
	self.state_variance = state_variance
	self.calibrated = False

	cmd_topic = 'crazyflie/deep_learning/cmd_vel'
	sub_topic = '/crazyflie/ground_truth/position_error_stamped' #'/'+ drone+'/ground_truth/position'
	self.cmd_publisher = rospy.Publisher(cmd_topic, Twist, queue_size = 1)
	self.prev_z = 0.0
	self.current_z = 0.0
	self.time_stamp = 0.0
	
	rospy.Subscriber(sub_topic, Point, self.get_state)

	if self.mode is 'behavioral cloning' :
		rospy.Subscriber(cmd_topic, Twist, self.get_action)

	self.file_name = '../policies/crazyflie_' + param + '_' + str(self.min_value) + '_' + str(self.max_value) + '_' + str(self.step_size) +'_'+self.state_variance[0]+'_'+self.state_variance[1] +'_policy_lt.p'

	self.load_policy()


    #Take an action
    def play(self) :

	if self.mode is 'reinforcement' :

		current_state = self.state
		current_action = None
		current_value = 0.0
		current_count = 0.0
	
		#If current state is in Q Learner's policy 
		if current_state in self.policy :

			#Epsilon-Greedy Strategy : Exploitation vs Exploration
			#Select the best action with (1-epsilon) probability
			if random.random() > self.epsilon :
				#Find the best action and value
				current_action, current_value,current_count = self.get_best_action_value(current_state,self.actions)

	 	#As current state is not in the policy, initialize a list for that particular state
		else :
			self.policy[current_state] = []

		#If the current state is not in the policy or if selecting a random action with epsilon probability 
		#Make sure you are selecting an action that has not been executed before , if all the actions have been 
		#executed before then do a random action

		if current_action is None :

			current_action, current_value,current_count = self.get_best_action_value(current_state,self.actions,random_action = True)
					
		print "current_state = " + str(current_state)

		#Execute the current action
		self.execute_action(current_action)

		self.current_state = current_state
		self.current_action = current_action
		self.current_value = current_value
		self.current_count = current_count

	else :
		print "Cannot execute actions in behavioral cloning mode"



    #Update the Q(s,a) table	
    def update_policy(self) :

	#Find the next state after the second player has played
	next_state = self.state

	print "next_state = " + str(next_state)

	#Initialize the value for next state action pair
	next_value = 0

	next_q_values = None

	reward = self.get_reward(next_state)

	#Check which moves are legal	

	#If the next state is in QLearner's policy, then
	if next_state in self.policy : 
		#Find the next action and the value of next state/action pair
		next_action, next_value,next_count = self.get_best_action_value(next_state,self.actions)

	#As next state is not in the policy, initialize a list for that particular state
	else :
		self.policy[next_state] = []

	if abs(next_state[0]) <= 0.03 and abs(next_state[1]) <= 0.005 :

		self.current_value = 10.0

	else :

		#Q(s,a) = Q(s,a) + Alpha * ( R' + Gamma*Q(s',a') - Q(s,a) )
		self.current_value = self.current_value + (self.learning_rate)*(reward+ self.discount_factor*next_value - self.current_value)
		self.current_value = reward
		#self.current_value = (self.get_reward(next_state)+ self.discount_factor*next_value)

	#Round off to four decimal points
	self.current_value = round(self.current_value,4)

	action_in_policy = False

	print "action = " + str(self.current_action) + " value = " + str(self.current_value)+ "count = " + str(self.current_count)

	#If the action is in the policy for current state modify the value of the action
	for av in self.policy[self.current_state] :
		if av['action'] == self.current_action :
			action_in_policy = True
			av['value'] = self.current_value
			av['count'] = self.current_count

	#If the action is not in the policy, augment the action value pair to that state
	if action_in_policy is not True :
		self.policy[self.current_state].append({'action':self.current_action,'value':self.current_value,'count':self.current_count})


	self.epochs += 1

	#Save policy once in every 10000 episodes
	if self.epochs % 1000 == 0 :
		#Save the updated policy
		self.save_policy()	

	return reward


    #Find the best action for a particular state
    def get_best_action_value(self, state, possible_actions, random_action = False, use_target = False):

	action = None
	value = 0.0
	count = 1.0
	q_values = None
	index = 0


	#if selecting a random action with epsilon probability, make sure you are selecting an action that has been never been executed before 			
	# If all the actions have been executed before, select action with less count, if count is more than 5 times , then do a random action	
	if random_action :

		action = random.choice(possible_actions)

		if len(self.policy[state]) < len(possible_actions) :
			for move in possible_actions:

				action_in_policy = False

				for av in self.policy[state] :

					if av['action'] == move :
				
						action_in_policy = True
				
					if av['action'] == action :

						value = av['value']
						count = av['count']


				if action_in_policy is False :
				
					action = move
					value = 0.0
					count = 1.0
					break

		else :

			sorted_av_table = []

			#Sort the actions according to the count
			sorted_av_table = sorted(self.policy[state], reverse=False, key = lambda av : av['count'])

			print sorted_av_table

			for av in sorted_av_table :
	
				if av['action'] == action :

					value = av['value']
					count = av['count'] + 1

				if av['count'] < 10 and av['action'] in possible_actions:
					action = av['action']
					value = av['value']
					count = av['count'] + 1
					break


	else :
	
		sorted_av_table = []

		#Sort the actions according to the values
		sorted_av_table = sorted(self.policy[state], reverse=True, key = lambda av : av['value'])

		print sorted_av_table
		for av in sorted_av_table :
			if av['value'] > -20*abs(self.initial_state[0]) and av['action'] in possible_actions :
				action = av['action']
				value = av['value']
				count = av['count'] + 1
				break

	return action, value, count

    #Reward Function
    def get_reward(self,state) :

	#Gaussian Reward Function
	reward = 10*math.exp(-(pow(state[0],2)/self.state_variance[0])-(pow(state[1],2)/self.state_variance[1]))
	
	print reward

	return reward

    #Get state of the crazyflie
    def get_state(self, msg) :

	if self.param == 'Roll' :
		value = msg.point.y
	elif self.param == 'Pitch' :
		value = msg.point.x
	elif self.param == 'Thrust' :
		value = msg.point.z
	else:
		print 'Invalid Param'
		return

	state = round(value,3)
	dt = msg.header.stamp.toSec() - self.timestamp
	self.state = (state, round(state-self.state[0],3))

	self.timestamp = msg.header.stamp.toSec()

	if self.initialized is False and self.reset_flag is False :
		self.initial_state = (state, 0.0)
		self.state = self.initial_state
		self.initialized = True

    #Get current action of the drone
    def get_action(self, msg) :

	if self.mode is 'behavioral cloning':
		if self.param == 'Y' :
			value = msg.linear.y
		elif self.param == 'X' :
			value = msg.linear.x
		elif self.param == 'Z' :
			value = msg.linear.z
		else:
			print 'Invalid Param'
			return

		value = round(value/self.step_size)*self.step_size

		self.current_action = value
		self.calibrated = True

	else :
		print "Cannot subscribe to actions in reinforcement learning mode"
		

    def execute_action(self,action) :

	if self.param == 'Pitch':

		self.cmd.linear.x = action

		print "Pitch = " + str(self.cmd.pitch) 

	elif self.param == 'Roll' :

		self.cmd.linear.y = action

		print "Roll = " + str(self.cmd.roll) 

	else :

		self.cmd.linear.z =   action

		print "Thrust = " + str(self.cmd.linear.z) 

	self.prev_z = self.current_z
	self.cmd_publisher.publish(self.cmd)	
	

    def decrement_epsilon(self,value) :

	if self.epsilon > 0.0 :
		self.epsilon -= 1.0/value
	else :
		self.epsilon = 0.0

    #Load the policy 
    def load_policy(self) :

	self.policy = {}

	#Open the json file if available
	try:
		policy_file = open(self.file_name, 'rb')
	except IOError:
		print "No such file exists. Will create one"
		time.sleep(3.0)
		return

	#Load the contents of the file to QLearner's policy
	self.policy = cPickle.load(policy_file)

	#Close the policy
	policy_file.close()

	print self.file_name + ' loaded'

    #Save the policy
    def save_policy(self) :

	#Save the policy as a pickle file
	policy_file = open(self.file_name,'wv')

	#Dump the dictionary in QLearner's file as a Pickle Object to policy file
	cPickle.dump(self.policy,policy_file) #,sort_keys = True,indent = 4, separators=(',',': '))

	#Close the policy file
	policy_file.close()

	print self.file_name + ' saved'	

    def reset(self) :

	self.cmd.linear.z = 0.0
	self.cmd.linear.x = 0.0
	self.cmd.linear.y = 0.0
	self.cmd.angular.y = 0.0
	self.reset_flag = True
	self.initialized = False
	self.calibrated = True

	self.cmd_publisher.publish(self.cmd)	
	time.sleep(0.2)
	

	

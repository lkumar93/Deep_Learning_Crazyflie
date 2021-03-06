#
# THIS IS AN IMPLEMENTATION OF Q LEARNING ALGORITHM
#
# COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE
#
# AUTHOR : LAKSHMAN KUMAR
# AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
# EMAIL : LKUMAR93@TERPMAIL.UMD.EDU
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
from geometry_msgs.msg import TwistStamped

from deep_learning_crazyflie.srv import *

###########################################
##
##	CLASSES
##
###########################################

#Create a framework for finding the optimal winning policy using Q Learning Algorithm
class QLearner:

    #Initialize the QLearner
    def __init__(self, param = 'Z',mode='behavioral cloning', action_limits = [20000, 55000], action_step_size = 1000, state_variance = [0.5,0.5], learning_rate = 0.05, discount_factor = 0.5, epsilon = 0.5, kp=5000, ki=3000, kd=6500, kp_range = [1000,10000], ki_range = [1000,6000], kd_range = [1000,10000]) :

	self.param = param
	self.learning_rate = learning_rate
	self.discount_factor = discount_factor
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
	self.kp = kp
	self.ki = ki
	self.kd = kd
	self.kp_max = kp_range[1]
	self.kp_min = kp_range[0]
	self.ki_max = ki_range[1]
	self.ki_min = ki_range[0]
	self.kd_max = kd_range[1]
	self.kd_min = kd_range[0]

	cmd_topic = '/crazyflie/deep_learning/cmd_vel'
	sub_topic = '/crazyflie/deep_learning/state_stamped' #'/'+ drone+'/ground_truth/position'
	self.cmd_publisher = rospy.Publisher(cmd_topic, Twist, queue_size = 1)
	self.prev_z = 0.0
	self.current_z = 0.0
	self.timestamp = 0.0

	if self.param == 'Z' :
		self.tune_param = 2
	elif self.param == 'Y' :
		self.tune_param = 1
	elif self.param == 'X' :
		self.tune_param = 0
	else :
		print "No such param available"
		return
	
	rospy.Subscriber(sub_topic, TwistStamped, self.get_state)

	if self.mode == 'behavioral cloning' :
		print "starting subscriber for position "+str(self.param)
		rospy.Subscriber(cmd_topic, Twist, self.get_action, queue_size=1)

	self.file_name = '../policies/crazyflie_' + param + '_' + str(self.min_value) + '_' + str(self.max_value) + '_' + str(self.step_size) +'_'+str(self.state_variance[0])+'_'+str(self.state_variance[1]) +'_policy_lt.p'

	self.load_policy()

    #Take an action
    def run(self) :

	if self.mode == 'reinforcement learning' :

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

		self.current_state = current_state
		self.current_action = current_action
		self.current_value = current_value
		self.current_count = current_count

		return self.current_action

	else :
		print "Cannot execute actions in behavioral cloning mode"



    def get_pid_action(self) :

	rospy.wait_for_service('crazyflie/tune_pid')

	try:
		pid_tuning_client = rospy.ServiceProxy('crazyflie/tune_pid', TunePID)
		return 	pid_tuning_client(self.tune_param, self.kp, self.ki, self.kd).action

	except rospy.ServiceException, e:
		print "tune_pid service call failed: %s"%e
		return self.current_action

    
    def randomize_pid(self) :

	self.kp = random.uniform(self.kp_min, self.kp_max);
	self.ki = random.uniform(self.ki_min, self.ki_max);
	self.kd = random.uniform(self.kd_min, self.kd_max);

    #Update the Q(s,a) table	
    def update_policy(self) :

	#Find the next state after the second player has played
	next_state = self.state

	#Initialize the value for next state action pair
#	next_value = 0

#	next_q_values = None

	reward = self.get_reward(next_state)

	#Check which moves are legal	

	#If the next state is in QLearner's policy, then
#	if next_state in self.policy : 
		#Find the next action and the value of next state/action pair
#		next_action, next_value,next_count = self.get_best_action_value(next_state,self.actions)
		
	#As next state is not in the policy, initialize a list for that particular state
	if next_state not in self.policy :
		self.policy[next_state] = []


	#Q(s,a) = Q(s,a) + Alpha * ( R' + Gamma*Q(s',a') - Q(s,a) )
	#self.current_value = self.current_value + (self.learning_rate)*(reward+ self.discount_factor*next_value - self.current_value)
	self.current_value = reward
	#self.current_value = (self.get_reward(next_state)+ self.discount_factor*next_value)

	#Round off to four decimal points
	#self.current_value = round(self.current_value,4)

	action_in_policy = False

	print "param =" +str(self.param) + " ,action = " + str(self.current_action) + " ,reward = " + str(self.current_value)+  " ,current state = " +str(self.current_state) + " ,next state =" +str(next_state)+" ,count = " + str(self.current_count)

	#If the action is in the policy for current state modify the value of the action
	for av in self.policy[self.current_state] :
		if av['action'] == self.current_action :
			action_in_policy = True
			av['value'] = self.current_value
			av['count'] = self.current_count
			break

	#If the action is not in the policy, augment the action value pair to that state
	if action_in_policy is not True :
		self.policy[self.current_state].append({'action':self.current_action,'value':self.current_value,'count':self.current_count})


	self.epochs += 1

	#Save policy once in every 100 episodes
	if self.epochs % 100 == 0 :
		#Save the updated policy
		self.save_policy()	

	return reward


    #Find the best action for a particular state
    def get_best_action_value(self, state, possible_actions, random_action = False):

	action = None
	value = 0.0
	count = 1.0
	q_values = None
	index = 0

	#if selecting a random action with epsilon probability, make sure you are selecting an action that has been never been executed before 			
	# If all the actions have been executed before, select action with less count, if count is more than 5 times , then do a random action	
	if random_action :

		action = round(self.get_pid_action()/self.step_size)*self.step_size

		for av in self.policy[state] :
			if av['action'] == action :
				value = av['value']
				count = av['count']

	else :
	
		sorted_av_table = []

		#Sort the actions according to the values
		sorted_av_table = sorted(self.policy[state], reverse=True, key = lambda av : av['value'])

		#print sorted_av_table
		for av in sorted_av_table :
			if av['action'] <=self.max_value and av['action'] >= self.min_value :
				action = av['action']
				value = av['value']
				count = av['count'] + 1
				break

	return action, value, count

    #Reward Function
    def get_reward(self,state) :

	#Gaussian Reward Function
	if self.param == 'Z' :
		#Add a gain term that depends on current action(thrust) to overcome ground effect
		ground_effect_gain = (self.current_action*(5+state[2])/(1+10*abs(state[0]) +10*abs(state[1]))) / (self.max_value )
		reward = 10*math.exp(-(pow(state[0],2)/self.state_variance[0])-(pow(state[1],2)/self.state_variance[1]))+ground_effect_gain
	else :
		reward = 10*math.exp(-(pow(state[0],2)/self.state_variance[0])-(pow(state[1],2)/self.state_variance[1]))
	
	return reward

    #Get state of the crazyflie
    def get_state(self, msg) :

	if self.param == 'Y' :
		value = msg.twist.angular.y
	elif self.param == 'X' :
		value = msg.twist.angular.x
	elif self.param == 'Z' :
		value = msg.twist.angular.z
	else:
		print 'Invalid Param'
		return

	state = round(value,2)
	dt = msg.header.stamp.to_sec() - self.timestamp
	ground_distance = round(msg.twist.linear.z, 2)

	if self.param == 'Z' :
		self.state = (state, round((state-self.state[0])/dt,2),ground_distance)
	else :
		self.state = (state, round((state-self.state[0])/dt,2))

	self.timestamp = msg.header.stamp.to_sec()

	if self.initialized is False and self.reset_flag is False :
		if self.param == 'Z':
			self.initial_state = (state, 0.0, ground_distance)
		else :
			self.initial_state = (state, 0.0)
		self.state = self.initial_state
		self.initialized = True

    #Get current action of the drone
    def get_action(self, msg) :

	if self.initialized is True :

		if self.mode == 'behavioral cloning' :
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

			self.calibrated = True
			self.current_action = value
			self.current_state = self.state
			count = 0

			if self.current_state not in self.policy :
				self.policy[self.current_state] = []

			else :
				for av in self.policy[self.current_state] :
					if  av['action'] == self.current_action :
						count = av['count'] + 1
						break
				
			self.current_count = count

			time.sleep(0.0095)

			self.update_policy()

		else :
			print "Cannot subscribe to actions in reinforcement learning mode"

	else :
		print "Have not received the state information yet"
		

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
	

	

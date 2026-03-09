"""
An environment that contains a differential drive robot with its own dynamics.
(c) Juan-Antonio Fernández-Madrigal, 2026
"""

import copy
import math
import sys
import gymnasium as gym # gym.spaces for defining observation and action spaces
import numpy as np
import pygame
from diffrobotdynamicssystem import DiffRobotDynamicsSystem


# --- environment class

class DiffRobotDynamicsEnv(gym.Env):
	
	metadata = {"render_modes": ["human"], "render_fps": 30}
	
	def __init__(self,stepper=0.010,resolt=1e-3,\
					  sideofenv=5.0,wradii=[0.03,0.03],wdist=0.15,sparserew=True,\
					  finalalphatest=4.0,ttest=100e-3,alphatest=2.0,inputtest=2.0,\
				 	  render_mode="human",pausereset=2000,trace=False):
	
		super().__init__()			

		if not (render_mode is None or \
			    render_mode in self.metadata["render_modes"]):
			raise ValueError("Invalid render mode")

		self._system = DiffRobotDynamicsSystem(stepper,resolt,
											   sideofenv,wradii,wdist,
					  						   finalalphatest,
					  						   ttest,alphatest,inputtest,
										 	   self.metadata["render_fps"],
										 	   pausereset,
										 	   trace)			
       										
		# Observation is the relative distance and angle to the target
		dobs = {\
				"reldist": gym.spaces.Box(low = 0.0, 
										  high = self._system.max_relative_dist(),
									   	  shape=(1,),dtype=np.float32),
				"relang": gym.spaces.Box(low = -math.pi, high = math.pi,
									   	  shape=(1,),dtype=np.float32)									   	  
			   }
		print("Observation space: {}".format(dobs))
		self.observation_space = gym.spaces.Dict(dobs)
		
        # Action is the power set to each wheel
		da = gym.spaces.Box(low = np.array([-100.0,-100.0],dtype=np.float32),
							high = np.array([+100.0,+100.0],dtype=np.float32),
							shape=(2,),
							dtype=np.float32)
		print("Action space: {}".format(da))
		self.action_space = da
		
		# set the render mode								
		self.render_mode = render_mode
        
		# use any other custom argument and set the rest of the instance members
		self._stepper = stepper
		self.setRewardMode(sparserew)
		self._numstepsinepisode = None
		self._numep = -1
		self._accreward = None
		self._trace = trace
		
	def close(self):
		# just close any resource acquired by the class
		self._system.close()
			
	def setRadii(self,radii):
		self._system.setRadii(radii)
			
	def setWheelDist(self,wdist):
		self._system.setWheelDist(wdist)

	def setRewardMode(self,sparseornot):
		if not isinstance(sparseornot,bool):
			raise ValueError("Invalid sparseornot mode")
		self._rewardmode = sparseornot

	def getRadii(self):
		return self._system.getRadii()
		
	def getWheelDist(self):
		return self._system.getWheelDist()
			
	def setPause(self,p):
		# change pause after rendering (in msecs)
		self._system.setPause(p)
		
	def setRendering(self,enable):
		# enable or disable rendering.
		self._system.setRendering(enable)
		
	def render(self):
		# standard method to update the rendering
		if self.render_mode == "human":
			self._system.render_frame(self._numep,self._numstepsinepisode)
		
	def reset(self,seed=None,options=None):						 
		super().reset(seed=seed)

		self._numep += 1
		self._numstepsinepisode = 0
		self._accreward = 0.0

		self._system.reset(self.np_random.uniform(low = -math.pi, high = math.pi), # orient
						   self.np_random.uniform(low = 0.0, high = self._system.env_side() / 2.0), # rel dist
						   self.np_random.uniform(low = -math.pi, high = math.pi), # rel ang
						   self.render_mode == "human",
						   self._numep,self._numstepsinepisode) 
		self._potentialrewardprev = self._potential_reward(\
										self._system.get_relative_angle(),
										self._system.get_relative_dist())
		
		observation = self._get_obs()
		info = self._get_info()

		return observation, info # new state, additional info
		
	def step(self,action):
		
		terminated = False
		truncated = False 
		reward = 0.0 
				
		# action is already chosen by baselines
		if self._trace:
			print("\tAction: {}".format(action))
		self._system.simulate(action) # let the system simulate for that period

		# Get new state and info
		observation = self._get_obs()
		info = self._get_info()

		# get reward (sparse)
		# get reward 
		if self._rewardmode: # sparse
			if self._system.get_collision_status():
				reward = -100.0
				terminated = True
			elif self._system.get_relative_dist() <= 0.05:
				if math.fabs(self._system.get_relative_angle()) <= 5.0 * math.pi / 180.0:
					reward = 100.0 - self._system.get_episode_time()
					terminated = True
			else:
				reward = 0.0
		else: # dense. CARE: a dense reward may be in opposition to the correct behaviour in some situations
			if self._system.get_collision_status():
				reward = -100.0
				terminated = True
			elif self._system.get_relative_dist() <= 0.05:
				if math.fabs(self._system.get_relative_angle()) <= 5.0 * math.pi / 180.0:
					reward = 100.0 - self._system.get_episode_time()
					terminated = True
			else:
				reward = 0.0
				potreward = self._potential_reward(\
								self._system.get_relative_angle(),
								self._system.get_relative_dist())
				reward += potreward - self._potentialrewardprev 
				self._potentialrewardprev = potreward
		self._accreward += reward
		if self._trace:
			print("\treward: {}, accrew: {}".format(reward,\
													self._accreward))
		# prepare for next step
		self._numstepsinepisode += 1
		if self._numstepsinepisode * self._stepper > self._system.max_relative_dist() * 2.0 / 0.4:
			truncated = True # if not truncated or terminated, no reward logging

		# render frame
		self.render()

		# new state, reward, 
		# episode termination because end state reached,
		# episode termination (truncation) because invalid situation, 
		# additional info
		return observation,reward,terminated,truncated,info		
	
	# ----- private members
	
	def _get_obs(self):
		# convenience routine to return current state in the format needed by 
		# gym.Env
		obs ={"reldist": np.array([self._system.get_relative_dist()], dtype=np.float32),
			  "relang": np.array([self._system.get_relative_angle()], dtype=np.float32)} # a simple scalar does not work; it has to have shape=(1,)
		return obs
		
	def _get_info(self):
		# additional information that must be returned by step() and reset()
		return {} # must be a dictionary even if returning nothing
		
	def _potential_reward(self,relang,reldist):
		# potential function Φ(s) that assigns a scalar value to each state, 
		# encoding how promising that state is
		return -10.0 * reldist 


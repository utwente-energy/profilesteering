   # Copyright 2023 University of Twente

   # Licensed under the Apache License, Version 2.0 (the "License");
   # you may not use this file except in compliance with the License.
   # You may obtain a copy of the License at

       # http://www.apache.org/licenses/LICENSE-2.0

   # Unless required by applicable law or agreed to in writing, software
   # distributed under the License is distributed on an "AS IS" BASIS,
   # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   # See the License for the specific language governing permissions and
   # limitations under the License.

import random
import operator 
import numpy as np

class Load():
	def __init__(self):
		self.profile = []	# x_m in the PS paper
		self.candidate = []	# ^x_m in the PS paper
		
		# Device specific params
		self.max = 5000
	
	def init(self, p):
		# Create a baseload for a given number of intervals
		self.profile = [] # Empty the profile
		
		# We create a random list of power values, but it can be any list
		for i in range(0, len(p)):
			self.profile.append(self.max*random.random())
			
		return list(self.profile)
			
	def plan(self, d):
		assert(len(d) == len(self.profile))
		p_m = list(map(operator.sub, self.profile, d)) # p_m = x_m - d
		
		self.candidate = list(self.profile)	# A baseload offers no flex, so we can just return the profile
											# Note that we need to create a new list due to "hidden pointers" in Python
											
		# Calculate the improvement by this device:
		e_m = np.linalg.norm(np.array(self.profile)-np.array(p_m)) - np.linalg.norm(np.array(self.candidate)-np.array(p_m))
	
		# Return the improvement
		# Note that e_m should be 0 for a static device
		# print("Improvement: ", self, e_m)
		return e_m
		
	def accept(self):
		# We are chosen as winner, replace the profile:
		self.profile = list(self.candidate)
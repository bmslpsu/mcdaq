#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

from uldaq import (get_daq_device_inventory, DaqDevice, InterfaceType, AiInputMode, AInFlag)

import rospy
import rosparam

from std_msgs.msg import Header
from daq_test_1.msg import MC_AnalogIN

class GetAI:

	def __init__(self):
		self.bInitialized = False

		# Initialize
		self.name = 'GetAI'
		rospy.init_node(self.name, anonymous=False)
		self.nodename = rospy.get_name()
		self.namespace = rospy.get_namespace()
		self.command = None

		# Messages & Services.
		self.topicAI = '%s/mcdaq/AI' % self.namespace.rstrip('/')
		self.pubAI = rospy.Publisher(self.topicAI, MC_AnalogIN,queue_size = 100)
		self.rate = rospy.Rate(5000)

		# DAQ setup
		self.daq_device = None
		self.descriptor_index = 0
		self.range_index = 0
		self.interface_type = InterfaceType.USB
		self.channels = [0]
		self.n_channels = len(self.channels)
		self.voltages = [None] * self.n_channels

		self.devices = get_daq_device_inventory(self.interface_type)
		self.number_of_devices = len(self.devices)
		if self.number_of_devices == 0:
			raise Exception('Error: No DAQ devices found')

		rospy.logwarn('Found %i DAQ device(s):',self.number_of_devices)
		for i in range(self.number_of_devices):
			rospy.logwarn('   %s    %s', self.devices[i].product_name, self.devices[i].unique_id)

		# Create the DAQ device object associated with the specified descriptor index.
		self.daq_device = DaqDevice(self.devices[self.descriptor_index])

		# Get the AiDevice object and verify that it is valid.
		self.ai_device = self.daq_device.get_ai_device()
		if self.ai_device is None:
			raise Exception('Error: The DAQ device does not support analog input')

		# Establish a connection to the DAQ device.
		self.descriptor = self.daq_device.get_descriptor()
		rospy.logwarn('Connecting to %s...',self.descriptor.dev_string)
		self.daq_device.connect()

		self.ai_info = self.ai_device.get_info()

		# The default input mode is SINGLE_ENDED.
		self.input_mode = AiInputMode.SINGLE_ENDED

		# If SINGLE_ENDED input mode is not supported, set to DIFFERENTIAL.
		if self.ai_info.get_num_chans_by_mode(AiInputMode.SINGLE_ENDED) <= 0:
			self.input_mode = AiInputMode.DIFFERENTIAL

		# Get the number of channels and validate the high channel number.
		self.number_of_channels = self.ai_info.get_num_chans_by_mode(self.input_mode)
		if max(self.channels) >= self.number_of_channels:
			rospy.logerr('Channels are outside of AI range')

		# Get a list of supported ranges and validate the range index.
		self.ranges = self.ai_info.get_ranges(self.input_mode)
		if self.range_index >= len(self.ranges):
			self.range_index = len(self.ranges) - 1

		rospy.logwarn('	%s ready',self.descriptor.dev_string)
		rospy.logwarn('    	Channels: %s ', ' '.join(map(str,self.channels)))
		rospy.logwarn('    	Input mode: %s', self.input_mode.name)
		rospy.logwarn('    	Range: %s', self.ranges[self.range_index].name)

		rospy.sleep(0.5) # Allow time to connect.

		self.bInitialized = True

	def run(self):
		rospy.logwarn('MC DAQ Initialized')
		iCount = 0
		while not rospy.is_shutdown():
			header = Header(seq=iCount, stamp=rospy.Time.now()) #if (len(self.channels)>0):
			for ch in range(self.channels[0], self.channels[-1] + 1):
				self.voltages[ch] = self.ai_device.a_in(ch, self.input_mode, self.ranges[self.range_index], AInFlag.DEFAULT)
				#rospy.logwarn('   Channel %i Data: %2.4f', self.channels[ch],voltages[ch])
				self.rate.sleep()
			self.pubAI.publish(header, self.channels, self.voltages)
			iCount += 1

	def command_callback(self, msg):
		self.command = msg.data

		if self.command == 'exit':
			rospy.signal_shutdown('User requested exit.')


		if self.command == 'help':
			rospy.logwarn('Help')

if __name__ == '__main__':
	main = GetAI()
	main.run()

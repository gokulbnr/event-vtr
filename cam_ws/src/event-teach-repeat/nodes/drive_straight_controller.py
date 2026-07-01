#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_conversions
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
import os
import datetime
import threading
import json
from rospy_message_converter import message_converter
import ros_numpy
import pyrealsense2 as rs
import threading

from teach_repeat.read_prophesee import PropheseeReader
import teach_repeat.image_processing as image_processing

class drive_straight_controller(PropheseeReader):

	def __init__(self):
		super().__init__("")
		self.event_frame_gen.set_fps(100)


		self.setup_parameters()
		self.setup_publishers()
	
		time_stamp = rospy.Time.now().to_sec()
		message_as_text = json.dumps({'time_stamp': time_stamp})
		with open(self.save_dir + ('zero_time.txt'), 'w') as start_file:
			start_file.write(message_as_text)

		self.setup_subscribers()

		self.rs_pipeline = rs.pipeline()
		rs_config = rs.config()

		rs_config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
		self.rs_pipeline.start(rs_config)

		rs_thread = threading.Thread(target=self.get_rs_frame, args=())
		rs_thread.start()

		self.run()


	def setup_parameters(self):
		self.teach_mode = rospy.get_param('~teach_mode', False)
		self.save_dir = os.path.expanduser(rospy.get_param('~save_dir', '/home/qcr/gokul/tnr-toy-data/'))

		self.state = 0
		self.event_count = 0
		self.rgb_count = 0

		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		
		self.save_dir += datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S/')
		self.event_save_dir = self.save_dir + 'events/'
		self.rgb_save_dir = self.save_dir + 'images/'
		self.rgb_pose_save_dir = self.save_dir + 'images_poses/'
		self.event_pose_save_dir = self.save_dir + 'events_poses/'
		self.event_offset_correction = self.save_dir + 'events_offset_correction/'
		self.rgb_offset_correction = self.save_dir + 'rgb_offset_correction/'

		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)
		if not os.path.isdir(self.event_save_dir):
			os.makedirs(self.event_save_dir)
		if not os.path.isdir(self.rgb_save_dir):
			os.makedirs(self.rgb_save_dir)
		if not os.path.isdir(self.rgb_pose_save_dir):
			os.makedirs(self.rgb_pose_save_dir)
		if not os.path.isdir(self.event_pose_save_dir):
			os.makedirs(self.event_pose_save_dir)
		if not os.path.isdir(self.event_offset_correction):
			os.makedirs(self.event_offset_correction)
		if not os.path.isdir(self.rgb_offset_correction):
			os.makedirs(self.rgb_offset_correction)
	
		self.mutex = threading.Lock()

		self.start_frame = None
		self.current_frame = None
		self.last_frame_rgb = None
		self.diff_last_curr_rgb = None
		self.last_frame_event = None
		self.diff_last_curr_event = None
		self.last_state_frame = None
		self.odom_ready = False

		self.resize = image_processing.make_size(height=44, width=115)

		if self.teach_mode is False:
			load_dir = rospy.get_param('~load_dir','/home/qcr/gokul/tnr-toy-data/2025-02-27_10:56:31/')
			
			event_files = self.get_image_files_from_dir(load_dir+'events/','.png')
			self.ref_event = self.load_events_raw(event_files)
			
			rgb_files = self.get_image_files_from_dir(load_dir+'images/','.png')
			self.ref_rgb = self.load_images_raw(rgb_files)


	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=0)


	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=1)
		# self.sub_events = rospy.Subscriber("event_frame", Image, self.process_event_frames, queue_size=1, buff_size=2**22)
		# self.sub_images = rospy.Subscriber("camera/color/image_raw", Image, self.process_rgb_frames, queue_size=1, buff_size=2**22)

	def get_image_files_from_dir(self,file_dir, file_ending):
		files = [f for f in os.listdir(file_dir) if f.endswith(file_ending)]
		files.sort()
		return [file_dir+f for f in files]

	def load_images_raw(self, image_files):
		raw_images = [cv2.imread(image_file) for image_file in image_files]
		normalised_images = [image_processing.patch_normalise_image(image, (9,9), resize=self.resize) for image in raw_images]
		return np.ascontiguousarray(normalised_images)

	def load_events_raw(self, image_files):
		raw_images = [cv2.imread(image_file) for image_file in image_files]
		imgs = [((cv2.threshold(image[:, :, 1], 100, 255, cv2.THRESH_BINARY)[1]).astype(np.int16) - 64).astype(np.int16) for image in raw_images]
		padded_imgs = [np.pad(image, ((0,0), (int(image.shape[1]/2), int(image.shape[1]/2))), mode='constant') for image in imgs]
		return np.ascontiguousarray(padded_imgs)

	# def process_event_frames(self, msg):
	# 	if self.odom_ready is True:
	# 		n = msg.header.seq
	# 		time_at_recv = rospy.Time.now().to_sec()

	# 		full_image = ros_numpy.numpify(msg)

	# 		if self.teach_mode is True:
	# 			self.diff_last_curr_event = self.last_frame_event.Inverse() * self.current_frame
	# 			id = '%06d'%self.event_count
	# 			if self.event_count == 0 or self.diff_last_curr_event.p.Norm() > 0.1:
	# 				cv2.imwrite(self.event_save_dir + id + '.png', full_image)
	# 				message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
	# 				with open(self.event_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
	# 					pose_file.write(message_as_text)
	# 				self.event_count += 1
	# 				self.last_frame_event = self.current_frame

	# 		else:
	# 			# resized_image = cv2.resize(full_image, None, fx=0.0625, fy=0.0625, interpolation=cv2.INTER_AREA)
	# 			Y = (cv2.threshold(full_image[:, :, 1], 100, 255, cv2.THRESH_BINARY)[1]).astype(np.int16)
	# 			normalised_image = (Y - 64).astype(np.int16)
	# 			ref_img = np.concatenate(self.ref_event, axis=1)

	# 			offsets_data, correlation_data, debug_image = image_processing.concat_fft_match_images_debug(ref_img, normalised_image, len(self.ref_event), 10, 1)
	# 			# offsets_deg = np.ascontiguousarray([offset * 35.943 / 80 for offset in np.ascontiguousarray(offsets_data)])
	# 			time_stamp = rospy.Time.now().to_sec()
				
	# 			if self.state < 4:
	# 				id = '%06d'%self.event_count
	# 				best_match = np.argmax(np.ascontiguousarray(correlation_data))
	# 				offset_deg = offsets_data[best_match] * 35.943 / 80
	# 				print('Event: %f'%offset_deg)

	# 				debug_image = image_processing.visualize((normalised_image+64).astype(np.uint8), (self.ref_event[best_match][:,40:120]+64).astype(np.uint8), offsets_data[best_match]+normalised_image.shape[1]//2)
	# 				cv2.imwrite(self.event_save_dir + id + '.png', debug_image)
	# 				message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
	# 				with open(self.event_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
	# 					pose_file.write(message_as_text)
	# 				message_as_text = json.dumps({'time_stamp': time_stamp, 'theta_offset': offset_deg, 'time_at_recv': time_at_recv})
	# 				with open(self.event_offset_correction+'%06d_correction.txt' % self.event_count, 'w') as correction_file:
	# 					correction_file.write(message_as_text)

	# 				self.event_count += 1


	def on_cd_frame_cb(self, ts, cd_frame):
		if self.odom_ready is True:
			full_image = np.flipud(np.fliplr(cv2.resize(cd_frame, None, fx=0.0625, fy=0.0625, interpolation=cv2.INTER_AREA)))
			time_at_recv = rospy.Time.now().to_sec()

			if self.teach_mode is True:
				self.diff_last_curr_event = self.last_frame_event.Inverse() * self.current_frame
				id = '%06d'%self.event_count
				if self.event_count == 0 or self.diff_last_curr_event.p.Norm() > 0.1:
					cv2.imwrite(self.event_save_dir + id + '.png', full_image)
					message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
					with open(self.event_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
						pose_file.write(message_as_text)
					self.event_count += 1
					self.last_frame_event = self.current_frame

			else:
				# resized_image = cv2.resize(full_image, None, fx=0.0625, fy=0.0625, interpolation=cv2.INTER_AREA)
				Y = (cv2.threshold(full_image[:, :, 1], 100, 255, cv2.THRESH_BINARY)[1]).astype(np.int16)
				normalised_image = (Y - 64).astype(np.int16)
				ref_img = np.concatenate(self.ref_event, axis=1)

				offsets_data, correlation_data, debug_image = image_processing.concat_fft_match_images_debug(ref_img, normalised_image, len(self.ref_event), 10, 1)
				offsets_deg = [offset * 35.943 / 80 for offset in offsets_data]
				time_stamp = rospy.Time.now().to_sec()
				
				if self.state < 4:
					id = '%06d'%self.event_count
					best_match = np.argmax(correlation_data)
					print('Event: %f'%offsets_deg[best_match])

					debug_image = image_processing.visualize((normalised_image+64).astype(np.uint8), (self.ref_event[best_match][:,40:120]+64).astype(np.uint8), offsets_data[best_match]+normalised_image.shape[1]//2)
					cv2.imwrite(self.event_save_dir + id + '.png', debug_image)
					message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
					with open(self.event_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
						pose_file.write(message_as_text)
					message_as_text = json.dumps({'time_stamp': time_stamp, 'theta_offset': offsets_deg[best_match], 'time_at_recv': time_at_recv})
					with open(self.event_offset_correction+'%06d_correction.txt' % self.event_count, 'w') as correction_file:
						correction_file.write(message_as_text)

					self.event_count += 1


	def get_rs_frame(self):
		while True:
			frames = self.rs_pipeline.wait_for_frames()
			full_image = frames.get_color_frame()

			if not full_image:
				continue

			full_image = np.asanyarray(full_image.get_data())
			time_at_recv = rospy.Time.now().to_sec()

			if self.teach_mode is True:
				self.diff_last_curr_rgb = self.last_frame_rgb.Inverse() * self.current_frame
				id = '%06d'%self.rgb_count
				if self.rgb_count == 0 or self.diff_last_curr_rgb.p.Norm() > 0.1:
					cv2.imwrite(self.rgb_save_dir + id + '.png', full_image)
					message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
					with open(self.rgb_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
						pose_file.write(message_as_text)
					self.rgb_count += 1
					self.last_frame_rgb = self.current_frame

			else:
				normalised_image = image_processing.patch_normalise_image(full_image, (9,9), resize=self.resize)

				match_data = [[] for i in range(len(self.ref_rgb))]
				for i in range(len(self.ref_rgb)):
					if i == 10:
						offset, corr, debug_image = image_processing.xcorr_match_images_debug(self.ref_rgb[i], normalised_image, 1)
						match_data[i] = (offset, corr)
					else:
						match_data[i] = image_processing.xcorr_match_images(self.ref_rgb[i], normalised_image, 1)

				offsets_data = np.ascontiguousarray([int(match[0]) for match in match_data])
				# offsets_deg = np.ascontiguousarray([int(match[0]) * 69.636 / 115 for match in match_data])
				correlation_data = np.ascontiguousarray([match[1] for match in match_data])
				time_stamp = rospy.Time.now().to_sec()

				if self.state < 4:
					id = '%06d'%self.rgb_count
					best_match = np.argmax(correlation_data)
					offset_deg = offsets_data[best_match] * 69.636 / 115
					print('RGB: %f'%offset_deg)
					
					debug_image = image_processing.visualize(np.uint8(255.0 * (1 + normalised_image) / 2.0), np.uint8(255.0 * (1 + self.ref_rgb[best_match]) / 2.0), offsets_data[best_match]+normalised_image.shape[1]//2)
					cv2.imwrite(self.rgb_save_dir + id + '.png', debug_image)
					message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
					with open(self.rgb_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
						pose_file.write(message_as_text)
					message_as_text = json.dumps({'time_stamp': time_stamp, 'theta_offset': offset_deg, 'time_at_recv': time_at_recv})
					with open(self.rgb_offset_correction+'%06d_correction.txt' % self.event_count, 'w') as correction_file:
						correction_file.write(message_as_text)
					
					self.rgb_count += 1


	# def process_rgb_frames(self, msg):
	# 	if self.odom_ready is True:
	# 		n = msg.header.seq
	# 		time_at_recv = rospy.Time.now().to_sec()

	# 		full_image = ros_numpy.numpify(msg)

	# 		if self.teach_mode is True:
	# 			self.diff_last_curr_rgb = self.last_frame_rgb.Inverse() * self.current_frame
	# 			id = '%06d'%self.rgb_count
	# 			if self.rgb_count == 0 or self.diff_last_curr_rgb.p.Norm() > 0.1:
	# 				cv2.imwrite(self.rgb_save_dir + id + '.png', full_image)
	# 				message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
	# 				with open(self.rgb_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
	# 					pose_file.write(message_as_text)
	# 				self.rgb_count += 1
	# 				self.last_frame_rgb = self.current_frame

	# 		else:
	# 			normalised_image = image_processing.patch_normalise_image(full_image, (9,9), resize=self.resize)

	# 			match_data = [[] for i in range(len(self.ref_rgb))]
	# 			for i in range(len(self.ref_rgb)):
	# 				if i == 10:
	# 					offset, corr, debug_image = image_processing.xcorr_match_images_debug(self.ref_rgb[i], normalised_image, 1)
	# 					match_data[i] = (offset, corr)
	# 				else:
	# 					match_data[i] = image_processing.xcorr_match_images(self.ref_rgb[i], normalised_image, 1)

	# 			offsets_data = np.ascontiguousarray([int(match[0]) for match in match_data])
	# 			# offsets_deg = np.ascontiguousarray([int(match[0]) * 69.636 / 115 for match in match_data])
	# 			correlation_data = np.ascontiguousarray([match[1] for match in match_data])
	# 			time_stamp = rospy.Time.now().to_sec()

	# 			if self.state < 4:
	# 				id = '%06d'%self.rgb_count
	# 				best_match = np.argmax(correlation_data)
	# 				offset_deg = offsets_data[best_match] * 69.636 / 115
	# 				print('RGB: %f'%offset_deg)
					
	# 				debug_image = image_processing.visualize(np.uint8(255.0 * (1 + normalised_image) / 2.0), np.uint8(255.0 * (1 + self.ref_rgb[best_match]) / 2.0), offsets_data[best_match]+normalised_image.shape[1]//2)
	# 				cv2.imwrite(self.rgb_save_dir + id + '.png', debug_image)
	# 				message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(tf_conversions.toMsg(self.current_frame)))
	# 				with open(self.rgb_pose_save_dir+id+'_pose.txt', 'w') as pose_file:
	# 					pose_file.write(message_as_text)
	# 				message_as_text = json.dumps({'time_stamp': time_stamp, 'theta_offset': offset_deg, 'time_at_recv': time_at_recv})
	# 				with open(self.rgb_offset_correction+'%06d_correction.txt' % self.event_count, 'w') as correction_file:
	# 					correction_file.write(message_as_text)
					
	# 				self.rgb_count += 1


	def process_odom_data(self, msg):
		if self.current_frame is None:
			self.start_frame = tf_conversions.fromMsg(msg.pose.pose)
			self.last_frame_rgb = tf_conversions.fromMsg(msg.pose.pose)
			self.last_frame_rgb = self.start_frame.Inverse() * self.last_frame_rgb
			self.last_frame_event = tf_conversions.fromMsg(msg.pose.pose)
			self.last_frame_event = self.start_frame.Inverse() * self.last_frame_event

		self.mutex.acquire()
		self.current_frame = tf_conversions.fromMsg(msg.pose.pose)
		self.current_frame = self.start_frame.Inverse() * self.current_frame
		self.odom_ready = True
		self.mutex.release()

		if self.teach_mode is False:
			if self.last_state_frame is None:
				self.last_state_frame = self.current_frame
			diff_last_current_state = self.last_state_frame.Inverse() * self.current_frame

			# strsight 0.5 metres
			if self.state == 0:
				if diff_last_current_state.p.x() <= 0.5:
					motor_command = Twist()
					motor_command.linear.x = 0.2
					motor_command.angular.z = 0
					self.pub_cmd_vel.publish(motor_command)
				else:
					print("End of state 0, moved X = %f"  % diff_last_current_state.p.x())
					self.state = 1
					self.last_state_frame = None

			elif self.state == 1:
				# strsight 0.5 metres
				if diff_last_current_state.M.GetRPY()[2] <= 3 * (math.pi/180):
					motor_command = Twist()
					motor_command.linear.x = 0.2
					motor_command.angular.z = 0.1
					self.pub_cmd_vel.publish(motor_command)
				else:
					print("End of state 1, moved M_z = %f"  % diff_last_current_state.M.GetRPY()[2])
					self.state = 2
					self.last_state_frame = None

			elif self.state == 2:
				# strsight 0.5 metres
				if diff_last_current_state.p.x() <= 0.5:
					motor_command = Twist()
					motor_command.linear.x = 0.2
					motor_command.angular.z = 0.0
					self.pub_cmd_vel.publish(motor_command)
				else:
					print("End of state 2, moved X = %f"  % diff_last_current_state.p.x())
					time_stamp = rospy.Time.now().to_sec()
					message_as_text = json.dumps({'time_stamp': time_stamp})
					with open(self.save_dir + ('start_time.txt'), 'w') as start_file:
						start_file.write(message_as_text)
					self.state = 3

			elif self.state == 3:
				# strsight 0.5 metres
				if diff_last_current_state.p.x() <= 0.5:
					motor_command = Twist()
					motor_command.linear.x = 0.2
					motor_command.angular.z = 0.0
					self.pub_cmd_vel.publish(motor_command)
				else:
					print("End of state 3, moved X = %f"  % diff_last_current_state.p.x())
					self.state = 4

			elif self.state == 4:
				motor_command = Twist()
				motor_command.linear.x = 0.0
				motor_command.angular.z = 0.0
				self.pub_cmd_vel.publish(motor_command)


if __name__ == "__main__":
	rospy.init_node("drive_straight_controller")
	controller = drive_straight_controller()
	rospy.spin()

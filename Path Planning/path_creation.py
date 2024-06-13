import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from .pathPlanning import pathPlanning
from .pathConfig import origin, pickup, dropoff, obstacle, offset, cardyn


class PathCreationNode(Node):

	def __init__(self):
		
		# Define the nodes name
		super().__init__("pathCreation_node")
		
		# Tell user node has started
		self.get_logger().info("Path Creation Node has been started.")
		
		# Define the messaged recieved flag
		self.message_received = False
		
		# Define the subscriber
		self.object_sub_ = self.create_subscription(String, "/detected", self.object_callback, 10)
		
		
	def object_callback(self, msg: String):
	
		self.object_type = msg.data
		
		self.get_logger().info("Detected: " + msg.data)
		
		self.message_received = True
		
		
	def wait_for_messages(self):
		
		while not self.message_received and rclpy.ok():
			
			# spin node once to check for message
			rclpy.spin_once(self)
			
	
	def create_path(self):
		
		# Call the pathPlanning function
		GPS, interp, path_local = pathPlanning(origin(), pickup(), dropoff(self.object_type), obstacle(), offset(), cardyn())
		
		# Convert the coords to csv and stor in desired path
		file_path = "/home/projects/ros2_ws/src/ucsd_robocar_hub2/pathing/coords/GPS.csv"
		np.savetxt(file_path, GPS, delimiter=",")
		
		# Inform user path creation was successful
		self.get_logger().info("Path creation was successful.")
			
				
def main(args=None):
    rclpy.init(args=args)
    node = PathCreationNode()
    node.wait_for_messages()
    node.create_path()
    rclpy.shutdown()
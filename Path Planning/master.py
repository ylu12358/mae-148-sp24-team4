import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

# Define ObjectDetectionNode as publisher (will send info to PathCreationNode)
class ObjectDetectionNode(Node):
	def __init__(self):

		# Define the nodes name
		super().__init__("objectDetection_node")

		# Define published data
		self.object_pub_ = self.create_publisher(String, "/detected", 10)
		
		# Inform user that detection has started
		self.get_logger().info("Object Detection Node has been started.")
		
		# instantiating an object (rf) with the RoboflowOak module
		self.rf = RoboflowOak(model="hard-hat-sample-tk2pm", confidence=0.05, overlap=0.5, version="1", api_key="Kq55iT9KOAwtcOUiZV9Z", rgb=True, depth=True, device=None, blocking=True)

		while True:
		
			# Running our model and displaying the video output with detections
			t0 = time.time()
			# The rf.detect() function runs the model inference
			result, frame, raw_frame, depth = self.rf.detect()
			predictions = result["predictions"]
			#{
			#    predictions:
			#    [ {
			#        x: (middle),
			#        y:(middle),
			#        width:
			#        height:
			#        depth: ###->
			#        confidence:
			#        class:
			#        mask: {
			#    ]
			#}
			#frame - frame after preprocs, with predictions
			#raw_frame - original frame from your OAK
			#depth - depth map for raw_frame, center-rectified to the center camera
			
			# timing: for benchmarking purposes
			t = time.time()-t0

			# displaying the video feed as successive frames
			#cv2.imshow("frame", frame)
		
			msg = String()
			con = 0.13
			if predictions:
				print("PREDICTIONS ", predictions[0].json())
				if predictions[0].confidence > np.float64(con):
					msg.data = predictions[0].class_name
					self.object_pub_.publish(msg)
					break

# OOP
def main(args=None):

	# Initialize ros2 features
	rclpy.init(args=args)

	# Create the node
	node = ObjectDetectionNode()

	# Shut down node and ros2 features
	rclpy.shutdown()

# Allow for call directly from command line (not necessary)
if __name__ == '__main__':
	main()
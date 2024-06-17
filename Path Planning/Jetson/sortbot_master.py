from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
from pathPlanning import pathPlanning
from functions import plotEnvironment
from pathConfig import angle, origin, dropoff, obstacle, offset, cardyn

# OOP
def main(args=None):

  print('Object detection process has started!')
  # Define confidence target
  confidence_targ = 0.85
  
  # Initialize roboflow model
  rf = RoboflowOak(model="mae-148-cv", confidence=0.05, overlap=0.5, version="1", api_key="Kq55iT9KOAwtcOUiZV9Z", rgb=True, depth=True, device=None, blocking=True)
  # Initialize empty variables
  confidence_curr = 0
  object_recog = ''
  while confidence_curr <= np.float64(confidence_targ):
    # Running our model and displaying the video output with detections
    t0 = time.time()
    # The rf.detect() function runs the model inference
    result, frame, raw_frame, depth = rf.detect()
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
    
    if predictions:
      confidence_curr = predictions[0].confidence
      print("PREDICTIONS ", predictions[0].json())
      object_recog = predictions[0].class_name
      
  # Generate path
  GPS, interp, path_local = pathPlanning(angle(), origin(), np.array([0, 0]), dropoff(object_recog), obstacle(), offset(), cardyn())
    
  # Convert the coords to csv and stor in desired path
  file_path = "/home/jetson/projects/mycar/sortbotpath.csv"
  np.savetxt(file_path, GPS, delimiter=", ", fmt='%f')
  print('Path created succesfully!')
  
  time.sleep(5)

# Allow for call directly from command line (not necessary)
if __name__ == '__main__':
  main()

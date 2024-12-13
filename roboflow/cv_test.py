from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    advanced_config = {
    	"nn_mode": "device"
    }
    #"resolution": "720p",
    rf = RoboflowOak(model="detect-car-jsvsk", confidence=0.65, overlap=0.5,
    version="4", api_key="7CfAEQ0li4t5GeTTqLXK", rgb=True,
    depth=True, device=None, blocking=True, advanced_config=advanced_config)
    # Running our model and displaying the video output with detections
    while True:
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
        print("FPS ", 1/t)
        #print("PREDICTIONS ", [p.json() for p in predictions])
        if np.shape(frame)[0] == 0:
        	continue

        # setting parameters for depth calculation
        # comment out the following 2 lines out if you're using an OAK without Depth
        #frame_height, frame_width = frame.shape[:2]
        #depth_resized = cv2.resize(depth, (frame_width, frame_height), interpolation=cv2.INTER_NEAREST)
	
        #max_depth = np.amax(depth)
        #cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        print(np.shape(frame))
        
        cv2.imshow("frame", frame)
    
        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
        if cv2.waitKey(1) == ord('q'):
            break

from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    advanced_config = {
    	"nn_mode": "host",
    	"resolution": "416x416"
    }
    rf = RoboflowOak(model="face-detection-mik1i", confidence=0.5,
    overlap=0.5, version="7", api_key="7CfAEQ0li4t5GeTTqLXK", rgb=True,
    depth=False, device=None, device_name="", blocking=True, advanced_config=advanced_config)
    while True:
        t0 = time.time()
        result, frame, raw_frame, depth = rf.detect()
        predictions = result["predictions"]
        #{
        #    predictions:
        #    [ { 
        #        x: (middle),
        #        y:(middle),
        #        width: ,
        #        height: ,
        #        depth: ###->,
        #        confidence: ,
        #        class: ,
        #        mask: { }
        #       }
        #    ]
        #}
        #frame - frame after preprocs, with predictions
        #raw_frame - original frame from your OAK
        #depth - depth map for raw_frame, center-rectified to the center camera
        #To access specific values within "predictions" use: [p.json() for p[a] in predictions]
        # set "a" to the index value you are attempting to access
        # Example: accessing the "y"-value: [p.json() for p[1] in predictions]
    
        t = time.time()-t0
        print("INFERENCE TIME IN MS ", 1/t)
        #print("PREDICTIONS ", [p.json() for p in predictions])
        print(np.shape(frame))
    
        # setting parameters for depth calculation
        #max_depth = np.amax(depth)
        #print(np.shape(depth))
        #cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        cv2.imshow("frame", frame)
    
        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
        if cv2.waitKey(1) == ord('q'):
            break

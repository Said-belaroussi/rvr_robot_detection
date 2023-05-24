import depthai_sdk
import cv2

# Download & deploy a model from Roboflow universe:
# # https://universe.roboflow.com/david-lee-d0rhs/american-sign-language-letters/dataset/6

with depthai_sdk.OakCamera() as oak:
    color = oak.create_camera('color', fps=15)
    model_config = {
        'source': 'roboflow', # Specify that we are downloading the model from Roboflow
        'model':'rvr_detection_v2/1',
        'key':'g0WaLaIt5ndRvBhoVPOh' # Fake API key, replace with your own private key!
        # You need to have a Roboflow account to get a key.
    }
    
    nn = oak.create_nn(model_config, color)
    oak.visualize(nn, fps=True)
    oak.start(blocking=True)

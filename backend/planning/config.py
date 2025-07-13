ROBOT_FILE = "apt68.xml" 
ROBOT_SCENE = "../robot/" + ROBOT_FILE
DOMAIN_ID = 1 # Domain id, using 1 for simulation
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
SIMULATE_DT = 0.002  # Match the timestep in the XML file
VIEWER_DT = 0.02  # 50 fps for viewer

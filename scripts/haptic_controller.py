# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import sys
import signal
import rospy
import carla
from typing import Literal
from std_msgs.msg import Int16, String
from haptic_carla_pkg.carla_client import CarlaClient



# ==============================================================================
# -- Global Parameters ---------------------------------------------------------
# ==============================================================================

autopilot_mode = True



# ==============================================================================
# -- Global Functions ----------------------------------------------------------
# ==============================================================================

def steerCb(msg):
    """Apply steering control to ego-vehicle based on ROS messages"""
    
    try:
        angle = (msg.data / 511.5) - 1
        angle = angle / 5
        env.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=angle))
    
    except:
        pass



def signal_handler(signal, frame):
    """Handles interrupt from Keyboard (Ctrl + C)"""

    sys.exit()



def connect_to_server() -> CarlaClient:
    """Trying to establish connection with CARLA server"""

    print("\nConneting To Simulator ...\n")

    signal.signal(signal.SIGINT, signal_handler)

    i = 1
    while True:
        try:
            env = CarlaClient()
            print("\n" + u'\u2713'*3 + " Connected Successfully")
            break

        except RuntimeError:
            print(u'\u2715' + f" Connection failed, trying again: {i}")
            i += 1
    
    return env



def modeCb(msg) -> None:
    """Switch ego-vehicle control mode between CARLA BasicAgent and User Manual Contorl"""
    
    global autopilot_mode
    
    try: 
        if msg.data == 'auto':
            print("\n>>> Autopilot Mode")
            autopilot_mode = True
        
        elif msg.data == 'manual':
            print("\n>>> Manual Mode")
            autopilot_mode = False

    except AttributeError:
        pass



# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    try:
        global env

        # ROS Declarations
        rospy.init_node('haptic_controller_node', anonymous=True)
        rospy.Subscriber('/steer_cmd', Int16, steerCb)
        rospy.Subscriber('/control_mode', String, modeCb)

        env = None
        env = connect_to_server()
        env.reset() 
        
        while not rospy.is_shutdown():
            
            if autopilot_mode:
                Agent_ctrl = env.agent.run_step()
                env.ego_vehicle.apply_control(Agent_ctrl)
                # print(f"Angle: {Agent_ctrl.steer}")

            else:
                man_ctrl = carla.VehicleControl(throttle=0, steer=0, brake=1.0, hand_brake=True)
                env.ego_vehicle.apply_control(man_ctrl)
    
        rospy.spin()


    except KeyboardInterrupt:
        if env != None:   
            env.destroy_actors()
        print('\n>>> Cancelled by user. Bye!\n')

    finally:
        if env != None:
            env.destroy_actors()
        print('\n>>> Cancelled by user. Bye!\n')



# ==============================================================================
# -- Main ----------------------------------------------------------------------
# ==============================================================================
        
if __name__ == '__main__':
    main()
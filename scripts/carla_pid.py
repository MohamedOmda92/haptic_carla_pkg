# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import sys
import signal
import rospy
import carla
from std_msgs.msg import Int16
from haptic_carla_pkg.carla_client import CarlaClient



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



# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    try:
        global env
        
        # ROS Declarations
        rospy.init_node('carla_pid', anonymous=True)
        rospy.Subscriber('/steer_cmd', Int16, steerCb)

        env = None
        env = connect_to_server()
        env.reset() 
        
        while not rospy.is_shutdown():
            control = env.agent.run_step()
            env.ego_vehicle.apply_control(control)
            print(f"Angle: {control.steer}")
    
        rospy.spin()


    except KeyboardInterrupt:
        if env != None:   
            env.destroy_actors()
        print('\nCancelled by user. Bye!\n')

    finally:
        if env != None:
            env.destroy_actors()
        print('\nCancelled by user. Bye!\n')



# ==============================================================================
# -- Main ----------------------------------------------------------------------
# ==============================================================================
        
if __name__ == '__main__':
    main()
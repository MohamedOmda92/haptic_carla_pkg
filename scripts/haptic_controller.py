# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import os
import sys
import signal
import rospy
import carla
from std_msgs.msg import Int16, String
from haptic_carla_pkg.carla_client import CarlaClient

pkg_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir))
dataset_dir = os.path.join(pkg_dir, 'dataset/images')



# ==============================================================================
# -- Global Parameters ---------------------------------------------------------
# ==============================================================================

ros_mode = True
autopilot_mode = True
dataset_size = 100



# ==============================================================================
# -- Global Functions ----------------------------------------------------------
# ==============================================================================

def delete_files_in_directory(directory_path):
   try:
     files = os.listdir(directory_path)
     for file in files:
       file_path = os.path.join(directory_path, file)
       if os.path.isfile(file_path):
         os.remove(file_path)
     print("\nAll files deleted successfully.")
   except OSError:
     print("Error occurred while deleting files.")



def signal_handler(signal, frame):
        """
        Handles interrupt from Keyboard (Ctrl + C)"""
        sys.exit()



def steerCb(msg):
    """
    Apply steering control to ego-vehicle based on ROS messages"""
    try:
        angle = (msg.data / 511.5) - 1
        angle = angle / 5
        env.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=angle))
    except:
        pass



def modeCb(msg) -> None:
    """
    Switch ego-vehicle control mode between CARLA BasicAgent and User Manual Contorl"""
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

    global env, dataset_size

    if autopilot_mode:
        Agent_ctrl = env.agent.run_step()
        env.ego_vehicle.apply_control(Agent_ctrl)        

    else:
        man_ctrl = carla.VehicleControl(throttle=0, steer=0, brake=1.0, hand_brake=True)
        env.ego_vehicle.apply_control(man_ctrl)

    if env.img_num > dataset_size-1:
        dataset_size = env.img_num + 1
        CarlaClient.form_dataset = False
        print("Dataset Size Reached")        
        env.generate_dataset_labels()



def haptic_controller():
    
    delete_files_in_directory(dataset_dir)

    try:
        global env

        # Carla Client Initialization
        CarlaClient.form_dataset = True
        env = CarlaClient()
        env.reset()
        
        if ros_mode:
            # ROS Declarations
            rospy.init_node('haptic_controller_node', anonymous=True)
            rospy.Subscriber('/steer_cmd', Int16, steerCb)
            rospy.Subscriber('/control_mode', String, modeCb)

            while not rospy.is_shutdown():
                main()

            rospy.spin()

        else:
            signal.signal(signal.SIGINT, signal_handler)

            while True:
                main()

    except KeyboardInterrupt:
        if env != None:   
            env.destroy_actors()
        print('\n>>> Cancelled by user. Bye!\n')

    finally:
        if env != None:
            env.destroy_actors()
        print('\n>>> Cancelled by user. Bye!\n')


     
if __name__ == '__main__':

    try:
        arg = rospy.get_param('haptic_controller_node/ignore-ros')

    except ConnectionRefusedError:
        ros_mode = False

    haptic_controller()
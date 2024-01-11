# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import glob
import os
import sys
import signal
import random
import time
import math
import rospy
import numpy as np
from std_msgs.msg import Int16, Float32
from simple_pid import PID
import matplotlib.pyplot as plt

try:
    sys.path.append('/home/mohamed/CARLA_0.9.11/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')
    sys.path.append('/home/mohamed/CARLA_0.9.11/PythonAPI/carla')
    
except IndexError:
    pass  

    
import carla
from agents.navigation.basic_agent import BasicAgent


# ===========================================================================
# -- Global Params ----------------------------------------------------------
# ===========================================================================

KP = 1.2
KI = 0
KD = 0
TIMESTEP = 0.01


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

"""class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))"""


# ==============================================================================
# -- carla-env Class -----------------------------------------------------------
# ==============================================================================

class CarlaPID(): 
    
    
    def __init__(self, default_spawn = False, min_map=False):
        
        self.default_spawn = default_spawn

        # PID Variables  
        self.time = 0
        self.time_prev = -1e-6
        self.integral = 0
        self.e_prev = 0

        # Connect to Carla server
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        print("\nCONNECTED!")

        # Set world map
        self.world = client.get_world()
        if min_map:
            current_map = self.world.get_map().name            
            if not "_Opt" in current_map:
                print(f"\nLoading minimum version of current map ({current_map})")
                print("This may take few seconds ...")
                client.set_timeout(60.0)
                self.world = client.load_world(current_map+"_Opt")
                self.world.unload_map_layer(carla.MapLayer.Buildings) 
                self.world.unload_map_layer(carla.MapLayer.Foliage)
                self.world.unload_map_layer(carla.MapLayer.StreetLights)   
                print("MAP LOADED SUCCESSFULLY!")          

            self.blueprint_library = self.world.get_blueprint_library()
            self.model3 = self.blueprint_library.filter("model3")[0]


    
    def reset(self):
        """This method intialize world for new epoch"""
        self.collision_hist = []
        self.actor_list = []
        self.lane_list = []
        
        # Spawn Ego-vehicle 
        self.map = self.world.get_map()
        spawn_points = self.map.get_spawn_points()
        self.transform = random.choice(spawn_points)

        if self.default_spawn:
            self.spawn_at_default()
        
        self.transform.rotation.yaw += random.randint(-45,45) # Random Orientation
        self.ego_vehicle = self.world.spawn_actor(self.model3, self.transform)
        self.ego_vehicle.role_name = "ego_vehicle"
        self.actor_list.append(self.ego_vehicle)

        # Set spectator navigation
        self._set_spectator()
        
        # Turn Ego-vehicle into Basic Agent
        self.agent = BasicAgent(self.ego_vehicle)
        self.agent.ignore_traffic_lights(active=True)
        

        # Spawn and attach collision sensor to ego_vehicle
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        colSensor = self.blueprint_library.find('sensor.other.collision')
        self.col_sensor = self.world.spawn_actor(colSensor, transform, attach_to=self.ego_vehicle)
        self.actor_list.append(self.col_sensor)
        self.col_sensor.listen(lambda event: self._collision_data(event))
        


    def get_lateral_error(self):
        
        ego_transform = self.ego_vehicle.get_location()
        ego_x = ego_transform.x
        ego_y = ego_transform.y
        
        nearest_wpt = self.map.get_waypoint(ego_transform, project_to_road=True)
        center_x = nearest_wpt.transform.location.x
        center_y = nearest_wpt.transform.location.y
        center_yaw = nearest_wpt.transform.rotation.yaw

        lateral_err = math.sqrt(((ego_x-center_x)**2) + ((ego_y-center_y)**2)) # Un-signed error distance
        
        lateral_angle = math.atan2((center_y-ego_y), (center_x-ego_x))*180/math.pi
        
        if lateral_angle < 0:
            lateral_angle += 360

        if center_yaw < 180:
            lb = center_yaw
            ub = center_yaw + 180
            if lateral_angle > lb and lateral_angle < ub:
                lateral_err *= -1 # LEFT
            else:
                pass # RIGHT
        
        if center_yaw > 180:
            ub = center_yaw
            lb = center_yaw - 180
            if lateral_angle > lb and lateral_angle < ub:
                pass # RIGHT
            else:
                lateral_err *= -1 # LEFT

        return lateral_err
        
    

    def PID_controller(self, Kp, Ki, Kd, measurement, setpoint=0):
        
        # PID calculations
        e = setpoint - measurement

        Proportional = Kp*e
        Integral = self.integral + Ki*e*(self.time - self.time_prev)
        print(f"Delta error: {e - self.e_prev}")
        print(f"Delta time: {self.time-self.time_prev}")
        Derivative = Kd*(e - self.e_prev)/(self.time - self.time_prev)

        # calculate manipulated variable - MV
        val = Proportional + Integral + Derivative

        # update stored data for next iteration
        self.e_prev = e
        self.time_prev = self.time
        self.integral = Integral

        return val



    def destroy_actors(self):

        for actor in self.actor_list:
            actor.destroy()

        print("All Actors Destroyed")



    def _set_spectator(self):
        
        self.spectator = self.world.get_spectator()
        specTrans = self.ego_vehicle.get_transform()
        specTrans.location.z += 3 # ABOVE vehicle
        specTrans.rotation.pitch -= 30 
        self.spectator.set_transform(specTrans)



    def _collision_data(self, event):
            """Displays impulse of collision / Resets the environment for new epoch

            Args:
                event (carla.CollisionEvent):  collision data retrieved by collision detector
            """
            self.collision_hist.append(event)

    

    def spawn_at_default(self):
        
        self.transform.location.x = 50.384322 # Decrese = Forward in the road
        self.transform.location.y = 193.564453
        self.transform.location.z = 0.275307
        self.transform.rotation.pitch = 0
        self.transform.rotation.yaw = 180
        self.transform.rotation.roll =  0
        

        
def steerCb(msg):

    try:
        angle = (msg.data / 511.5) - 1
        angle = angle / 5
        env.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=angle))
    
    except:
        pass



# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    try:
        global env
        rospy.init_node('carla_pid', anonymous=True)
        pub = rospy.Publisher('/err', Float32, queue_size=10)
        rospy.Subscriber('/steer_cmd', Int16, steerCb)

        msg = Float32()

        env = CarlaPID(min_map=True)
        env.reset() 
        max = 0
        err_lst = np.array([])
        
        while not rospy.is_shutdown():

            env.ego_vehicle.apply_control(env.agent.run_step())
        #     err = env.get_lateral_error()
        #     msg.data = err
        #     # pub.publish(msg)
        #     err_lst = np.append(err_lst, err)
        #     print(f"Current Error: {round(err*100)} Cm")

        #     env.time = env.time * TIMESTEP
        #     pid_controller = PID(KP, KI, KD, setpoint=0)
        #     pidVal = pid_controller(err)
            
        #     if pidVal > max:
        #         max = pidVal
        #         # print("Max Steering Mag: {:.3f}".format(pidVal))
            
        #     env.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=pidVal))
            
        #     if len(env.collision_hist) != 0 or err > 1.1:
        #         env.destroy_actors()
        #         env.reset()

        #     time.sleep(TIMESTEP)

        
        ## Plot Performance
        # fig, ax = plt.subplots()
        # ax.plot(err_lst)
        # ax.grid()
        # fig.savefig("/home/mohamed/Desktop/response.png")

        rospy.spin()


    except KeyboardInterrupt:
        
        env.destroy_actors()
        print('\nCancelled by user. Bye!')

    finally:
        env.destroy_actors()
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
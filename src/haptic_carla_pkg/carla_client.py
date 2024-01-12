# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import random
import carla
from typing import Literal
from agents.navigation.basic_agent import BasicAgent



# ==============================================================================
# -- CarlaClient Class ---------------------------------------------------------
# ==============================================================================

class CarlaClient():

    def __init__(self, min_map=False):
        
        self.default_spawn = False

        # Connect to Carla server
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)

        # Set world map
        self.world = client.get_world()
        
        if min_map:
            current_map = self.world.get_map().name            
            
            # Sets minimal-version map
            if not "_Opt" in current_map:
                print(f"\nLoading minimum version of current map ({current_map})")
                print("This may take few seconds ...")
                client.set_timeout(60.0)
                self.world = client.load_world(current_map+"_Opt")
                self.world.unload_map_layer(carla.MapLayer.Buildings) 
                self.world.unload_map_layer(carla.MapLayer.Foliage)
                self.world.unload_map_layer(carla.MapLayer.StreetLights)   
                print("\n" + u'\u2713'*3 + " Loaded Successfully!")          

        self.blueprint_library = self.world.get_blueprint_library()
        self.model3 = self.blueprint_library.filter("model3")[0]


    
    def reset(self):
        """This method intialize world for new epoch"""
        self.collision_hist = []
        self.actor_list = []
        self.lane_list = []
        
        # Spawn Ego-vehicle 
        map = self.world.get_map()
        spawn_points = map.get_spawn_points()
        self.transform = random.choice(spawn_points)

        if self.default_spawn:
            self._spawn_at_default()
        
        self.ego_vehicle = self.world.spawn_actor(self.model3, self.transform)
        self.ego_vehicle.role_name = "ego_vehicle"
        self.actor_list.append(self.ego_vehicle)

        # Set spectator navigation
        self._set_spectator()
        
        # Turn Ego-vehicle into Basic Agent
        self.agent = BasicAgent(self.ego_vehicle)
        self.agent.ignore_traffic_lights(active=True)

        # Spawn and attach collision sensor to ego_vehicle
        col_sensor = self._spawn_ego_sensor(self.ego_vehicle, "collision")
        col_sensor.listen(lambda event: self._collision_data(event))
        
        

    def destroy_actors(self) -> None:
        """Destroy all actors in the actor_list"""

        for actor in self.actor_list:
            actor.destroy()

        print("\nAll Actors Destroyed")



    def _set_spectator(self) -> None:
        """Focuses world spectator at ego vehilce"""
        
        spect = self.world.get_spectator()

        trans = self.ego_vehicle.get_transform()
        trans.location.z += 3 # ABOVE vehicle
        trans.rotation.pitch -= 30 # LOOK down

        spect.set_transform(trans)


    
    def _spawn_ego_sensor(self, parent: carla.Actor, 
                          type: Literal['rgb_cam','collision']) -> carla.Actor:
        """Spawn sensor and attach it to ego-vehicle"""

        loc = parent.get_location()
        trans = carla.Transform(location=loc)

        # Sensor Type Selection
        if type == 'rgb_cam':
            bp = 'sensor.camera.rgb'
        elif type == 'collision':
            bp = 'sensor.other.collision'
            
        sensorBp = self.blueprint_library.find(bp)
        sensor = self.world.spawn_actor(sensorBp, trans, attach_to=parent)
        self.actor_list.append(sensor)

        return sensor



    def _collision_data(self, event) -> None:
        """Appends collision event to history list"""

        self.collision_hist.append(event)

    

    def _spawn_at_default(self) -> None:
        """Spawn actor at defined default location"""

        self.transform.location.x = 50.384322 # Decrese = Forward in that road
        self.transform.location.y = 193.564453
        self.transform.location.z = 0.275307
        self.transform.rotation.pitch = 0
        self.transform.rotation.yaw = 180
        self.transform.rotation.roll =  0
        


# ==============================================================================
# -- Main ----------------------------------------------------------------------
# ==============================================================================
          
if __name__ == "__main__":
    client = CarlaClient()

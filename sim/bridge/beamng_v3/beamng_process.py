import math
import time
import numpy as np

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection

# from metadrive.engine.core.engine_core import EngineCore
# from metadrive.engine.core.image_buffer import ImageBuffer
# from metadrive.envs.metadrive_env import MetaDriveEnv
# from metadrive.obs.image_obs import ImageObservation


from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics,Camera,GPS


from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H


C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)
MAX_STEERING = 6  #TODO make this configurable


beamng_simulation_state = namedtuple("beamng_simulation_state", ["running", "done", "done_info"])
beamng_vehicle_state = namedtuple("beamng_vehicle_state", ["velocity", "position", "bearing", "steering_angle"])


def process_controls(vehicle,control_q):
  while True:
      try:
        (steering,throttle,brake)=control_q.get()
        print("recv: ", steering, throttle, brake)
        vehicle.control(steering=steering,throttle=throttle,brake=brake)
      except Exception as e:
        print("control recv exception :",e)

def process_sensors(vehicle,sensors_q):
   while True:
    try:
      vehicle.sensors.poll()
      velocity = vehicle.state['vel']
      position = vehicle.state['pos']
      el2 = vehicle.sensors['ele']
      steering = el2.get('steering')
      # steering = el2.get('steering_input')
      # print(el2.get('steering_input'),el2.get('steering'))
      print((velocity,position,steering,float(0)))
      sensors_q.put((velocity[0],velocity[1],velocity[2],position[0],position[1],steering,float(0)))
    except Exception as e:
      print("sensor send exception:",e)


def process_camera(dash_cam,camera_array,image_lock):
  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  while True:

    try:
      # imgArr = np.asarray(dash_cam.stream()['colour'].convert('RGB'))
      color = dash_cam.poll()['colour']
      if color is not None:
        imgArr=np.asarray(color.convert('RGB'))
        print("_____________")
        print(imgArr.shape)
        print(imgArr)
        print("_____________")
        road_image [...]= imgArr
      # camera_send.send(imgArr)
    except Exception as e:
      print("camera send exception:",e)
    image_lock.release()
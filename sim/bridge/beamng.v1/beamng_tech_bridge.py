import math
from multiprocessing import Queue

# from metadrive.component.sensors.base_camera import _cuda_enable
# from metadrive.component.map.pg_map import MapGenerateMethod

from openpilot.tools.sim.bridge.common import SimulatorBridge
# from openpilot.tools.sim.bridge.metadrive.metadrive_common import RGBCameraRoad, RGBCameraWide
from openpilot.tools.sim.bridge.beamng.beamng_tech_world import BeamNGTechWorld
from openpilot.tools.sim.lib.camerad import W, H


import csv
import pathlib
import time

import math

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera,Electrics,GPS

import cv2 as cv
import numpy as np




class BeamNGTechBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False):
    super().__init__(dual_camera, high_quality)

    self.should_render = False
    self.test_run = test_run
    self.test_duration = test_duration if self.test_run else math.inf

  def spawn_world(self, queue: Queue):

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:64256
    bng = BeamNGpy(host='172.30.112.1', port=64256,home="/mnt/c/Program Files/BeamNG.tech.v0.32.2.0",quit_on_close=True,)
    # Launch BeamNG.tech
    bng.open(launch=False)
    # Create a scenario in west_coast_usa called 'example'
    scenario = Scenario('west_coast_usa', 'example')
    # Create an ETK800 with the licence plate 'PYTHON'
    vehicle = Vehicle('ego_vehicle', model='etk800', license='PYTHON')
    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()



    vehicle.ai.set_mode('disabled')
    # vehicle.ai_set_aggression(0)
    # vehicle.ai.set_target(target_vehicle.vid,mode="follow")
    # vehicle.ai_drive_in_lane(True)


    # vehicle.attach_sensor(sensor=Electrics(),name="el2")

    resolution=(W,H)
    # resolution=(1920,1080)

    # vehicle.recover()
    dash_cam = Camera(name="dash_cam",
                      vehicle=vehicle,
                      bng=bng,pos=(0,-1.5,1),
                      is_streaming=False,
                      is_render_colours=True,
                      is_using_shared_memory=False,
                      is_render_instance=True,
                      resolution=resolution)
    GPS(name="gps",bng=bng,vehicle=vehicle)
    vehicle.attach_sensor(sensor=Electrics(),name="ele")
    # vehicle.attach_sensor(sensor=GPS(),name="gps")







    sensors = {
      "dash_cam": (dash_cam, W, H, )
    }

    # if self.dual_camera:
    #   sensors["rgb_wide"] = (RGBCameraWide, W, H)

    config = dict(
      use_render=self.should_render,
      vehicle_config=dict(
        enable_reverse=False,
        image_source="dash_cam",
      ),
      sensors=sensors,
      vehicle=vehicle,
      traffic_density=0.0, # traffic is incredibly expensive
    )

    return BeamNGTechWorld(queue, config, self.test_duration, self.test_run, self.dual_camera)

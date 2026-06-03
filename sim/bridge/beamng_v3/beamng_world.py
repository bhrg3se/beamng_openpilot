import ctypes
import functools
import multiprocessing
import threading

import numpy as np
import time

from multiprocessing import Pipe, Array, SimpleQueue

from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType
from openpilot.tools.sim.bridge.beamng_v3.beamng_process import process_controls,process_camera,process_sensors
from openpilot.tools.sim.bridge.beamng_v3.buffer import DoubleBuffer
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import vec3

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics,Camera,GPS



class BeamNGWorld(World):
  def __init__(self, status_q, config, test_duration, test_run, dual_camera=False):
    super().__init__(dual_camera)
    self.status_q = status_q
    self.config =config
    self.test_duration = test_duration
    self.test_run=test_run
    self.is_engaged=True
    self.camera_array = Array(ctypes.c_uint8, W*H*3)
    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
    self.control_q = DoubleBuffer((0,0,0),name="control")
    self.sensors_q = DoubleBuffer((0,0,0,0,0,0,0),name="sensors")




    print("----------------------------------------------------------")
    print("---- Spawning BeamNG world, this might take awhile ----")
    print("----------------------------------------------------------")

    #TODO use config

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:64256
    self.bng = BeamNGpy(host='169.254.212.158', port=64256,home="/mnt/c/Program Files/BeamNG.tech.v0.33.3.0",quit_on_close=True)
    # self.bng = BeamNGpy(host='169.254.212.158', port=64256,home="/mnt/c/Program Files/BeamNG.tech.v0.32.5.0",quit_on_close=True)
    # Launch BeamNG.tech
    self.bng.open(launch=False)
    # self.bng.settings.set_deterministic(60)
    self.bng.settings.set_nondeterministic()


    # Create a scenario in west_coast_usa called 'example'
    # self.scenario = Scenario('west_coast_usa', 'example')
    # self.scenario = Scenario('east_coast_usa', 'example')
    # Create an ETK800 with the licence plate 'PYTHON'
    active_vehicles = self.bng.vehicles.get_current()
    print(active_vehicles)
    self.vehicle = active_vehicles['thePlayer']
    self.vehicle.connect(self.bng)
    # self.vehicle = Vehicle('ego_vehicle', model='etk800', license='PYTHON')
    # Add it to our scenario at this position and rotation
    # self.scenario.add_vehicle(self.vehicle, pos=(-950.195007,-627.664978,106.809998), rot_quat=(0, 0, 1, -0.9238795))
    # self.scenario.add_vehicle(self.vehicle, pos=(-710, 101, 30), rot_quat=(0, 0, 0.3826834, 0.9238795))
    # self.scenario.add_vehicle(self.vehicle, pos=(-718, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
    # Place files defining our scenario for the simulator to read
    # self.scenario.make(self.bng)


    # Load and start our scenario
    # self.bng.scenario.load(self.scenario)
    # self.bng.scenario.start()

    # self.bng_control = BeamNGpy(host='172.30.112.1', port=64256,home="/mnt/c/Program Files/BeamNG.tech.v0.32.2.0",quit_on_close=True,)
    # # Launch BeamNG.tech
    # self.bng_control.open(launch=False)
    # self.vehicle_control = self.bng_control.get_current_vehicles().get("ego_vehicle")





    # self.vehicle.ai.set_mode('disabled')
    # vehicle.ai_set_aggression(0)
    # vehicle.ai.set_target(target_vehicle.vid,mode="follow")
    # vehicle.ai_drive_in_lane(True)



    self.resolution=(W,H)
    # resolution=(1920,1080)

    # vehicle.recover()
    self.dash_cam = Camera(name="dash_cam",
                      vehicle=self.vehicle,
                      bng=self.bng,
                      pos=(0,0,1.5),
                      # bng=self.bng,pos=(0,-1.5,1.5),
                      # field_of_view_y=0.8,
                      update_priority=1,
                      is_streaming=True,
                      is_render_colours=True,
                      is_using_shared_memory=False,
                      is_render_depth=False,
                      is_render_instance=False,
                      resolution=self.resolution)
    GPS(name="gps",bng=self.bng,vehicle=self.vehicle)
    self.vehicle.attach_sensor(sensor=Electrics(),name="ele")

    self.camera_process = multiprocessing.Process(name="camera process", target=functools.partial(process_camera,self.dash_cam,self.camera_array,self.image_lock))
    self.camera_process.start()

    self.control_process = multiprocessing.Process(name="control process", target=functools.partial(process_controls,self.vehicle,self.control_q))
    self.control_process.start()

    self.sensors_process = multiprocessing.Process(name="sensors process", target=functools.partial(process_sensors,self.vehicle,self.sensors_q))
    self.sensors_process.start()

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    # print("apply control: ",self.is_engaged,(-steer_angle,throttle_out,brake_out))
    if self.is_engaged:
      # self.vehicle_control.set_velocity(50)
      # print("send: ",(-steer_angle/900,throttle_out,brake_out))
      # self.vehicle_control.control(steering=-steer_angle/900,throttle=throttle_out,brake=brake_out)
      self.control_q.put((-steer_angle/700,throttle_out,brake_out))



  def read_state(self):
    pass


  def read_sensors(self, state: SimulatorState):
    try:
      # if not self.sensors_q.empty():
      (velocity_x,velocity_y,velocity_z,position_x,position_y,steering,bearing) = self.sensors_q.get()
      # print((velocity_x,velocity_y,velocity_z,position_x,position_y,steering,bearing))

      state.velocity = vec3(x=float(velocity_x), y=float(velocity_y), z=float(velocity_z))
      # state.bearing = float(10)
      state.steering_angle = -steering*700
      state.gps.from_xy((position_x,position_y))
      state.valid = True
      # print(state.speed)
      self.is_engaged = state.is_engaged
    except Exception as e:
      print("exception read_sensors : ",e)



    # self.is_engaged=state.is_engaged
    # self.is_engaged=state.is_engaged

  # def read_cameras_t(self):
  #     # print(self.dash_cam.poll())
  #   # road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  #   while True:
  #     try:
  #       self.vehicle.sensors.poll()
  #       self.vehicle.sensors.poll()
  #       imgArr = np.asarray(self.dash_cam.poll()['colour'].convert('RGB'))
  #       self.camera_send.send((imgArr))
  #       if self.control_recv.poll(0):
  #         (steering,throttle,brake)=self.control_send.recv()
  #         self.vehicle.control(steering=steering,throttle=throttle,brake=brake)
  #       # self.road_image [...]= imgArr[...,:3]
  #       # road_image [...]= rgba2rgb(imgArr)
  #     except Exception as e:
  #       print("exception:",e)
  #     # self.image_lock.release()

  def read_cameras(self):
    pass

  def tick(self):
    pass

  def reset(self):
    pass

  def close(self, reason: str):
    self.exit_event.set()
    self.beamng_process.join()

import ctypes
import functools
import multiprocessing
import numpy as np
import time
import math

from multiprocessing import Pipe, Array

from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType

from openpilot.tools.sim.lib.common import SimulatorState, World,vec3
from openpilot.tools.sim.lib.camerad import W, H



class BeamNGTechWorld(World):
  def __init__(self, status_q, config, test_duration, test_run, dual_camera=False):
    super().__init__(dual_camera)
    self.status_q = status_q
    self.config=config
    (self.camera,_,_) = self.config['sensors']['dash_cam']
    self.vehicle = config['vehicle']
    self.test_run=test_run
    self.dual_camera=dual_camera
    self.vehicle_last_pos=(0,0,0)
    self.distance_moved=0
    self.last_check_timestamp=0




    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "starting"))

    print("----------------------------------------------------------")
    print("---- Spawning Beamng world, this might take awhile ----")
    print("----------------------------------------------------------")

    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))

    self.steer_ratio = 15
    self.vc = [0.0,0.0]
    self.reset_time = 0
    self.should_reset = False

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    # print("apply controls called: ", steer_angle, throttle_out, brake_out)
    vehicle = self.config['vehicle']
    if (time.monotonic() - self.reset_time) > 2:
      vehicle.control(steering=steer_angle/6, throttle=throttle_out or None, brake=brake_out or None)

    # else:
    #   self.vc[0] = 0
    #   self.vc[1] = 0

  def read_state(self):
    # print("read state called")
    # self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, None))
    # self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, md_state.done_info))
    # self.exit_event.set()
    pass

  def read_sensors(self, state: SimulatorState):
    self.vehicle.sensors.poll()

    vel = self.vehicle.state['vel']
    state.velocity = vec3(vel[0],vel[1],vel[2])
    # state.velocity.x , state.velocity.y ,state.velocity.z  = self.vehicle.state['vel']
    # state.velocity = (math.sqrt(v[0]**2 + v[1]**2 + v[2]**2))*3.6

    eleS = self.vehicle.sensors['ele']
    # print(self.vehicle.sensors.items())
    # print(dir(self.vehicle.state))
    # gpsS = self.vehicle.state['gps']
    rpm = eleS['rpm']
    curr_pos = self.vehicle.sensors['state']['pos']

    # wheel_angle = vehicle.state

    state.steering_angle = eleS.get('steering')

    # curr_pos = md_vehicle.position
    #
    # state.velocity = vehicle.velocity
    # state.bearing = md_vehicle.bearing
    # state.steering_angle = md_vehicle.steering_angle
    state.gps.from_xy(curr_pos)
    state.valid = True


    is_engaged = state.is_engaged
    if is_engaged and self.first_engage is None:
      self.first_engage = time.monotonic()
      self.op_engaged.set()

    # check moving 5 seconds after engaged, doesn't move right away
    after_engaged_check = is_engaged and time.monotonic() - self.first_engage >= 5 and self.test_run

    x_dist = abs(curr_pos[0] - self.vehicle_last_pos[0])
    y_dist = abs(curr_pos[1] - self.vehicle_last_pos[1])
    dist_threshold = 1
    if x_dist >= dist_threshold or y_dist >= dist_threshold: # position not the same during staying still, > threshold is considered moving
      self.distance_moved += x_dist + y_dist

    time_check_threshold = 30
    current_time = time.monotonic()
    since_last_check = current_time - self.last_check_timestamp
    if since_last_check >= time_check_threshold:
      if after_engaged_check and self.distance_moved == 0:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, {"vehicle_not_moving" : True}))
        self.exit_event.set()

      self.last_check_timestamp = current_time
      self.distance_moved = 0
      self.vehicle_last_pos = curr_pos

  def read_cameras(self):
    # print("read cameras called")
    self.road_image  = np.asarray(self.camera.poll()['colour'])
    pass

  def tick(self):
    pass

  def reset(self):
    self.should_reset = True

  def close(self, reason: str):
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()
    # self.metadrive_process.join()

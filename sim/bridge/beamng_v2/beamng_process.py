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



def beamng_process(dual_camera: bool, config: dict, camera_array, wide_camera_array, image_lock,
                      controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection,
                      exit_event, op_engaged, test_duration, test_run):
  arrive_dest_done = config.pop("arrive_dest_done", True)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  # env = MetaDriveEnv(config)


   # setup sceneraio

  #TODO use config

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






  def get_current_lane_info(vehicle):
    _, lane_info, on_lane = vehicle.navigation._get_current_lane(vehicle)
    lane_idx = lane_info[2] if lane_info is not None else None
    return lane_idx, on_lane

  def reset():
    # bng.stop()
    # bng.start()
    # bng.restart()
    # env.reset()
    # env.vehicle.config["max_speed_km_h"] = 1000
    # lane_idx_prev, _ = get_current_lane_info(env.vehicle)

    simulation_state = beamng_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)

    return 0
    return 0

  lane_idx_prev = reset()
  start_time = None

  # def get_cam_as_rgb(cam):
  #   cam = env.engine.sensors[cam]
  #   cam.get_cam().reparentTo(env.vehicle.origin)
  #   cam.get_cam().setPos(C3_POSITION)
  #   cam.get_cam().setHpr(C3_HPR)
  #   img = cam.perceive(to_float=False)
  #   if type(img) != np.ndarray:
  #     img = img.get() # convert cupy array to numpy
  #   return img

  def rgba2rgb( rgba, background=(255,255,255) ):
    row, col, ch = rgba.shape

    if ch == 3:
      return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros( (row, col, 3), dtype='float32' )
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]

    a = np.asarray( a, dtype='float32' ) / 255.0

    R, G, B = background

    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray( rgb, dtype='uint8' )

  rk = Ratekeeper(100, None)

  steer_ratio = 8
  vc = [0,0,0]

  while not exit_event.is_set():
    vehicle.sensors.poll()
    velocity = vehicle.state['vel']
    position = vehicle.state['pos']

    el2 = vehicle.sensors['ele']
    rpm = el2['rpm']

    wheel_angle = vehicle.state

    # sim_time = vehicle.sensors['time']
    sim_time = round(time.time() * 1000)
    steering = el2.get('steering')
    vehicle_state = beamng_vehicle_state(
      velocity=vec3(x=float(velocity[0]), y=float(velocity[1]), z=velocity[2]),
      position=position,
      bearing=float(0), #TODO fix this
      # bearing=float(math.degrees(env.vehicle.heading_theta)),
      steering_angle=steering
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas,brake, should_reset = controls_recv.recv()
        vehicle.control(steering=steer_angle/6,throttle=gas,brake=brake)
        # print(steer_angle/6,gas,brake,should_reset)

      # steer_beamng = steer_angle * 1 / (MAX_STEERING * steer_ratio)
      # steer_beamng = np.clip(steer_beamng, -1, 1)
      steer_beamng = steer_angle /6

      vc = [steer_beamng, gas,brake]

      if should_reset:
        lane_idx_prev = reset()
        start_time = None

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()


    # vehicle.control(steering=vc[0],throttle=vc[1],brake=vc[2])
    # if rk.frame % 5 == 0:
    #   pass
    #   timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
    #   lane_idx_curr, on_lane = get_current_lane_info(env.vehicle)
    #   out_of_lane = lane_idx_curr != lane_idx_prev or not on_lane
    #   lane_idx_prev = lane_idx_curr
    #
    #   if terminated or ((out_of_lane or timeout) and test_run):
    #     if terminated:
    #       done_result = env.done_function("default_agent")
    #     elif out_of_lane:
    #       done_result = (True, {"out_of_lane" : True})
    #     elif timeout:
    #       done_result = (True, {"timeout" : True})
    #
    #     simulation_state = beamng_simulation_state(
    #       running=False,
    #       done=done_result[0],
    #       done_info=done_result[1],
    #     )
    #     simulation_state_send.send(simulation_state)

    # if dual_camera:
    #   #TODO implememnt this
    #   pass
      # wide_road_image[...] = get_cam_as_rgb("rgb_wide")
    # road_image[...] = dash_cam.stream()['colour']

    try:
      print(dash_cam.poll())
      imgArr = rgba2rgb(np.asarray(dash_cam.poll()['colour']))
      road_image [...]= imgArr
    except Exception as e:
      print("exception: ",e)
      pass
    image_lock.release()

    rk.keep_time()

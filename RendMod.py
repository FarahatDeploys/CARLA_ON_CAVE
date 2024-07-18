import carla
import os
import cv2
import numpy as np
import queue
import threading
import time
from multiprocessing import Process, current_process

class CarlaController:
    def __init__(self, vehicle_id_file, window_x=0, window_y=0, 
                 camera_x=1.5, camera_y=0, camera_z=2.4, 
                 camera_pitch=0, camera_roll=0, camera_yaw=0, frame_rate=30, fov=90):
        self.vehicle_id_file = vehicle_id_file
        self.window_x = window_x
        self.window_y = window_y
        self.camera_x = camera_x
        self.camera_y = camera_y
        self.camera_z = camera_z
        self.camera_pitch = camera_pitch
        self.camera_roll = camera_roll
        self.camera_yaw = camera_yaw
        self.frame_rate = frame_rate
        self.fov = fov
        self.image_queue = queue.Queue()
        self.client = None
        self.vehicle = None
        self.camera = None
        self.processing_thread = None

    def read_vehicle_id(self):
        if os.path.exists(self.vehicle_id_file):
            with open(self.vehicle_id_file, 'r') as f:
                vehicle_id = int(f.read().strip())
            return vehicle_id
        else:
            raise FileNotFoundError(f"Vehicle ID file '{self.vehicle_id_file}' does not exist.")

    def connect_to_vehicle(self, vehicle_id):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(20.0)
        world = self.client.get_world()
        try:
            vehicle = world.get_actor(vehicle_id)
            if vehicle is not None:
                print(f"Connected to vehicle with ID: {vehicle_id}")
                return vehicle
        except Exception as e:
            print(f"Failed to connect to vehicle with ID {vehicle_id}: {e}")
            raise

    def image_callback(self, image):
        self.image_queue.put((image.raw_data, image.width, image.height))

    def process_images(self):
        cv2.namedWindow('CARLA Image', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('CARLA Image', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow('CARLA Image', self.window_x, self.window_y)

        while True:
            if not self.image_queue.empty():
                raw_data, width, height = self.image_queue.get()
                array = np.frombuffer(raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (height, width, 4))
                cv2.imshow('CARLA Image', array)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.01)

    def setup_camera(self):
        world = self.client.get_world()
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('sensor_tick', str(1.0 / self.frame_rate))  # Set the desired frame rate
        camera_bp.set_attribute('fov', str(self.fov))  # Set the desired FOV
        camera_transform = carla.Transform(
            carla.Location(x=self.camera_x, y=self.camera_y, z=self.camera_z),
            carla.Rotation(pitch=self.camera_pitch, roll=self.camera_roll, yaw=self.camera_yaw)
        )
        self.camera = world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(lambda image: self.image_callback(image))

    def start(self):
        vehicle_id = self.read_vehicle_id()
        self.vehicle = self.connect_to_vehicle(vehicle_id)
        self.setup_camera()
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.start()

    def stop(self):
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
        cv2.destroyAllWindows()

def set_max_priority():
    pid = current_process().pid
    try:
        if os.name == 'nt':
            import psutil
            p = psutil.Process(pid)
            p.nice(psutil.HIGH_PRIORITY_CLASS)
        else:
            os.nice(-20)
        print(f"Process {pid} priority set to maximum.")
    except Exception as e:
        print(f"Failed to set process priority for PID {pid}: {e}")

def run_controller(vehicle_id_file, window_x, window_y, camera_x, camera_y, camera_z, 
                   camera_pitch, camera_roll, camera_yaw, frame_rate, fov):
    set_max_priority()
    controller = CarlaController(vehicle_id_file, window_x, window_y, camera_x, camera_y, camera_z, 
                                 camera_pitch, camera_roll, camera_yaw, frame_rate, fov)
    try:
        controller.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()

if __name__ == '__main__':
    processes = []
    
    # Example to run multiple instances with shared vehicle ID file and different camera positions
    vehicle_file = 'vehicle_id.txt'
    window_positions = [(0, 0), (800, 0), (1600, 0), (2400, 0), (3200, 0), (4000, 0)]
    camera_positions = [(1.5, 0, 2.4), (1.5, 1, 2.4), (1.5, 0, 2.4), (1.5, 0, 2.4), (1.5, 0, 2.4), (1.5, 0, 2.4)]
    camera_rotations = [(90, 0, 0), (-90, 0, 0), (0, 0, 0), (90, 0, 0), (-90, 0, 0), (0, 0, 0)]
    frame_rate = 30  # Set the desired frame rate here
    fov = 90  # Set the desired field of view here

    for window_pos, camera_pos, camera_rot in zip(window_positions, camera_positions, camera_rotations):
        p = Process(target=run_controller, args=(
            vehicle_file, window_pos[0], window_pos[1], 
            camera_pos[0], camera_pos[1], camera_pos[2], 
            camera_rot[0], camera_rot[1], camera_rot[2], frame_rate, fov
        ))
        p.start()
        processes.append(p)
    
    for p in processes:
        p.join()

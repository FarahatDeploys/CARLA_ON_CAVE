import os
import cv2
import carla
import numpy as np
import pygame
from multiprocessing import Process, Queue, Manager, Barrier
from threading import Thread, Event
import time
import pygetwindow as gw
import platform


def initialize_carla_client():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle')[0]

    spawn_points = world.get_map().get_spawn_points()
    for spawn_point in spawn_points:
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            vehicle.set_autopilot(False)
            return vehicle, world
        except RuntimeError as e:
            print(f"Spawn failed at {spawn_point.location}: {e}")
            continue
    raise RuntimeError("All spawn points are occupied.")

def create_camera(world, vehicle, transform, index, queue, barrier):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_bp.set_attribute('sensor_tick', '0.1')  # Approximately 30 FPS
    camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        queue.put((index, array.copy(), time.time()))  # Send image data to queue with timestamp
        barrier.wait()  # Synchronize with other cameras

    camera.listen(camera_callback)
    return camera

def bgra_to_rgb(bgra_image):
    return cv2.cvtColor(bgra_image, cv2.COLOR_BGRA2BGR)

def display_worker(camera_queue, shared_data, barrier, stop_event, window_name):
    cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while not stop_event.is_set():
            idx, img, timestamp = camera_queue.get()
            combined_image_rgb = bgra_to_rgb(img)

            # Get the throttle and steering values
            throttle = shared_data.get("throttle", 0)
            steer = shared_data.get("steer", 0)

            # Overlay the throttle and steering values on the image

            # Display the processed image
            cv2.imshow(window_name, combined_image_rgb)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

            barrier.wait()  # Synchronize display with other cameras

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while not stop_event.is_set():
            idx, img, timestamp = camera_queue.get()
            combined_image_rgb = bgra_to_rgb(img)

            # Get the throttle and steering values
            throttle = shared_data.get("throttle", 0)
            steer = shared_data.get("steer", 0)

            # Overlay the throttle and steering values on the image

            # Display the processed image
            cv2.imshow(window_name, combined_image_rgb)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

            barrier.wait()  # Synchronize display with other cameras

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

def clear_carla_cache():
    if platform.system() == "Windows":
        cache_dir = os.path.join(os.getenv('LOCALAPPDATA'), 'CarlaSimulator', 'Cache')
    elif platform.system() == "Linux":
        cache_dir = os.path.join(os.path.expanduser('~'), '.cache', 'CarlaSimulator')
    else:
        print("Unsupported OS")
        return

    if os.path.exists(cache_dir):
        shutil.rmtree(cache_dir)
        print(f"Cache directory {cache_dir} has been removed.")
    else:
        print(f"Cache directory {cache_dir} does not exist.")

def handle_joystick_input(joystick, vehicle, shared_data, stop_event):
    while not stop_event.is_set():
        pygame.event.pump()
        axis_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        raw_steer = joystick.get_axis(0)
        raw_throttle = joystick.get_axis(5)
        raw_brake = joystick.get_axis(4)

        throttle = max(0.0, (raw_throttle + 1) / 2)
        brake = max(0.0, (raw_brake + 1) / 2)
        steer = raw_steer/4

        if joystick.get_button(0):
            print("Exit button pressed. Exiting...")
            stop_event.set()
            break

        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = steer
        control.brake = brake
        control.hand_brake = bool(joystick.get_button(1))

        vehicle.apply_control(control)

        # Update throttle and steering values in the shared data dictionary
        shared_data["throttle"] = throttle
        shared_data["steer"] = steer

def main():
    clear_carla_cache()
    os.environ['SDL_VIDEO_WINDOW_POS'] = "0,0"

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("No joystick found.")
        pygame.quit()
        exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    vehicle, world = initialize_carla_client()

    driver_head_location = carla.Location(x=0.3, y=0.0, z=1.2)
    camera_transforms = [
        carla.Transform(carla.Location(x=0.3, y=0.0, z=1.2), carla.Rotation(yaw=90)),   # Forward
        carla.Transform(carla.Location(x=0.3, y=0.0, z=1.2), carla.Rotation(pitch=-90)),  # Right
        carla.Transform(carla.Location(x=0.3, y=0.0, z=1.2), carla.Rotation(pitch=0)), # Backward
        carla.Transform(carla.Location(x=0.31, y=0.0, z=1.2), carla.Rotation(yaw=90)), # Left
        carla.Transform(carla.Location(x=0.31, y=0.0, z=1.2), carla.Rotation(pitch=-90)), # Up
        carla.Transform(carla.Location(x=0.31, y=0.0, z=1.2), carla.Rotation(pitch=0)) # Down
    ]

    camera_queues = [Queue() for _ in range(len(camera_transforms))]

    manager = Manager()
    shared_data = manager.dict()

    barrier = Barrier(len(camera_transforms) + 1)  # Barrier for synchronization

    cameras = [create_camera(world, vehicle, transform, idx, queue, barrier) 
               for idx, (transform, queue) in enumerate(zip(camera_transforms, camera_queues))]

    stop_event = Event()

    display_threads = []
    for idx in range(len(camera_queues)):
        thread = Thread(target=display_worker, args=(camera_queues[idx], shared_data, barrier, stop_event, f'Camera {idx+1}'))
        thread.start()
        display_threads.append(thread)
    

    time.sleep(0.5)
    
    windows = gw.getAllWindows()
    for window in windows:
        print(f"Title: {window.title}")
    
    try:
        win = gw.getWindowsWithTitle('Camera 6')[0]
        win.moveTo(4000, 0)
        win = gw.getWindowsWithTitle('Camera 5')[0]
        win.moveTo(3200, 0)
        win = gw.getWindowsWithTitle('Camera 1')[0]
        win.moveTo(2400, 0)
    except IndexError:
        print("Window named 'Camera 6' not found.")

    
        
    try:
        handle_joystick_input(joystick, vehicle, shared_data, stop_event)
        
    except KeyboardInterrupt:
        
        print("Exiting...")
    finally:
        for camera in cameras:
            camera.stop()
        vehicle.destroy()
        stop_event.set()
        for thread in display_threads:
            thread.join()
        pygame.quit()

if __name__ == "__main__":
    main()

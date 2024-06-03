import pygame
import time
import carla
import numpy as np
import threading
from queue import Queue

def initialize_carla_client():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)  # Increased timeout to 20 seconds
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle')[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(False)  # Disable autopilot
    return vehicle, world

def create_camera(world, vehicle):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '110')
    spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
    camera = world.spawn_actor(camera_bp, spawn_point, attach_to=vehicle)
    return camera

def process_camera_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]  # Remove alpha channel
    array = array[:, :, ::-1]  # Convert from BGRA to RGB
    return array

def log_values():
    while True:
        time.sleep(180)  # Log every 3 minutes
        print(f"Steering: {log_values.steer}, Throttle: {log_values.throttle}")

def main():
    # Initialize Pygame
    pygame.init()
    display = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("CARLA Simulator")

    # Initialize the joystick
    pygame.joystick.init()

    # Check for joystick
    if pygame.joystick.get_count() < 1:
        print("No joystick found.")
        pygame.quit()
        exit()

    # Get the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick Name: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")

    # Initialize CARLA client and get the vehicle and world
    vehicle, world = initialize_carla_client()

    # Create a camera and attach it to the vehicle
    camera = create_camera(world, vehicle)

    # Use a queue to store the latest camera image
    camera_queue = Queue()

    # Set up the camera listener to store the latest image
    camera.listen(lambda image: camera_queue.put(process_camera_image(image)))

    # Initialize log values
    log_values.steer = 0.0
    log_values.throttle = 0.0

    # Start a thread to log the values every 3 minutes
    log_thread = threading.Thread(target=log_values)
    log_thread.daemon = True
    log_thread.start()

    try:
        while True:
            # Process Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            pygame.event.pump()

            # Capture all axis values
            axis_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            print(f"Axis Values: {axis_values}")

            # Capture raw axis values
            raw_steer = joystick.get_axis(0)  # Example for steering
            raw_throttle = joystick.get_axis(1)  # Attempting Y-axis for throttle

            # Print raw axis values for debugging
            print(f"Raw Steering: {raw_steer}, Raw Throttle: {raw_throttle}")

            # Adjust values as needed for CARLA (e.g., reversing the axis direction)
            throttle = max(0.0, (raw_throttle + 1) / 2)  # Convert from [-1, 1] to [0, 1]
            steer = raw_steer  # Use raw steering value directly

            # Update log values
            log_values.steer = steer
            log_values.throttle = throttle

            # Capture button values (example: brake and handbrake)
            brake = joystick.get_button(0)  # Example button for brake
            handbrake = joystick.get_button(1)  # Example button for handbrake

            # Exit game if button 0 is pressed
            if joystick.get_button(0):
                print("Exit button pressed. Exiting...")
                raise KeyboardInterrupt

            # Create CARLA control command
            control = carla.VehicleControl()
            control.throttle = throttle
            control.steer = steer
            control.brake = float(brake)
            control.hand_brake = bool(handbrake)

            # Apply control to the vehicle
            vehicle.apply_control(control)

            # Sleep for a short period to reduce CPU usage
            time.sleep(0.1)

            # Render camera image if available
            if not camera_queue.empty():
                camera_image = camera_queue.get()
                camera_surface = pygame.surfarray.make_surface(camera_image.swapaxes(0, 1))
                display.blit(camera_surface, (0, 0))
                pygame.display.flip()

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        camera.stop()
        pygame.quit()
        vehicle.destroy()

if __name__ == "__main__":
    main()

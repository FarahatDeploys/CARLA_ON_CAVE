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
            # Write the vehicle ID to a file
            with open("vehicle_id.txt", "w") as file:
                file.write(str(vehicle.id))
            return vehicle, world
        except RuntimeError as e:
            print(f"Spawn failed at {spawn_point.location}: {e}")
            continue
    raise RuntimeError("All spawn points are occupied.")

def handle_joystick_input(joystick, vehicle, stop_event):
    last_print_time = time.time()
    while not stop_event.is_set():
        pygame.event.pump()
        axis_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        raw_steer = joystick.get_axis(0)
        raw_throttle = joystick.get_axis(5)
        raw_brake = joystick.get_axis(4)

        throttle = max(0.0, (raw_throttle + 1) / 2)
        brake = max(0.0, (raw_brake + 1) / 2)
        steer = raw_steer / 4

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

        # Print the values of throttle, steer, and brake every 0.5 seconds
        current_time = time.time()
        if current_time - last_print_time >= 0.5:
            print(f'Throttle: {throttle:.2f}, Steer: {steer:.2f}, Brake: {brake:.2f}')
            last_print_time = current_time

def main():
    # initialize pygame to capture the input from the user 
    pygame.init()
    pygame.joystick.init()
    stop_event = Event()
    if pygame.joystick.get_count() < 1:
        print("No joystick found.")
        pygame.quit()
        exit()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    vehicle, world = initialize_carla_client()
    handle_joystick_input(joystick, vehicle, stop_event)

if __name__ == "__main__":
    main()

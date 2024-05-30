# Imports
import sys
import math
import threading
import time
from pymavlink import mavutil

# Globals
PORT = 5770
srcSystem = 1
NaN = math.nan


# Functions
def ack_item(connection_obj, message):
    status = str(connection_obj.recv_match(type=message, blocking=True))
    print(f"{status} received successfully...")


def arm(connection_obj):
    print("arming...")
    connection_obj.arducopter_arm()
    ack_item(connection_obj, "COMMAND_ACK")


def takeoff(connection_obj):
    print("Takeoff Start...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                         0, 0, 0, 0, 0, 0, 0, 10)
    ack_item(connection_obj, "COMMAND_ACK")


def land(connection_obj):
    print("Landing...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND,
                                         0, 0, 0, 0, NaN, 32.0756, 34.7756, 10)
    ack_item(connection_obj, "COMMAND_ACK")


def set_rtl(connection_obj):
    print("Set Return to Launch...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                         0, 0, 0, 0, 0, 0, 0, 0)
    ack_item(connection_obj, "COMMAND_ACK")


def follow(connection_obj):
    print("Set Return to Launch...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_FOLLOW,
                                         connection_obj.target_system, 0, 0, 0, 0, 0, 0, 0)
    ack_item(connection_obj, "COMMAND_ACK")


def read_config_file(config_file):
    with open(config_file, 'r') as file:
        lines = file.readlines()
        i = lines.index(f"PORT {PORT}\n")  # Read the role from the config file - depends on its structure
    return lines[i - 3]


# Function to send current position to other drone
def send_position_to_follower(connection_obj):
    while True:
        # Read current position from the flight controller
        msg = connection_obj.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg.get_srcSystem() == srcSystem:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.alt / 1000  # Convert to meters

            # Construct and send position message to follower drone
            msg = connection_obj.mav.global_position_int_encode(
                int(time.time() * 1e6),  # Timestamp
                int(current_lat * 1e7),  # Latitude in degrees * 1e7
                int(current_lon * 1e7),  # Longitude in degrees * 1e7
                int(current_alt * 1000),  # Altitude in meters * 1000
                0, 0, 0  # Relative altitude, vx, vy
            )

        connection_to_follower_obj = mavutil.mavlink_connection(f'tcp:127.0.0.1:{(PORT - 10)}')

        connection_to_follower_obj.global_position_int_send(msg.time_boot_ms,
                                                            msg.lat,
                                                            msg.lon,
                                                            msg.alt,
                                                            msg.relative_alt,
                                                            msg.vx,
                                                            msg.vy,
                                                            msg.vz,
                                                            msg.hdg)
        connection_to_follower_obj.close()

        time.sleep(1)  # Send position every second


# Function to receive leader position and navigate towards it
def receive_position_and_navigate(connection_obj):
    while True:
        # Receive leader position from the leader
        connection_to_leader_obj = mavutil.mavlink_connection(f'tcp:127.0.0.1:{(PORT + 13)}') # +3 for open Serial2
        msg = connection_to_leader_obj.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        leader_lat = msg.lat / 1e7
        leader_long = msg.lon / 1e7
        leader_alt = msg.alt / 1000  # Convert from mm to meters

        # Navigate the follower drone towards the leader position
        connection_obj.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
                                                                          connection_obj.target_system,
                                                                          connection_obj.target_component,
                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                          int(0b110111111000),  # mask for position only
                                                                          leader_long,
                                                                          leader_lat,
                                                                          leader_alt,
                                                                          0, 0,
                                                                          0, 0, 0, 0, 0,
                                                                          0))
        connection_to_leader_obj.close()

        time.sleep(1)  # Repeat every second


def main():
    print("Program Started!")

    connection_obj = mavutil.mavlink_connection(f'tcp:127.0.0.1:{PORT + 3}') # +3 for open Serial2

    while connection_obj.target_system == 0:
        print("Checking Heartbeat...")
        connection_obj.wait_heartbeat()
        print(f"Heartbeat for system (system {connection_obj.target_system} {connection_obj.target_component})")

    role = read_config_file("config.txt")
    print(role)

    connection_obj.mav.command_long_send(connection_obj.target_system, connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
    ack_item(connection_obj, "COMMAND_ACK")

    arm(connection_obj)

    takeoff(connection_obj)

    # TODO: implement if else roles
    if role == 'leader\n':
        # fly to given wp
        connection_obj.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
                                                                                              connection_obj.target_system,
                                                                                              connection_obj.target_component,
                                                                                              mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                              int(0b110111111000),
                                                                                              10, 0, -10,
                                                                                              0, 0,
                                                                                              0, 0, 0, 0, 0,
                                                                                              0))

        # Start a thread to send position to the follower drone
        send_position_thread = threading.Thread(target=send_position_to_follower, args=(connection_obj,))
        send_position_thread.daemon = True
        send_position_thread.start()
        send_position_thread.join()  # wait for the thread to exit before exiting

    elif role == 'follower\n':
        # Start a thread to receive leader position and navigate towards it
        follow(connection_obj)

        receive_position_thread = threading.Thread(target=receive_position_and_navigate, args=(connection_obj,))
        receive_position_thread.daemon = True
        receive_position_thread.start()
        receive_position_thread.join()  # wait for the thread to exit before exiting

    else:
        print("Unknown, exiting...")
        exit()


if __name__ == '__main__':
    main()

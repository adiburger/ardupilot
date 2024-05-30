# Imports
import math
import time
from pymavlink import mavutil

NaN = math.nan

# Classes
class MissionItem:
    def __init__(self, seq: int, cwp, long, lat, alt):  #TODO - see what happens if I switch the seq and cwd index
        self.seq = seq,
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = cwp
        self.autocontinue = 1
        self.hold = 0.0
        self.accepted_radius = 1.0
        self.pass_radius = 0.0
        self.yaw = NaN
        self.long = long
        self.lat = lat
        self.alt = alt
        self.mission_type = 0

# Functions
def ack_item(connection_obj, message):
    status = str(connection_obj.recv_match(type=message, blocking=True))
    print(f"{status} received successfully...")


def upload_mission(connection_obj, mission_items):
    items_count = len(mission_items)

    # send mission items count
    connection_obj.mav.mission_count_send(connection_obj.target_system,
                                          connection_obj.target_component,
                                          items_count, 0)

    ack_item(connection_obj, "MISSION_REQUEST")

    for wp in mission_items:
        print(f"creating waypoint {wp.seq}...")

        connection_obj.mav.mission_item_send(connection_obj.target_system,
                                             connection_obj.target_component,
                                             wp.seq[0],
                                             wp.frame,
                                             wp.command,
                                             wp.autocontinue,
                                             wp.hold,
                                             wp.accepted_radius,
                                             wp.pass_radius,
                                             wp.yaw,
                                             wp.long,
                                             wp.lat,
                                             wp.alt,
                                             wp.mission_type)

        if wp != mission_items[items_count - 1]:
            ack_item(connection_obj, "MISSION REQUEST")

        ack_item(connection_obj, "MISSION_ACK")


def arm(connection_obj):
    print("arming...")
    connection_obj.arducopter_arm()
    ack_item(connection_obj, "COMMAND_ACK")


def takeoff(connection_obj):
    print("Takeoff Start...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                         0, 0, 0, 0, NaN, 32.0712, 34.78, 10)
    ack_item(connection_obj, "COMMAND_ACK")


def land(connection_obj):
    print("Landing...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND,
                                         0, 0, 0, 0, NaN, 32.0756, 34.7756, 10)
    ack_item(connection_obj, "COMMAND_ACK")


def start_mission(connection_obj):
    print("Start Mission...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_MISSION_START,
                                         0, 0, 0, 0, 0, 0, 0, 0)
    ack_item(connection_obj, "COMMAND_ACK")


def set_rtl(connection_obj):
    print("Set Return to Launch...")
    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                         0, 0, 0, 0, 0, 0, 0, 0)
    ack_item(connection_obj, "COMMAND_ACK")


def auto_logic():
    mission_wp = []

    print("Program Started!")

    connection_obj = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

    while connection_obj.target_system == 0:
        print("Checking Heartbeat...")
        connection_obj.wait_heartbeat()
        print(f"Heartbeat for system (system {connection_obj.target_system} {connection_obj.target_component})")

    mission_wp.append(MissionItem(int(0), 0, 32.0712, 34.78, 100.0))
    mission_wp.append(MissionItem(1, 0, 32.0756, 34.7756, 100.0))
    mission_wp.append(MissionItem(2, 0, 32.0714, 34.7827, 10.0))

    upload_mission(connection_obj, mission_wp)

    arm(connection_obj)
    takeoff(connection_obj)
    start_mission(connection_obj)

    for wp in mission_wp:
        print("Message Read" + str(connection_obj.recv_match(type="MISSION_ITEM_REACHED",
                                                             condition=f"MISSION_ITEM_REACHED.seq == {wp.seq}")))

    set_rtl(connection_obj)

    connection_obj.close()


def main():
    print("Program Started!")

    wp = MissionItem(1, 0, 32.08, 34.8, 10.0)

    connection_obj = mavutil.mavlink_connection('tcp:127.0.0.1:5772')

    while connection_obj.target_system == 0:
        print("Checking Heartbeat...")
        connection_obj.wait_heartbeat()
        print(f"Heartbeat for system (system {connection_obj.target_system} {connection_obj.target_component})")

    connection_obj.mav.command_long_send(connection_obj.target_system,
                                         connection_obj.target_component,
                                         1,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)

    # connection_obj.set_mode_rtl()

    arm(connection_obj)

    takeoff(connection_obj)

    time.sleep(10)

    # fly to given wp
    connection_obj.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
                                                                                          connection_obj.target_system,
                                                                                          connection_obj.target_component,
                                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                          int(0b110111111000),
                                                                                          wp.long, wp.lat, wp.alt, 0, 0,
                                                                                          0, 0, 0, 0, 0,
                                                                                          0))

    # land(connection_obj)
    #
    # connection_obj.close()


if __name__ == '__main__':
    main()

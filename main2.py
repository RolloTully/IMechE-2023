import pymavlink
from pymavlink import mavutil
import time
from multiprocessing import Process, active_children
from time import sleep
import signal
import sys
connection = mavutil.mavlink_connection("/dev/ttyACM0")
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
connection.wait_heartbeat()
connection.mav.ping_send(int(time.time() * 1e6),0,0,0)
#connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
#connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 115200, 1)

time.sleep(0.5)
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,"QGC will read this".encode())
connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
    #print(connection.messages['GPS_RAW_INT'])
    #print(connection.messages['WIND'])
    #print(connection.messages['ATTITUDE'])
    #print(connection.messages['RANGEFINDER'])
#print(connection.messages[''])
#msg = connection.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True)
#print(msg)
connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,21196,0,0,0,0,0)
sleep(20)

connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,21196,0,0,0,0,0)
connection.mav.command_long_send(connection.target_system, connection.target_component, 16,0,0,21196,0,0,0,0,0)

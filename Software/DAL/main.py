import pymavlink
from pymavlink import mavutil, mavwp
import time
from multiprocessing import Process, active_children
import numpy as np
from time import sleep
import signal
import sys
import json

global FailSafe_Ready
FailSafe_Ready = False

class Link(object):
    def __init__(self, address):
        self.Waypoint_Loader = mavwp.MAVWPLoader()
        self.connection = mavutil.mavlink_connection(address)
        print("Connection Established.")
        self.connection.wait_heartbeat()
        print("Heartbeat recived.")
        self.connection.mav.ping_send(int(time.time() * 1e6),0,0,0)
        self.heart_task = Process(target = self.send_heartbeat)
        self.heart_task.start()
        print("Link Ready.")
    def Set_Messages(self):
        ''' Sets up the nessecary message intervals'''
        self.message_list = [33, 83]# Filtered predicted positon, Commanded Attitude
        for cmd in self.message_list:
            self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,cmd,2000,1,0,0,0,0) # Requests the required messages
            self.connection.recv_match(type='COMMAND_ACK', blocking = True) # Waits for confirmation.
    def Upload_Waypoint(self, Waypoints):
        mav.waypoint_clear_all_send()
        for self.waypoint in Waypoints:
            self.self.latitude, self.longditude, self.radius, self.pass_radius, self.altitude, self.flag = self.waypoint()
            if self.flag = "WP"
                self.point = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, self.radius, self.pass_radius, 0, self.latitude, self.longditude, self.altitude)
            elif self.flag = "HALO":
                self.point = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_LAND, current, 1, 0, 0, 0, self.yaw, self.latitude, self.longditude, self.altitude)
            elif self.flag = "PCR"
                self.point = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_LAND, current, 1, 0, 0, 0, self.yaw, self.latitude, self.longditude, self.altitude)

    def send_heartbeat(self):
        while True:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            self.connection.wait_heartbeat()
            sleep(0.2)
    def set_pwm(self, channel, position):
        self.rc_channel_values = [65535 for _ in range(18)]
        self.rc_channel_values[channel- 1] = position
        self.commection.mav.rc_channels_override_send(self.connection.target_system,self.connection.target_component,*self.rc_channel_values)
    def disarm(self):
        '''Force disarms drone'''
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,0,21196,0,0,0,0,0)
        return self.connection.recv.match(type='COMMAND_ACK', blocking = True)
    def is_landed(self):
        return self.connection.messages['MAV_LANDED_STATE']
    def set_course(self, items):
        '''Uploads a new waypoint mission'''
        self.connection.mav.send()
        self.connection.recv.match(type='MISSION_REQUEST_INT', blocking = True)
        for item in items:
            self.connection.mav.send()
            self.connection.recv.match(type='COMMAND_ACK', blocking = True)
        pass
    def set_geofence(self):
        pass



class Path(object):
    '''Defines a path for the drone to follow'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, p_rad = 0, altitude = 60):
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.altitude = altitude     #Landing waypoint altitude
        self.radius = radius         #Way point acceptance radius
        self.pass_radius = r_rad        #Direction to pass the way point
        self.altitude = altitude     #Target waypoint altitude
    def __call__(self):
        return self.latitude, self.longditude, self.radius, self.pass_radius, self.altitude, self.flag
class Takeoff(object):
    "Defines the takeoff"
    def __init__(self, latitude = None, longditude = None, altitude = 50, minimum_pitch = 15):
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.altitude = altitude     #Landing waypoint altitude
        self.minimum_Pitch = minimum_pitch
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.altitude, self.flag])
class HALO(object):
    '''The Dynamic Auto Landing mission item instructs the flight director to perform the relevent actions'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, altitude = 60):
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.altitude = altitude     #Landing waypoint altitude
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])
class PCR(object):
    '''This mission item in formed the flight director to monitor the aircraft position and release the cargo at the optimal moment'''
    def __init__(self, latitude = None, longditude = None, altitude = 30):
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.altitude = altitude     #Target waypoint altitude
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class main():
    def __init__(self):
        '''Parameter List'''
        self.cargo_release_time = 0.25 #Time taken for the cargo to exit the aircraft
        self.cargo_relased = False
        self.FS_Maximum_Period = 1 #second
        self.Velocity_FS_Threshold = [0,30]# Meters per second, keeps the done in line with competiton rules
        self.Altitude_FS_Threshold = [0,120]#Above altitude, keeps the drone in line with regulations
        self.Battery_Voltage_FS_Threshold = [0,24]#Volts, prevents brown outs
        self.Battery_Current_FS_Threshold = [0,100]#Amps, prevents catastrophic short circuit
        '''Runs at system boot'''
        self.link = Link("/dev/ttyACM0") #Establishes communciation with the flight controller.
        signal.signal(signal.SIGINT, self.keyboard_interupt_handler) #Stop people accidently crashing the flight controller while in flight.
        '''Start the Failsafe watchdog'''
        self.FailSafe_Process = Process(target = self.Failsafe_Watchdog) #Defines the Failsafe watch dog daughter process.
        self.FailSafe_Process.start() #Starts the Failsafe process, this must be started before all other processes to ensure saftey.
        print("Watchdog Active.")
        '''Load Plane Parameters'''
        self.ret = self.Load_Parameters()
        if not self.ret:
            pass
        print("Params Loaded.")
        '''Loads mission from json'''
        self.ret = self.Load_mission()
        if not self.ret:
            pass
        print("Mission Loaded Successfully.")
        self.mainloop_process = Process(target = self.mainloop) #Define the
        self.mainloop_process.start()

    def Get_Home(self):

    def Do_Waypoints(self, path):


    def Heuristic_Automatic_Landing_Operation(self):# HALO
        '''A fully autinimious heuristic algorithum that optimised the landing heading and flight path angle'''
        pass

    def Precision_Cargo_Release(self):# PCR
        '''Uses onboard velocity time and position estimations to accuratly release the cargo'''
        self.cargo_relased = False
        self.cargo_release_time_threshold = 0.1 #seconds
        '''Upload Path'''
        '''Uploads 2 waypoints'''

        '''Monitor progress'''
        '''Checks to see when way point 1 has been passed'''
        '''Starts monitoring position and release time'''

        '''Release cargo, does t<=0'''
        self.link.set_pwm()#actuated the release servo


        while not self.cargo_relased:
            self.link.connection.messages
            self.position = np.array([])
            self.v_x = self.v*np.cos(self.heading+self.d_heading)
            self.v_Y = self.v*np.sin(self.heading+self.d_heading)
            self.rho = self.d_heading*self.v
            self.centre_or_rot = self.position + np.array([self.rho*np.cos(self.heading),self.rho*np.sin(self.heading)])
            self.prop_forward = []
            #Extrapolate the planes current flight path to predict the time at which the cargo should be released
            pass
    def Load_Parameters(self):
        '''Loads Program Parameters'''
        self.failure = False
        self.param_data = json.load(open("param.json","r"))
        self.param_items = self.param_data.keys()

        return self.failure


    def Load_mission(self):
        '''Loads in mission waypoints from a JSON file on an sd card'''
        self.Mission_objects = []
        self.mission_data = json.load(open("Mission.json","r"))
        self.mission_sections = self.mission_data.keys()
        for self.mission_section in self.mission_sections:
            self.section_type = self.mission_data[self.mission_section]["Type"]
            if self.section_type = "TKOF":
                self.Minimum_Pitch = self.mission_data[self.mission_section]["TKOF_Options"]["Minimum_Pitch"]
                self.Latitude = self.mission_data[self.mission_section]["TKOF_Options"]["Latitude"]
                self.Longditude = self.mission_data[self.mission_section]["TKOF_Options"]["Longditude"]
                self.Altitude = self.mission_data[self.mission_section]["TKOF_Options"]["Altitude"]
                self.Mission_objects.append(Takeoff(0,self.Latitude, self.Longditude, self.Altitude, self.Minimum_Pitch))
            elif self.section_type = "WP":
                self.Latitude = []
                self.Longditude = []
                self.Altitude = []
                self.Acceptance_Radius = []
                self.Pass_Distance = []
                for self.Mission_Item in self.mission_data[self.mission_section]["Waypoints"].keys():
                    print(sef.Mission_item)
                    self.Latitude.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Latitude"])
                    self.Longditude.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Longditude"])
                    self.Altitude.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Altitude"])
                    self.Acceptance_Radius.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Acceptance_Radius"])
                    self.Pass_Distance.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Pass_Distance"])
                self.Mission_objects.append(Path(self.Latitude, self.Longditude, self.Acceptance_Radius, self.Pass_Distance, self.Altitude))
            elif self.section_type = "PCR":
                self.Latitude = self.mission_data[self.mission_section]["PCR_Data"]["Latitude"]
                self.Longditude = self.mission_data[self.mission_section]["PCR_Data"]["Longditude"]
                self.Mission_objects.append(PCR(self.Latitude, self.Latitude))
            elif self.section_type = "HALO":
                self.Latitude = self.mission_data[self.mission_section]["HALO_Data"]["Latitude"]
                self.Longditude = self.mission_data[self.mission_section]["HALO_Data"]["Longditude"]
                self.Altitude = self.mission_data[self.mission_section]["HALO_Data"]["Altitude_Abort"]
                self.Mission_objects.append(HALO(self.Latitude, self.Latitude, self.Altitude))
            else:
                print("Mission item unknown")

    def Failsafe_Action(self):
        '''
        The actions of the FTS must aim to safely land the UA as soon as possible after initiation.
        For Fixed Wing aircraft, the throttle shall be set to ‘engine off’ and the control surfaces
        set to initiate a rapid spiral descent.
        For hybrid aircraft, the aircraft shall not transition between hover and forward flight
        following any activation of FTS or Failsafes.
        Other actions could include deployment of a recovery parachute.
        Full aileron left
        Full up elevator
        Full rudder left
        Initiates a death spiral
        '''
        # If in no failsafe region override.
        self.link.set_pwm(2,1900)# Elevator
        self.link.set_pwm(3,1900)# Aileron
        self.link.set_pwm(4,1900)# Rudder
        self.link.set_pwm(1,500) #Throttle

    def Failsafe_Watchdog(self):
        '''Monitors system parameters to ensure complicate with all failsafe conditions'''
        '''This method also enacts the failsafe protocol when nessecary to maintain the saftey of ground staff'''
        '''
        This method must monitor:
        The failsafe watchdog to ensure proper functioning, is the failsafe watchdog running sufficently fast?
        geofence compliance
        minimum altitude compliance
        maximum speed compliance
        battery voltage compliance
        minimum speed compliance
        '''
        self.FS_Maximum_Period = 1 #second
        self.Velocity_FS_Threshold = [0,30]# Meters per second, keeps the done in line with competiton rules
        self.Altitude_FS_Threshold = [0,120]#Above altitude, keeps the drone in line with regulations
        self.Battery_Voltage_FS_Threshold = [0,24]#Volts, prevents brown outs
        self.Battery_Current_FS_Threshold = [0,100]#Amps, catastrophic short circuit
        print("Watchdog Boot.")
        while True:
            sleep(1)
            print("Set state")
            FailSafe_Ready = True
            pass
    def keyboard_interupt_handler(self,q,p):
        '''Execute order 66, kills all the children'''
        '''Interupts the standard keyboard interupt to ensure all processes are correctly terminated.'''
        if not self.link.is_landed():
            print("Craft in flight, command override active.")
        else:
            self.link.disarm()
            self.active = active_children()
            print(len(self.active),"children found.")
            for self.child in self.active:
                self.child.kill()
            print("All child processes terminated.")
            print("Terminating flight director.")
            sys.exit()
    def mainloop(self):
        '''hold while plane is not armed'''
        while not FailSafe_Ready:
            sleep(1)
            print(FailSafe_Ready)
            pass
        print(self.link.connection.motors_armed_wait())
        print("Armed")
        '''Upload Mission'''



        '''Monitor waypoint mission progress.'''

        while True:
            '''What is the current'''
            sleep(0.1)

        ''' Move to cargo drop, Start PCR'''
        self.PCR()



        '''Monitor waypoint mission progress.'''
        while True:
            '''what is the current waypoint.'''
            sleep(0.1)
        '''Start HALO.'''


        sleep(5)
        self.link.disarm()
        #while True:
        #    pass


if __name__ == "__main__":
    main()

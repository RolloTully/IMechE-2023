import json
class Path(object):
    '''Defines a path for the drone to follow'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, p_rad = 0, altitude = 60):
        self.mission_index = index   #Mission Item index, defines the order in which the items will be executed
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.altitude = altitude     #Landing waypoint altitude
        self.radius = radius         #Way point acceptance radius
        self.pass_radius = r_rad        #Direction to pass the way point
        self.altitude = altitude     #Target waypoint altitude
        self.flag = "WP"             #Waypoint flag
    def __call__(self):
        return self.latitude, self.longditude, self.radius, self.pass_radius, self.altitude, self.flag

class Takeoff(object):
    "Defines the takeoff"
    def __init__(self, index, latitude = None, longditude = None, altitude = 50, minimum_pitch = 15, flag = None):
        self.mission_index = index
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.altitude = altitude     #Landing waypoint altitude
        self.minimum_Pitch = minimum_pitch
        self.flag = "TKOF"            #DAL flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.altitude, self.flag])

class HALO(object):
    '''The Dynamic Auto Landing mission item instructs the flight director to perform the relevent actions'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None):
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.altitude = altitude     #Landing waypoint altitude

    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class PCR(object):
    '''This mission item in formed the flight director to monitor the aircraft position and release the cargo at the optimal moment'''
    def __init__(self, index, latitude = None, longditude = None, altitude = 30, flag = None):
        self.mission_index = index
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.altitude = altitude     #Target waypoint altitude
        self.flag = "PCR"             #Waypoint flag, Precision Cargo Release flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class main():
    def __init__(self):
        self.Load_mission()
    def Load_mission(self):
        '''Loads in mission waypoints from a JSON file on an sd card'''
        self.failure = False
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

        '''
        for self.mission_item in self.mission_items:
            if self.mission_data[self.mission_item]["Flag"] == "Waypoint": #Mission item is a waypoint
                self.Mission_objects.append(Waypoint(self.mission_data[self.mission_item]["Mission_Item_Index"], self.mission_data[self.mission_item]["Latitude"],self.mission_data[self.mission_item]["Longditude"],self.mission_data[self.mission_item]["Acceptance_Radius"], self.mission_data[self.mission_item]["Flag"]))
            elif self.mission_data[self.mission_item]["Flag"] == "HALO": #Mission Landing command
                self.Mission_objects.append(HALO(self.mission_data[self.mission_item]["Mission_Item_Index"], self.mission_data[self.mission_item]["Latitude"],self.mission_data[self.mission_item]["Longditude"],self.mission_data[self.mission_item]["Acceptance_Radius"], self.mission_data[self.mission_item]["Flag"]))
            elif self.mission_data[self.mission_item]["Flag"] == "PCR": #Cargo release supervision
                self.Mission_objects.append(PCR(self.mission_data[self.mission_item]["Mission_Item_Index"], self.mission_data[self.mission_item]["Latitude"],self.mission_data[self.mission_item]["Longditude"],self.mission_data[self.mission_item]["Acceptance_Radius"], self.mission_data[self.mission_item]["Flag"]))
            else:
                print("Dont know how you've ended up here.")
                self.failure = True
                break
        self.Mission_objects.sort(key = lambda x:x.mission_index)
        self.mission = Mission()
        self.mission.path = np.array(self.Mission_objects)
        '''

        return self.failure
if __name__ == "__main__":
    main()
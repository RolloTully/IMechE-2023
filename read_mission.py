import json
class main():
    def __init__(self):
        self.Load_mission()
    def Load_mission(self):
        '''Loads in mission waypoints from a JSON file on an sd card'''
        self.failure = False
        self.Mission_objects = []
        self.mission_data = json.load(open("Mission.json","r"))
        self.mission_items = self.mission_data.keys()
        print(self.mission_items)
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

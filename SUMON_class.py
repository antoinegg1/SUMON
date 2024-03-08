import numpy as np
# basic class:edge(lane),junction,connection
# traffic_light class:tl_Logic,Phase,request
# vehicle class:vehicle,trip,flow
class flow:
    def __init__(self, id,depart,origin,destination,begin,end,vehsPerHour):
        self.id = id
        self.depart = depart
        self.origin = origin
        self.destination = destination
        self.origin_index=-1
        self.destination_index=-1
        self.begin=float(begin)
        self.end=float(end)
        self.vehsPerHour=float(vehsPerHour)
        self.route=None
        self.display()
    def vehicle_generate(self,current_time):
            return vehicle(self.id+"_"+str(current_time),self.origin_index,self.route,current_time)
    def display(self):
        print("Flow Name:", self.id, ", Depart:", self.depart, ", Origin:", self.origin, ", Destination:", self.destination)
class trip:# 从一个lane的起点到另一个lane的起点(endpoint)
    def __init__(self, id,depart,origin,destination):
        self.id = id
        self.depart = depart
        self.origin = origin
        self.destination = destination
        self.route=None
        self.display()
    def display(self):
        print("Trip Name:", self.id, ", Depart:", self.depart, ", Origin:", self.origin, ", Destination:", self.destination)
class vehicle:
    def __init__(self, id,position,route,depart_time,speed=10):
        self.id = id
        self.speed =float(speed)
        self.position=position 
        self.moving_precent=0 #移动进度
        self.route=route # vehicle路径
        self.depart_time=depart_time
        self.wait=False #是否等待
        self.total_waiting_time=0
        self.linkIndex=-1
        self.traffic_light_index=-1
        self.current_connection=None
        self.direction=1 #沿着当前lane的方向，1为正，-1为负 (暂时没用)
        #self.display()
    def display(self):
        print("Vehicle Name:", self.id, ", Position:", self.position,
              "will launch at:",self.depart_time,", Speed:", self.speed,
              ", Direction:", self.direction)
    def move(self,edges,junctions):
        
        if self.wait==False:
            self.moving_precent+=self.speed/edges[self.position].length
            #print("Vehicle Name:", self.id, ", Position:", self.position,"moving_precent:",self.moving_precent)
            if self.moving_precent>=1:
                tmp_position_index=self.route.index(edges[self.position].id)
                next_position_name=self.route[tmp_position_index+1]
                for connection in edges[self.position].from_connections:
                    #print(next_position_name ,connection.to_edge)
                    if connection.to_edge==next_position_name:
                        #print("find connection")
                        matching_indices = [i for i, x in enumerate(junctions) if x.id == connection.via]
                        self.traffic_light_index=matching_indices[0]
                        self.linkIndex=int(connection.linkIndex)
                        self.current_connection=connection
                        if junctions[self.traffic_light_index].current_phase.state[self.linkIndex]=="r":
                            self.wait=True
                            connection.waiting_vehicles.append(self)
                            print("Vehicle Name:", self.id, ", Position:", self.position,"start waiting")
                        else :
                            matching_indices = [i for i, x in enumerate(edges) if x.id == next_position_name]
                            self.position=matching_indices[0]
                            self.moving_precent=0
                        break
        else:
            if junctions[self.traffic_light_index].current_phase.state[self.linkIndex]==("G" or "g" or "y"):
                tmp_position_index=self.route.index(edges[self.position].id)
                next_position_name=self.route[tmp_position_index+1]
                self.wait=False
                self.current_connection.waiting_vehicles.remove(self)
                matching_indices = [i for i, x in enumerate(edges) if x.id == next_position_name]
                self.position=matching_indices[0]
                self.moving_precent=0
                print("Vehicle Name:", self.id, ", Position:", self.position,"end waiting")
        if edges[self.position].id==self.route[-1]:
            print("Destination Reached")
            edges[self.position].arrive_count+=1
            return 1
        return 0
class edge:
    def __init__(self, id,length,speed_limit=1e9 ):
        self.id = id
        self.length=length
        self.from_connections=[] # 从这个lane出发的connection
        self.to_connections=[] # 到这个lane的connection
        self.from_edges=[] # 指向这个lane的edge
        self.to_edges=[] # 从这个lane指向的edge
        self.depart_count=0  
        self.arrive_count=0
        self.speed_limit=speed_limit
        self.display()
    def display(self):
        print("Edge Name:", self.id, ", Length:", self.length, ", Speed Limit:", self.speed_limit)
class connection:
    def __init__(self,from_edge,to_edge,via=None,linkIndex=-1,dir=None,state=None):
        self.from_edge=from_edge
        self.to_edge=to_edge
        self.via=via
        self.linkIndex=linkIndex
        self.dir=dir
        self.state=state
        self.waiting_vehicles=[]
        self.total_waiting_time=0
        self.display()
    def display(self):
        print("Connection Name:", self.from_edge, "->", self.to_edge, ", Via:", self.via, ", LinkIndex:", self.linkIndex)
    def waiting_time_calculation(self):
        for vehicle in self.waiting_vehicles:
            vehicle.total_waiting_time+=1
            self.total_waiting_time+=1
class request:# 动态交通信号
    def __init__(self,index,response):
        self.index=index
        self.response=response
        self.display()
    def display(self):
        print("Request Index:", self.index, ", Response:", self.response)
class junction:
    def __init__(self, id):
        self.id = id
        self.requests=[] # 动态交通信号
        self.traffic_light_ordinal=-1 # 静态交通信号
        self.connections=[None]*15 # 相连的lane 
        self.current_phase=None
        self.display()
        self.use_request=-1 #默认交通模式
    def display(self):
        print("Junction Name:", self.id)

class Phase:
    def __init__(self, duration, state):
        self.duration =int(duration)
        self.state = state
        self.display()
    def display(self):
        print("Duration:", self.duration, ", State:", self.state)
class tl_Logic:# 静态交通信号
    def __init__(self,id):
        self.phases = []
        self.id=id
        self.runtime=0
        self.current_phase=None
        self.display()
    def add_phase(self, duration, state):
        phase = Phase(duration, state)
        self.phases.append(phase)
    def change_phase(self):
        if (self.current_phase).duration==self.runtime:
            tmp_phase_index=self.phases.index(self.current_phase)
            self.current_phase=self.phases[(tmp_phase_index+1)%len(self.phases)]
            self.runtime=0
        else:
            self.runtime+=1
    # def present_current_phase(self):
    #     return self.phases[self.current_phase].state
    def display(self):
        print("Traffic Light Name:", self.id)
    

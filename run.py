from largecity import SUMONclass as sc
from largecity import netload as nl
from tqdm import tqdm # 仅debug需要
import time
import keyboard as ky
from gym.spaces import Box, Discrete
import sys
from bintrees import FastRBTree as RBTree
import random
import numpy as np
import cProfile



# global total_car_count
# 目前仅支持使用rou.xml文件生成flow,不支持使用trips文件生成trip
#不需要坐标系

  
#通过一个队列来存储所有将要加载的vehicle
#通过一个队列来更新所有正在运行的vehicle
# vehicle_loading_queue=[]
# vehicle_running_queue=[]


class Large_city:
    def __init__(self,net_path, rou_path):
        
        # if "newyork" or "cologne" in rou_path:
        #     from largecity import netload_newyork as nl
        # else:
        #     from largecity import netload_grid_monaco as nl
        
        
        self.edges,self.traffic_lights,self.junctions,self.connections=nl.netload(net_path)
        
        print("traffic_lights_num=",len(list(self.traffic_lights.values())))

        self.n_agents = len(list(self.traffic_lights.values()))
        self.n_agent = self.n_agents
        if self.n_agents == 25:
            self.MAX_CONNECTION_NUM=30
        elif self.n_agents == 28:
            self.MAX_CONNECTION_NUM=22
        elif self.n_agents == 436:
            self.MAX_CONNECTION_NUM=17
        elif self.n_agents == 1218:
            self.MAX_CONNECTION_NUM=30
        self.n_s=self.MAX_CONNECTION_NUM
        A = []
        for index, traffic_light in enumerate(self.traffic_lights.values()):
            A.append(len(traffic_light.phases))
        
        self.n_action = max(A)
        self.action_space = Discrete(self.n_action)
        #trips=nl.tripload(trip_path,route_tensor,edges)  
        self.flows,self.vehicles=nl.rou_load(rou_path,self.edges) #()
        self.vehicle_loading_queue=[]
        self.vehicle_running_queue=[]
        

        
        max_junction_num = 0
        for traffic_light in self.traffic_lights.values():
            max_junction_num = max(max_junction_num, len(traffic_light.junctions))
        self.state_dim = max_junction_num*self.MAX_CONNECTION_NUM
        
        self.valid_junctions=RBTree()
        for index,traffic_light in enumerate(self.traffic_lights.values()):
            self.valid_junctions.insert(index,traffic_light.id)
        self.reverse_valid_junctions=RBTree()
        for index,traffic_light in self.valid_junctions.items():
            self.reverse_valid_junctions.insert(traffic_light,index)
        

        self.neighbor_mask = self.get_adjacent_matrix()





    
    def launch_vehicle(self, current_time):
        if len(self.vehicle_loading_queue)==0:
            return
        
        if self.n_agents < 100:
            max_load_time = 500

        elif self.n_agents > 200:
            max_load_time = 2000


        while len(self.vehicle_loading_queue)!=0 and ((self.vehicle_loading_queue[0]).depart_time) <=current_time and current_time < max_load_time:
        #while len(self.vehicle_loading_queue)!=0:    # for 436 agents
            global total_car_count
            total_car_count+=1
            #print("Vehicle Name:", vehicle_loading_queue[0].id, ", Position:", vehicle_loading_queue[0].position,
                # "launched at:",current_time,", Speed:", vehicle_loading_queue[0].speed,
                # ", Direction:", vehicle_loading_queue[0].direction)
            self.edges[self.vehicle_loading_queue[0].position].depart_count+=1
            self.vehicle_running_queue.append(self.vehicle_loading_queue.pop(0))





    def initialization(self):
        self.vehicle_loading_queue=[]
        self.vehicle_running_queue=[]
        #初始化
        global total_car_count
        total_car_count=0
        for edge in self.edges.values():
            edge.depart_count=0
            edge.arrive_count=0
        for connection in self.connections.values():
            connection.total_waiting_time=0
            connection.waiting_vehicles=[]
        #1.traffic_lights状态更新
        for junction_id in self.valid_junctions.values():
            if junction_id not in self.junctions:
                continue  # 跳过当前循环迭代
            tmp_junction=self.junctions[junction_id]
            tmp_junction.current_phase=self.traffic_lights[junction_id].phases[0]
        for flow in self.flows.values():
            # 计算总迭代次数
            total_iterations = int((flow.end - flow.begin) / (float(60) / flow.vehsPerHour))
            # 初始化tqdm进度条
            pbar = tqdm(total=total_iterations)
            tmp_time = flow.begin
            interval_time = float(60) / flow.vehsPerHour
            while tmp_time < flow.end:
                self.vehicle_loading_queue.append(flow.vehicle_generate(tmp_time))
                tmp_time += interval_time
                pbar.update(1)  # 每次迭代更新进度条
            pbar.close() 
        for vehicle in self.vehicles.values():
            self.vehicle_loading_queue.append(vehicle)
        self.vehicle_loading_queue.sort(key=lambda x:x.depart_time)
        self.launch_vehicle(0)





    def step(self, action, current_time):
        #更新
        state=np.zeros((self.n_agents,self.state_dim))
        reward=np.zeros((self.n_agents,1))
        # 0.launch_vehicle
        if len(self.vehicle_loading_queue)!=0:
            self.launch_vehicle(current_time)
        # 1.vehicle 移动
        for vehicle in self.vehicle_running_queue:
            if vehicle.move(self.edges,self.junctions)==1:
                self.vehicle_running_queue.remove(vehicle)
        # 2.traffic_lights状态更新
        #for traffic_light in self.traffic_lights.values():
        for index, traffic_light in enumerate(self.traffic_lights.values()):
            traffic_light.change_phase(int(action[index]))
            i=0
            for junction in traffic_light.junctions:
                for connection in junction.connections:
                    if connection==None:
                        continue
                    state[index][i]=len(connection.waiting_vehicles)
                    i+=1
            reward[index]=-sum(state[index])
        # 3.junction状态更新
        for junction_id in self.valid_junctions.values():
            if junction_id not in self.junctions:
                continue  # 跳过当前循环迭代
            tmp_junction=self.junctions[junction_id]
            tmp_junction.current_phase=self.traffic_lights[junction_id].current_phase
        # 4.connection时间更新
        for connection in self.connections.values():
            connection.waiting_time_calculation()
            connection.current_past_count=0
        done = np.array([False]*self.n_agent)
        # 5.返回next_state和reward
        return state,reward,done,done


    def finish(self):
        total_depart_count=0
        total_arrive_count=0
        total_waiting_time=0
        for connection in self.connections.values():
            total_waiting_time+=connection.total_waiting_time
            #print("Connection in Junction:", connection.via," Waiting time:",connection.total_waiting_time)# 输出每个connection的等待时间
        for edge in self.edges.values():
            total_depart_count+=edge.depart_count
            total_arrive_count+=edge.arrive_count
            #print("Edge:",edge.id ," Depart count: ",edge.depart_count)
            #print("Edge:",edge.id ," Arrive count: ",edge.arrive_count)
        print("Total Depart count: ",total_depart_count)
        print("Total Arrive count: ",total_arrive_count)
        print("Total Car Count: ",total_car_count)
        print("Total Waiting Time: ",total_waiting_time)
        print("Simulation Finished")
    
    # 获取车辆位置在车道上的分布统计
    def get_vehicle_position(self, vehicles,edges):
        vehicle_position_distribution = {}
        for vehicle in vehicles:
            if vehicle.position in vehicle_position_distribution:
                vehicle_position_distribution[vehicle.position] += 1
            else:
                vehicle_position_distribution[vehicle.position] = 1
        return vehicle_position_distribution
         
    # 获取各个connection的等待车辆数量   
    def get_waiting_list(self, connections):
        waiting_lists={}
        for i in range(len(connections)):
            waiting_lists[i]=len(connections[i].waiting_vehicles)
        return waiting_lists
            
    # 返回所有交通灯当前的phase
    def get_traffic_light_waiting_time(self, junctions):
        total_waiting_time = 0
        for connection in self.connections.values():
            total_waiting_time+=connection.total_waiting_time
        return total_waiting_time
    
    # 重置环境
    def reset(self):
        self.initialization()
        
    # 获取当前的状态信息
    def get_state_(self):
        state=np.zeros((self.n_agents,self.state_dim))
        for index, traffic_light in enumerate(self.traffic_lights.values()):
            i=0
            for junction in traffic_light.junctions:
                for connection in junction.connections:
                    if connection==None:
                        continue
                    state[index][i]=len(connection.waiting_vehicles)
                    i+=1
        return state
    def get_adjacent_matrix(self):
        adjacent_matrix = np.zeros((self.n_agents, self.n_agents))
        for index,junction_id in tqdm(self.valid_junctions.items()):
            if junction_id not in self.junctions:
                continue  # 跳过当前循环迭代
            tmp_junction=self.junctions[junction_id]
            for junction_id in tmp_junction.junction_point_to:
                if junction_id==None:
                    continue
                if self.reverse_valid_junctions.__contains__(junction_id):
                    tmp_index=self.reverse_valid_junctions[junction_id]
                    adjacent_matrix[index][tmp_index]=1
            
        return adjacent_matrix


def Large_city_without_sumo():

    #dijistra_cache='/home/chengdong/MA-Processor/algorithms/envs/largecity/route_tensor.npy'  


    # net_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/exp.net.xml'                   #25
    # rou_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/exp_30.rou.xml'


    # net_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/most.net.xml'
    # rou_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/most.rou.xml'                    # 28

    
    net_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/newyork_map.net.xml'          #436
    rou_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/newyork_map.rou.xml'


    # net_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/cologne/cologne.net.xml'          #1218
    # rou_path='/home/chengdong/MA-Processor/algorithms/envs/largecity/cologne/cologne.rou.xml'

    return Large_city(net_path, rou_path)


if __name__ == '__main__':

    end_time=4000

    # net_path='./largecity/Grid/exp.net.xml'                        # 25
    # rou_path='./largecity/Grid/exp_30.rou.xml'

    # net_path='./largecity/cologne/cologne.net.xml'          #1218
    # rou_path='./largecity/cologne/cologne.rou.xml'

    net_path='largecity\\newyork\\newyork_map.net.xml'          #436
    rou_path='largecity\\newyork\\newyork_map.rou.xml'


    env = Large_city(net_path, rou_path)
    env.reset()
    pbar = tqdm(total=end_time)
    for current_time in range(end_time):
        action = [0]*env.n_agents    #这里假设给的action都为0

        cur_state = env.get_state_()
        #print("cur_state=",cur_state.shape)
        next_state, reward, done, _ = env.step(action,current_time)
        #print("next_state=",next_state.shape)
        #print("reward=",reward.shape)
        """
        需要有以下几个函数或的返回值：
        
        1.环境输入action（每个交通灯的相位）后，环境step返回下一个状态和奖励（均为numpy数组），为如下形式
        next_state, reward = env.step(action,current_time)
        其中，next_state的shape为 （env.n_agents, state_dim）
        其中每个交通灯的状态为当前路口的所有车道的等待车辆数量，比如路口A有四个车道,每个车道上的等待车辆数量分别为 4，2，3，6，则路口A的状态为[4，2，3，6].其state_dim为4
        以此类推，还有路口B,C,D，...
        
        其中reward的shape为 （env.n_agents, 1），其中每个路口的奖励为  负的（当前路口所有等待车辆数量之和），比如路口A的奖励值为 -（4+2+3+6）=-15
        
        
        2. 环境需要一个reset函数，用于重置所有环境状态
        
        env.reset()  
        
        这里的reset函数应该和现在代码中的env.initialization()是大概一样的？
        
        3.环境需要一个函数用来获取当前的状态信息 cur_state
        
        cur_state = env.get_state()
        
        其中cur_state的shape和next_state相同
        
        
        """
        
        
        #waiting_lists = env.get_waiting_list(env.connections)     
        #print("waiting_lists=",waiting_lists)
        
        pbar.update(1)  # 每次迭代更新进度条
    pbar.close()
    env.finish()



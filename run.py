import SUMON_class as sc
import netload as nl
from tqdm import tqdm # 仅debug需要
import time



net_path = 'exp.net.xml'
#trip_path = 'trips.trips.xml'
rou_path='exp_30.rou.xml'
edges,traffic_lights,junctions,connections,route_tensor=nl.netload(net_path)
#trips=nl.tripload(trip_path,route_tensor,edges)  
flows=nl.rou_load(rou_path,route_tensor,edges)  
launch_time=0
end_time=1000
global total_car_count
# 目前仅支持使用rou.xml文件生成flow,不支持使用trips文件生成trip
#不需要坐标系

# 考虑全部列表化，不通过id查询？可能溢出？32位python的限制是 536870912 个元素。64位python的限制是 1152921504606846975 个元素。
#通过一个队列来存储所有将要加载的vehicle
#通过一个队列来更新所有正在运行的vehicle
vehicle_loading_queue=[]
vehicle_running_queue=[]
def launch_vehicle(current_time):
    if len(vehicle_loading_queue)==0:
        return
    while len(vehicle_loading_queue)!=0 and ((vehicle_loading_queue[0]).depart_time) <=current_time:
        global total_car_count
        total_car_count+=1
        print("Vehicle Name:", vehicle_loading_queue[0].id, ", Position:", vehicle_loading_queue[0].position,
              "launched at:",current_time,", Speed:", vehicle_loading_queue[0].speed,
              ", Direction:", vehicle_loading_queue[0].direction)
        edges[vehicle_loading_queue[0].position].depart_count+=1
        vehicle_running_queue.append(vehicle_loading_queue.pop(0))
def initialization():
    #初始化
    global total_car_count
    total_car_count=0
    #1.traffic_lights状态更新
    for junction in junctions:
        # print(junction.id,junction.traffic_lights.phases[0])
        junction.current_phase=traffic_lights[junction.traffic_light_ordinal].current_phase
    for flow in flows:
        # 计算总迭代次数
        total_iterations = int((flow.end - flow.begin) / (float(60) / flow.vehsPerHour))
        # 初始化tqdm进度条
        pbar = tqdm(total=total_iterations)
        tmp_time = flow.begin
        interval_time = float(60) / flow.vehsPerHour
        while tmp_time < flow.end:
            vehicle_loading_queue.append(flow.vehicle_generate(tmp_time))
            tmp_time += interval_time
            pbar.update(1)  # 每次迭代更新进度条
        pbar.close() 
    vehicle_loading_queue.sort(key=lambda x:x.depart_time)
    launch_vehicle(0)

def update(current_time):
    #更新
    # 0.launch_vehicle
    launch_vehicle(current_time)
    # 1.vehicle 移动
    for vehicle in vehicle_running_queue:
        if vehicle.move(edges,junctions)==1:
            vehicle_running_queue.remove(vehicle)
    # 2.traffic_lights状态更新
    for traffic_light in traffic_lights:
        traffic_light.change_phase()
    # 3.junction状态更新
    for junction in junctions:
        junction.current_phase=traffic_lights[junction.traffic_light_ordinal].current_phase
    # 4.connection时间更新
    for connection in connections:
        connection.waiting_time_calculation()

def finish():
    total_depart_count=0
    total_arrive_count=0
    total_waiting_time=0
    for connection in connections:
        total_waiting_time+=connection.total_waiting_time
        print("Connection in Junction:", connection.via," Waiting time:",connection.total_waiting_time)# 输出每个connection的等待时间
    for edge in edges:
        total_depart_count+=edge.depart_count
        total_arrive_count+=edge.arrive_count
        print("Edge:",edge.id ," Depart count: ",edge.depart_count)
        print("Edge:",edge.id ," Arrive count: ",edge.arrive_count)
    print("Total Depart count: ",total_depart_count)
    print("Total Arrive count: ",total_arrive_count)
    print("Total Car Count: ",total_car_count)
    print("Total Waiting Time: ",total_waiting_time)
    print("Simulation Finished")
if __name__ == '__main__':
    # 记录开始时间
    start_time = time.time()
    initialization()
    pbar = tqdm(total=end_time)
    for current_time in range(launch_time+1,end_time):
        update(current_time)
        pbar.update(1)  # 每次迭代更新进度条
    pbar.close()
    finish()
    end_time = time.time()

    # 计算运行时长
    duration = end_time - start_time

    print(f"程序运行了 {duration} 秒。")

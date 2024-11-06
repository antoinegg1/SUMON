from largecity import SUMONclass as sc
import xml.etree.ElementTree as ET
import numpy as np
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import Manager
from itertools import repeat
from tqdm import tqdm
from bintrees import RBTree 
import pickle
import zlib
import heapq
import copy
import os

paths_cache ={}
def calculate_distance(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def netload(net_path):
    root = ET.parse(net_path).getroot()
    edges = RBTree()
    traffic_lights = RBTree()
    junctions=RBTree()
    connections=RBTree()   
    
    
    for edge in root.findall('edge'):
        lane=edge.find('lane')
        shape_attr = lane.attrib['shape']
        shape_coords = [tuple(map(float, point.split(',')[:2])) for point in shape_attr.split()]
        total_length = sum(calculate_distance(shape_coords[i], shape_coords[i+1]) for i in range(len(shape_coords)-1))
        if edge.get('speed') is None:
            edge=sc.edge(edge.get('id'),total_length)
        else:
            edge=sc.edge(edge.get('id'),total_length,edge.get('speed'))
        # for connection in connections.values():
        #     if connection.from_edge==edge.id:
        #         edge.from_connections.append(connection)
        #         edge.to_edges.append(connection.to_edge)
        #         count+=1
        #     if connection.to_edge==edge.id:
        #         edge.to_connections.append(connection)
        #         edge.from_edges.append(connection.from_edge)
        #         count+=1
        edges.insert(edge.id,edge)
    
    for junction in root.findall('junction'):
        tmp_junction=sc.junction(junction.get('id'))
        for request in junction.findall('request'):
            tmp_junction.requests.append(sc.request(request.get('index'),request.get('response')))
        # for connection in connections.values():
        #     if connection.via==tmp_junction.id:
        #         print(int(connection.linkIndex))
        #         tmp_junction.connections[int(connection.linkIndex)]=connection  
        #for tl in traffic_lights.keys():
             #if tl==tmp_junction.id:
        #         tmp_junction.traffic_light=tl
        junctions.insert(tmp_junction.id,tmp_junction)
    for tl_logic in root.findall('tlLogic'):
        tl=sc.tl_Logic(tl_logic.get('id'))
        for phase in tl_logic.findall('phase'):
            tl.add_phase(phase.get('duration'),phase.get('state'))
        tl.current_phase=tl.phases[0]
        traffic_lights.insert(tl.id,tl)
        if junctions.__contains__(tl.id):
            junctions[tl.id].traffic_light=tl.id
            tl.junctions.append(junctions[tl.id])
    for i,connection in enumerate(root.findall('connection')):
        connection=sc.connection(connection.get('from'),connection.get('to'),connection.get('tl'),
                                 connection.get('linkIndex'),connection.get('dir'),connection.get('state'))
        connection.index=i
        connections.insert(i,connection)
        if edges.__contains__(connection.from_edge):
            edges.get_value(connection.from_edge).to_edges.append(connection.to_edge)
            edges.get_value(connection.from_edge).from_connections.append(connection)
        if edges.__contains__(connection.to_edge):
            edges.get_value(connection.to_edge).from_edges.append(connection.from_edge)
            edges.get_value(connection.to_edge).to_connections.append(connection)
        if connection.via is not None:
            if junctions.__contains__(connection.via):
                junctions.get_value(connection.via).connections[int(connection.linkIndex)]=connection
                junctions.get_value(connection.via).edge_point_to.append(connection.to_edge)
                junctions.get_value(connection.via).edge_point_in.append(connection.from_edge)
    for junction in junctions.values():
        for edge_id in junction.edge_point_to:
            tmp_edge=edges.get_value(edge_id)
            for connection in tmp_edge.from_connections:
                junction.junction_point_to.append(connection.via)
        for edge_id in junction.edge_point_in:
            tmp_edge=edges.get_value(edge_id)
            for connection in tmp_edge.to_connections:
                junction.junction_point_in.append(connection.via)
        
    # if dijistra_cache is not None:
    #     route_tensor=np.load(dijistra_cache)
        # else:
    #route_tensor=calculate_shortest_paths(edges)
    #np.save('route_tensor.npy', route_tensor)
    # print(route_tensor)
    return edges,traffic_lights,junctions,connections
def tripload(trip_path,edges):
    root = ET.parse(trip_path).getroot()
    trips = RBTree()
    for trip in root.findall('trip'):
        tmp_trip=sc.trip(trip.get('id'),trip.get('depart'),trip.get('from'),trip.get('to'))
        tmp_trip.route=dijkstra(edges,trip.get('from'),trip.get('to'))
        if tmp_trip.route==[]:
            continue
        trips.insert(tmp_trip.id,tmp_trip)
    return trips
def rou_load(rou_path,edges):
    root = ET.parse(rou_path).getroot()
    rou=RBTree()
    vehicles=RBTree()
    with tqdm(total=len(root.findall('flow')), desc='Loading flows') as pbar:
        for flow in root.findall('flow'):
            tmp_flow=sc.flow(flow.get('id'),flow.get('departPos'),flow.get('from'),flow.get('to'),
                            flow.get('begin'),flow.get('end'),flow.get('vehsPerHour'))
            tmp_flow.route=dijkstra(edges,flow.get('from'),flow.get('to'))
            if tmp_flow.route==[]:
                continue
            #print(tmp_flow.route)
            rou.insert(tmp_flow.id,tmp_flow)
            pbar.update(1)
    for vehicle in root.findall('vehicle'):
        for route in vehicle.findall('route'):
            tmp_route=route.get('edges').split(" ")
        tmp_connection_list=[]
        for index,edge_id in enumerate(tmp_route):
            if index==len(tmp_route)-1:
                break
            for connection in edges[edge_id].from_connections:
                if connection.to_edge==tmp_route[index+1]:
                    tmp_connection_list.append(connection.index)
                    break
        vehicles.insert(vehicle.get('id'),sc.vehicle(vehicle.get('id'),tmp_route[0],tmp_route,tmp_connection_list,vehicle.get('depart')))
        
    return rou,vehicles
    #处理rou文件：flow,vehicle
def dijkstra(edges, start_id,end_id):
    distances={edge_id:float('inf') for edge_id in edges.keys()}
    previous = {edge_id: None for edge_id in edges.keys()}
    distances[start_id]=0
    # 初始化最小堆，并将所有节点加入，起始节点距离为0，其他为无穷大
    min_heap = [(0, start_id)]  # (距离, 节点ID)
    heapq.heapify(min_heap) 
    visited = set()
    while min_heap:
        current_distance, current_id = heapq.heappop(min_heap)
        
        if current_id in visited:
            continue
        
        visited.add(current_id)  # 标记为已访问
        if current_id == end_id:
            break
        # 获取当前节点的邻接节点信息
        current_edge = edges.get(current_id)
        for neighbor_id in current_edge.to_edges:
            if neighbor_id in visited:  # 确保只处理未访问的邻接节点
                continue
            alt = current_distance + 0.5*(current_edge.length+(edges.get(neighbor_id)).length)  # 更新距离
            if alt < distances[neighbor_id]:
                # 更新最短路径和前驱节点信息
                distances[neighbor_id] = alt
                previous[neighbor_id] = current_id
                # 更新邻接节点在堆中的距离（通过重新加入堆实现）
                heapq.heappush(min_heap, (alt, neighbor_id))

    path = []
    current_id = end_id
    if distances[end_id] == float('inf'):
        return []
    while current_id != start_id:
        path.insert(0, current_id)
        current_id = previous[current_id]
    path.insert(0, start_id)
    # print(path)
    return path

  


# # def calculate_shortest_paths(edges):
# #     T = RBTree()  # 假设RBTree的实现已经存在
# #     unvisited_keys = [(edge_id, True) for edge_id, _ in edges.items(reverse=False)]  # 未访问的节点标记为 True
# #     unvisited_template = RBTree(unvisited_keys)
    
# #     distances_keys = [(edge_id, float('inf')) for edge_id, _ in edges.items(reverse=False)]
# #     distances_template = RBTree(distances_keys)
# #     # 起始节点到自己的距离是0

# #     previous_keys = [(edge_id, None) for edge_id, _ in edges.items(reverse=False)]
# #     previous_template = RBTree(previous_keys)
# #     with tqdm(total=len(edges), desc='Calculating shortest paths') as pbar:
# #         for start_edge_id, _ in tqdm(edges.items(reverse=False)):
# #             tmp_unvisited = copy.deepcopy(unvisited_template)
# #             tmp_distances = copy.deepcopy(distances_template)
# #             tmp_previous = copy.deepcopy(previous_template)
# #             distances, previous = dijkstra( edges,start_edge_id,tmp_unvisited,tmp_distances,tmp_previous)
# #             for end_edge_id, _ in edges.items(reverse=False):
# #                 path = reconstruct_path(previous, start_edge_id, end_edge_id)
# #                 T.insert((start_edge_id, end_edge_id), path)  # 插入路径列表
# #             pbar.update(1)
# #     return T
# def process_edge_chunk(start_edge_id, edges,unvisited_template,distances_template,previous_template):
#     global paths_cache
#     # # tmp_unvisited = copy.deepcopy(unvisited_template)
#     # # tmp_distances = copy.deepcopy(distances_template)
#     # # tmp_previous = copy.deepcopy(previous_template)
#     if start_edge_id not in paths_cache:
#         distances, previous = dijkstra(edges, start_edge_id,unvisited_template,distances_template,previous_template)
#         paths_cache[start_edge_id] = {}
#         for end_edge_id in edges.keys():
            
#             if start_edge_id != end_edge_id:
#                 path = reconstruct_path(previous, start_edge_id, end_edge_id)
#                 paths_cache[start_edge_id][end_edge_id] = path
#             else:
#                 paths_cache[start_edge_id][end_edge_id] = [start_edge_id]
#     return paths_cache[start_edge_id]

# def calculate_shortest_paths(edges):
#     # 初始化模板
#     unvisited_template = RBTree([(edge_id, True) for edge_id, _ in edges.items(reverse=False)])
#     distances_template = RBTree([(edge_id, float('inf')) for edge_id, _ in edges.items(reverse=False)])
#     previous_template = RBTree([(edge_id, None) for edge_id, _ in edges.items(reverse=False)])
#     # num_workers = os.cpu_count()  # 获取CPU核心数
#     # edge_ids = [edge_id for edge_id, _ in edges.items(reverse=False)]
#     # chunk_size = len(edge_ids) // num_workers + 1  # 计算每个任务的大小
#     # chunks = [(edges, edge_ids[i:i + chunk_size], unvisited_template, distances_template, previous_template) 
#     #           for i in range(0, len(edge_ids), chunk_size)]

#     # manager = Manager()
#     # progress_queue = manager.Queue()

#     # 准备进程池和收集结果
#     edge_id_to_index = {edge_id: index for index, edge_id in enumerate(edges.keys())}
#     num_edges = len(edges)
#     T = np.empty((num_edges, num_edges), dtype=object) # 最终的结果也是一个红黑树
#     with  ProcessPoolExecutor() as executor:
#         # 异步执行任务并立即开始处理结果
#         # future_to_chunk = {executor.submit(process_edge_chunk, chunk): chunk for chunk in chunks}
#         future_to_edge_id = {executor.submit(process_edge_chunk, edge_id, edges,unvisited_template,distances_template,previous_template): edge_id for edge_id in edges.keys()}
        
#         # 根据进度队列更新进度条
#         with tqdm(total=num_edges, desc="Calculating paths") as progress:
#             for future in tqdm(as_completed(future_to_edge_id)):
#                 edge_id = future_to_edge_id[future]  # 这是 edge_id，非索引
#                 edge_index = edge_id_to_index[edge_id]  # 转换为整数索引
#                 try:
#                     paths_dict = future.result()
#                     for end_edge_id in edges.keys():
#                         j = edge_id_to_index[end_edge_id]  # 同样，转换为整数索引
#                         T[edge_index][j] = paths_dict.get(end_edge_id, None)  # 使用get以防止KeyError
#                 except Exception as exc:
#                     print(f'Edge {edge_id} generated an exception: {exc}')
#                 finally:
#                     progress.update(1)
#     return T
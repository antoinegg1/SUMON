import SUMON_class as sc
import xml.etree.ElementTree as ET
import numpy as np

def calculate_distance(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def netload(net_path):
    root = ET.parse(net_path).getroot()
    edges = []
    traffic_lights = []
    junctions=[]
    connections=[]   
                
    for tl_logic in root.findall('tlLogic'):
        tl=sc.tl_Logic(tl_logic.get('id'))
        for phase in tl_logic.findall('phase'):
            tl.add_phase(phase.get('duration'),phase.get('state'))
        tl.current_phase=tl.phases[0]
        traffic_lights.append(tl)
    
    for connection in root.findall('connection'):
        connection=sc.connection(connection.get('from'),connection.get('to'),connection.get('tl'),
                                 connection.get('linkIndex'),connection.get('dir'),connection.get('state'))
        connections.append(connection)
        
    for edge in root.findall('edge'):
        lane=edge.find('lane')
        shape_attr = lane.attrib['shape']
        shape_coords = [tuple(map(float, point.split(',')[:2])) for point in shape_attr.split()]
        total_length = sum(calculate_distance(shape_coords[i], shape_coords[i+1]) for i in range(len(shape_coords)-1))
        edge=sc.edge(edge.get('id'),total_length,edge.get('speed'))
        for connection in connections:
            if connection.from_edge==edge.id:
                edge.from_connections.append(connection)
                edge.to_edges.append(connection.to_edge)
            if connection.to_edge==edge.id:
                edge.to_connections.append(connection)
                edge.from_edges.append(connection.from_edge)
        edges.append(edge)
    
    for junction in root.findall('junction'):
        tmp_junction=sc.junction(junction.get('id'))
        for request in junction.findall('request'):
            tmp_junction.requests.append(sc.request(request.get('index'),request.get('response')))
        for connection in connections:
            if connection.via==tmp_junction.id:
                print(int(connection.linkIndex))
                tmp_junction.connections[int(connection.linkIndex)]=connection  
        for tl in range(len(traffic_lights)):
            if (traffic_lights[tl]).id==tmp_junction.id:
                tmp_junction.traffic_light_ordinal=tl
        junctions.append(tmp_junction)
    route_tensor=calculate_shortest_paths(edges)
    # print(route_tensor)
    return edges,traffic_lights,junctions,connections,route_tensor
def tripload(trip_path,route_tensor,edges):
    root = ET.parse(trip_path).getroot()
    trips = []
    for trip in root.findall('trip'):
        tmp_trip=sc.trip(trip.get('id'),trip.get('depart'),trip.get('from'),trip.get('to'))
        matching_indices_from = [index for index, edge in enumerate(edges) if edge.id== tmp_trip.origin]
        matching_indices_to = [index for index, edge in enumerate(edges) if edge.id == tmp_trip.destination]
        if matching_indices_from.__len__() == 0 or matching_indices_to.__len__() == 0:
            print("Error: edge not found")
            continue
        tmp_trip.route=route_tensor[matching_indices_from[0]][matching_indices_to[0]]
        trips.append(tmp_trip)
    return trips
def rou_load(rou_path,route_tensor,edges):
    root = ET.parse(rou_path).getroot()
    rou=[]
    for flow in root.findall('flow'):
        tmp_flow=sc.flow(flow.get('id'),flow.get('departPos'),flow.get('from'),flow.get('to'),
                         flow.get('begin'),flow.get('end'),flow.get('vehsPerHour'))
        matching_indices_from = [index for index, edge in enumerate(edges) if edge.id== tmp_flow.origin]
        matching_indices_to = [index for index, edge in enumerate(edges) if edge.id == tmp_flow.destination]
        if matching_indices_from.__len__() == 0 or matching_indices_to.__len__() == 0:
            print("Error: edge not found")
            continue
        tmp_flow.origin_index=matching_indices_from[0]
        tmp_flow.destination_index=matching_indices_to[0]
        tmp_flow.route=route_tensor[tmp_flow.origin_index][tmp_flow.destination_index]
        print(tmp_flow.route)
        rou.append(tmp_flow)
    return rou
    #处理rou文件：flow,vehicle
# 我们现在有edge一种点构成的有向图，edge存储在列表edges中，edge.from_edges可以获得指向它的edge的id的列表，
# edge.to_edges可以获得它指向的edge的id的列表，
# 请使用Dijkstra算法计算两个edge之间的最短路径，并给出详细注释，要求返回一个张量(tensor)T，
# T[i][j]表示edge[i]到edge[j]的最短路径列表
# ，列表存储了edge[i]到edge[j]的最短路径上的edge的id，如果edge[i]到edge[j]不可达，则T[i][j]为空列表。
def dijkstra(edges, start_id):
    # 初始化
    unvisited = set(edge.id for edge in edges)  # 未访问的edge的id集合
    distances = {edge.id: float('inf') for edge in edges}  # 存储到每个edge的最短距离
    previous = {edge.id: None for edge in edges}  # 存储到每个edge的最短路径的前驱edge

    distances[start_id] = 0  # 起始edge到自己的距离是0

    while unvisited:
        # 选择当前距离最短的未访问edge
        current_id = min(unvisited, key=lambda id: distances[id])
        current_distance = distances[current_id]
        
        if current_distance == float('inf'):
            break  # 剩下的edge都是不可达的

        # 更新当前edge的邻接edge的距离
        current_edge = next(edge for edge in edges if edge.id == current_id)
        for neighbor_id in current_edge.to_edges:
            alt = current_distance + 1  # 假设每个边的权重为1
            if alt < distances[neighbor_id]:
                distances[neighbor_id] = alt
                previous[neighbor_id] = current_id

        unvisited.remove(current_id)

    return distances, previous

def reconstruct_path(previous, start_id, end_id):
    path = []
    current_id = end_id
    while current_id != start_id:
        if previous[current_id] is None:
            return []  # 不可达
        path.insert(0, current_id)
        current_id = previous[current_id]
    path.insert(0, start_id)  # 添加起始点
    return path

def calculate_shortest_paths(edges):
    num_edges = len(edges)
    T = np.empty((num_edges, num_edges), dtype=object)

    for i, start_edge in enumerate(edges):
        distances, previous = dijkstra(edges, start_edge.id)
        for j, end_edge in enumerate(edges):
            if i != j:
                path = reconstruct_path(previous, start_edge.id, end_edge.id)
                T[i][j] = path
            else:
                T[i][j] = [start_edge.id]  # 自己到自己的路径

    return T

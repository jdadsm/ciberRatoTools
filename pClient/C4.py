
from asyncio import sleep
import sys
from croblink import *
import math
import xml.etree.ElementTree as ET
from itertools import permutations

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    
    
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.h = 0.05

        self.Kp = 0.7
        self.Ti = 1/self.h
        self.Td = 1*self.h

        self.e_m1 = 0
        self.e_m2 = 0
        self.u_m1 = 0

        self.max_u = 10

        self.estado = "explore"
        
        self.finishing = False # Used to return to (0,0)

        #list of intersections
        self.vertices = []

        #ultimo goal 
        self.last_goal = [0,0]

        #ultima orientacao
        self.last_orientation = 0

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def print_map(self):
        for line in reversed(self.map):
            for element in line:
                if element == 0:
                    print(" ", end = "")
                else:
                    print(element, end="")
            print()

    def print_map_to_file(self):
        file = open("map_c2.map", "w") #write to file
        for line in reversed(self.map):
            for element in line:
                if element == 0:
                    file.write(" ")
                else:
                    file.write(element)
            file.write('\n')
        file.close()

    def PID(self,r,y):
        u = 0
        
        K0 = self.Kp*(1+self.h/self.Ti+self.Td/self.h)
        K1 = -self.Kp*(1+2*self.Td/self.h)
        K2 = self.Kp*self.Td/self.h
        
        e = r-y
        
        u = self.u_m1 + K0*e + K1*self.e_m1 + K2*self.e_m2
            
        self.e_m2 = self.e_m1
        self.e_m1 = e
        self.u_m1 = u
        
        if u > self.max_u:
            u = self.max_u
        if u < -self.max_u:
            u = -self.max_u
            
        #print("u:",u)
        return u
    
    def getLinePos(self,line):
        posOverLine = 0
        nActiveSensors = 0
        
        for i in range(7):
            if line[i] == "1":
                posOverLine += i-3
                nActiveSensors+=1
        
        if nActiveSensors != 0:
            posOverLine = 0.08*posOverLine/nActiveSensors
        else:
            return None
        
        return posOverLine
    
    def calculate_ones(self, line):
        
        ones_left = line[0:3].count("1")
        ones_right = line[4:7].count("1")

        return ones_left-ones_right

    def run(self):
        self.initialize_map()

        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.readSensors()

        BUFFER_DEFAULT = ["0","0","1","1","1","0","0"]      
        BUFFER_SIZE = 5
        buffer = []
        for i in range(BUFFER_SIZE):
            buffer.append(BUFFER_DEFAULT)
            
        previous_pos = [0.0,0.0]
        previous_compass = 0
        previous_out = [0,0]
        current_in = [0,0]
            
        velSetPoint = 0.15
        compass_extra_laps = 0
        pos_inicial_real = [self.measures.x, self.measures.y]

        sensor = 0

        exact_sensor = self.get_exact_sensor_value(sensor)
        goal = self.get_next_goal(exact_sensor, 0, 0)

        intersections = {}
        graph = {}

        alpha_extra_laps = 0
        sensor_extra_laps = 0

        last_coords = (0,0)
        backtracking = False
        beacons = [(0,0)]
        
        for i in range(int(self.nBeacons)-1):
            beacons.append(None)
            
        while True:
            self.readSensors()
            
            line = self.measures.lineSensor
            
            sensor = self.measures.compass #compass - em graus 

            orientation_string = self.get_orientation_string(sensor)

            x = self.measures.x - pos_inicial_real[0]
            y = self.measures.y - pos_inicial_real[1]

            if self.measures.ground != -1:
                beacons[self.measures.ground] = (round(x),round(y))
            
            if abs(x-goal[0]) <= 0.15 and abs(y-goal[1]) <= 0.15: 

                if self.estado == "backtrack":
                    self.estado = "explore"

                self.update_graph(x,y,graph,last_coords)
                
                if self.finishing:
                    if (round(x),round(y)) == (0,0):
                        self.finish_C3(beacons,graph)
                    goal = self.dijkstra(graph,(round(x),round(y)),(0,0))[0][1]
                else:
                
                    #print("Exact sensor value:", self.get_exact_sensor_value(sensor))
                    if (round(x),round(y)) not in intersections:
                        intersections.setdefault((round(x), round(y)), set())
                        #open paths e uma lista sem items repetidos
                        open_paths = self.get_open_paths_for_intersection(buffer, x, y, self.get_exact_sensor_value(sensor))
                        intersections[(round(x), round(y))].update(open_paths)
                        
                        #print("intersections:",intersections)
                        
                        if intersections[(round(x),round(y))]:  # if there are still open paths in this intersection, choose one and explore it
                            self.last_goal = goal
                            goal = self.get_next_goal(intersections[(round(x),round(y))].pop(),goal[0],goal[1]) 
                        else:                                   # if not go back
                            self.last_goal,goal = goal,self.last_goal
                    else:
                        if intersections[(round(x),round(y))]:
                            self.last_goal = goal
                            goal = self.get_next_goal(intersections[(round(x),round(y))].pop(),goal[0],goal[1]) 
                        else:
                            if not backtracking:
                                goal = self.minimun_weight_path(intersections, graph, x, y)
                                if goal == None:
                                    continue

                self.estado = "rotate"
                last_coords = (round(x),round(y))
            
            if None not in beacons and not self.finishing:
                remaining = [i for i in intersections if intersections[i]!=set()]
                if len(remaining) == 0:
                    self.update_graph(x,y,graph,last_coords)
                    self.finishing = True

            tight_turn = False
            gps_pos = [x,y]
            
            #print("Sensor:",sensor)
            print("Posição GPS:",gps_pos)
            
            #print("Line:",line)
            original_line = line
            for i in range(len(line)):
                if line[i] != buffer[0][i]:
                    line[i] == "1"

            posOverLine = self.getLinePos(line)
            
            #print("posOverLine:",posOverLine)
            
            if posOverLine == None:
                xt = round(xt / 2.0) * 2.0
                yt = round(yt / 2.0) * 2.0
                
                val = 0
                for l in buffer:
                    val+=self.calculate_ones(l)
                res = [0,0,0,0,0,0,0]
                for l in buffer:
                    for i in range(len(l)):
                        if l[i] == "1":
                            res[i]+=1
                #print("res:",res)
                #print("val:",val)
                #print("buffer:",buffer)
                if val > 0:
                    lPow,rPow = (-0.15,0.15)
                else:
                    lPow,rPow = (0.15,-0.15)
                tight_turn = True
            
            
            if not tight_turn:
                PID = self.PID(0, posOverLine)
                lPow = velSetPoint - PID
                rPow = velSetPoint + PID
                
            self.driveMotors(lPow,rPow)
            
            current_in = [lPow,rPow]
            if current_in[0] > 0.15:
                current_in[0] = 0.15
            if current_in[0] < -0.15:
                current_in[0] = -0.15
            if current_in[1] > 0.15:
                current_in[1] = 0.15
            if current_in[1] < -0.15:
                current_in[1] = -0.15
            out = [(previous_out[0]+current_in[0])/2,(previous_out[1]+current_in[1])/2]
            
            lin = (out[0] + out[1])/2
            
            #print("lin:",lin)
            
            xt = round(previous_pos[0] + lin*math.cos(previous_compass),1)
            yt = round(previous_pos[1] + lin*math.sin(previous_compass),1)
            
            rot = math.degrees(out[1]-out[0])
            
            compass = round(previous_compass + rot,0)
            
            if compass > 180:
                compass -= 360
            if compass < -180:
                compass += 360
            
            #print("previous_compass:",previous_compass)
            #print("compass:",compass)
            print("Predict Posição:",[xt,yt])
            #print("Predict Sensor:",compass)
            #print("Current in:",current_in)
            
            print("Diff Posição:",[x-xt,y-yt])
            print("Diff Sensor:",sensor-compass)
            #print("Current in:",current_in)
            
            previous_pos = [xt,yt]
            previous_compass = compass
            previous_out = out
            
            buffer = buffer[0:BUFFER_SIZE]
            buffer = [original_line] + buffer

            if line[2:5].count("1") >= 2:
                if (orientation_string != "-" and orientation_string != "|"):
                    if(round(x)%2 == 1 and round(y)%2 == 1):
                        self.put_in_map(round(x), round(y), orientation_string)
                        self.print_map_to_file()
                else:
                    if (round(x)%2 == 1 or round(y)%2 == 1):
                        self.put_in_map(round(x), round(y), orientation_string)
                        self.print_map_to_file()


    def finish_C3(self,beacons,graph):    
        print(beacons)
        cost = None
        solution = None
        #func to add beacon to graph
        for combination in permutations(beacons[1:],len(beacons)-1):
            #print("combination:",combination)
            temp = []
            temp.append((0,0))
            temp.extend(combination)
            temp.append((0,0))
            #print("temp:",temp)
            possible_solution = []
            possible_cost = 0
            for i in range(len(temp)-1):
                temp_dijkstra = self.dijkstra(graph,temp[i],temp[i+1])
                if i == 0:
                    possible_solution += temp_dijkstra[0]
                    possible_cost += temp_dijkstra[1]
                else:
                    possible_solution += temp_dijkstra[0][1:]
                    repeated_points_weight = self.get_distance(temp_dijkstra[0][0],temp_dijkstra[0][1])
                    possible_cost += temp_dijkstra[1] - repeated_points_weight
                #print("test:",possible_solution)
                #print(temp[i])
                #print(temp[i+1])
            if cost is None:
                cost = possible_cost
                solution = possible_solution
            else:
                if possible_cost < cost:
                    cost = possible_cost
                    solution = possible_solution
        
        print("Solution:",solution)
        file = open("pathC3.path", "w")
        for coord in solution:
            file.write(f"{coord[0]} {coord[1]}\n")
        self.driveMotors(0.0,0.0)
        self.finish()
        exit(0)

    def update_graph(self,x,y,graph,last_coords):
        if (round(x),round(y)) in graph:        
            weight = self.get_distance((round(x),round(y)),last_coords)
            if weight > 0:
                graph[(round(x),round(y))][last_coords] = weight
            if last_coords not in graph:
                graph[last_coords] = {}     #PROBABLY DEVIAMOS AUTALIZAR ISTO, ADICIONAR LIGACAO AO QUE TEMOS
            weight = self.get_distance(last_coords,(round(x),round(y)))
            if weight > 0:
                graph[last_coords][(round(x),round(y))] = weight
            
        else:
            if last_coords:                 # se houver ponto de origem e goal nao existe no no grafo
                graph[(round(x),round(y))] = {}
                weight = self.get_distance((round(x),round(y)),last_coords)
                if weight > 0:                            
                    graph[(round(x),round(y))][last_coords] = weight
                if last_coords not in graph:
                    graph[last_coords] = {}
                weight = self.get_distance(last_coords,(round(x),round(y)))
                if weight > 0:
                    graph[last_coords][(round(x),round(y))] = weight
        
            print("\nGraph:")
            for node, connections in graph.items():
                print(f"{node} - Connections:")
                for neighbor, weight in connections.items():
                    print(f"  -> {neighbor} (Weight: {weight})")

    def get_orientation_string(self, sensor):
        """this function takes a sensor value and converts it to a string representation of an orientation:
            - -> horizontal line                    -
            | -> vertical line                      | 
            / -> oblique line (tan is positive)     /
            \ -> oblique line (tan is negative)     \
            0 -> empty (should not be returned under any circunstance)
        """
        if sensor < -180 or sensor > 180:
            print("out of bounds")
            return 0

        if (sensor <= 22.5 and sensor >= -22.5) or (sensor >= 157.5 and sensor <= 180) or (sensor>=180 or sensor<=-157.5):
            return '-'
        elif (sensor <= -67.5 and sensor >= -112.5) or (sensor >= 67.5 and sensor <= 112.5):
            return '|'
        elif (sensor < -112.5 and sensor > -157.5) or (sensor > 22.5 and sensor < 67.5):
            return '/'
        elif (sensor < -22.5 and sensor > -67.5) or (sensor > 112.5 and sensor < 157.5):
            return chr(92) # \ character
        else:
            print("Orientation interval was not found...")
            return 0

    def put_in_map(self, x, y, orientation):
        x_map = 24 + x
        y_map = 10 + y

        self.map[y_map][x_map] = orientation
    
    def initialize_map(self):
        lines = 21
        columns = 49

        self.map = [[0 for x in range(columns)] for y in range(lines)] #melhor fazer com variavel global...

        self.map[10][24] = "I"

    def get_exact_sensor_value(self, sensor):
        """
            this function takes a sensor value and returns the closest sensor value from the possible directions in the map 
        """ 
        if sensor < -180:
            return self.get_exact_sensor_value(sensor+360)
        elif sensor > 180:
            return self.get_exact_sensor_value(sensor-360)
        
        if (sensor <= 22.5 and sensor >= 0) or (sensor <= 0 and sensor >= -22.5):
            return 0
        elif (sensor >= 22.5 and sensor <= 67.5):
            return 45
        elif (sensor >= 67.5 and sensor <= 112.5):
            return 90
        elif (sensor >= 112.5 and sensor <= 157.5):
            return 135
        elif (sensor >= 157.5 and sensor <= 180) or (sensor <= -157.5 and sensor >= -180):
            return -180
        elif (sensor >= -157.5 and sensor <=-112.5):
            return -135
        elif (sensor >= -112.5 and sensor <= -67.5):
            return -90
        elif (sensor >= -67.5 and sensor <= -22.5):
            return -45
        else:
            return None

    def get_next_goal(self, sensor, x, y):
        """
        This function takes the position of the robot the sensor value of the line where he is at the moment and return the position to have as a next goal
        """
        #print("Sensor:",sensor)
        new_goal = None
        possible_new_goals = [[x+2, y],[x+2, y+2],[x, y+2],[x-2, y+2],[x-2, y],[x-2, y],[x+2, y-2],[x, y-2],[x-2, y-2]]
        sensor_values = [0,45,90,135,-180,-180,-45,-90,-135]
        
        for i in range(len(possible_new_goals)):
            if sensor == sensor_values[i]:
                new_goal = possible_new_goals[i]
        
        if new_goal:
            print("New goal:",new_goal)
        else:
            print("Error in get_next_goal")
        return new_goal

    def convert_orientation_to_list_index(self, orientation):
        possible_orientations = [0,45,90,135,-180,-180,-135,-90,-45]
        return_indexes = [0,1,2,3,4,4,5,6,7]
        
        for i in range(len(possible_orientations)):
            if possible_orientations[i] == orientation:
                return return_indexes[i]

    def get_middle_buffer_paths(self,buffer,exact_sensor):
        paths = []

        if buffer[0].count('1') >= 2:
            paths.append(exact_sensor)

        for b in buffer:
            if b.count('1') <= len(b)-2:
                if exact_sensor in paths:
                    paths.remove(exact_sensor)

        if self.is_sublist_of(buffer,[['1', '1', '1'], ['0', '1', '1']]):
            paths.append(exact_sensor+90)
        elif self.is_sublist_of(buffer, [['1', '1', '1'],['1', '1', '0']]):
            paths.append(exact_sensor-90)


        middle_left_buffer = self.get_left_buffer_paths(buffer[:][0:2], exact_sensor)
        middle_right_buffer = self.get_right_buffer_paths(buffer[:][1:3], exact_sensor)

        #print("Extensions right: ", middle_left_buffer)
        #print("Extensions left: ", middle_right_buffer)


        paths.extend(middle_left_buffer)
        paths.extend(middle_right_buffer)

        for i in range(len(paths)):
            if paths[i] >= 180:
                paths[i] = paths[i] - 360
            if paths[i] <-180:
                paths[i] = paths[i] + 360

        return paths

    def get_left_buffer_paths(self,buffer,exact_sensor):
        paths = []
        if len(buffer) <= 1:
            return paths
        if self.is_sublist_of(buffer,[['0', '0'], ['1', '1']]):
            paths.append(90+exact_sensor)
        if self.is_sublist_of(buffer,[['0', '0'], ['0', '1'], ['1', '1']]) or self.is_sublist_of(buffer, [['1', '1'], ['1', '0']]):
            paths.append(135+exact_sensor)
        if self.is_sublist_of(buffer,[['1', '0'], ['1', '1'], ['0', '1']])  or self.is_sublist_of(buffer,[['1', '1'], ['0', '1']]):
            paths.append(45+exact_sensor)
        
        for i in range(len(paths)):
            if paths[i] >= 180:
                paths[i] = paths[i] - 360
            if paths[i] <-180:
                paths[i] = paths[i] + 360
        
        return paths
    
    def get_right_buffer_paths(self,buffer,exact_sensor):
        paths = []
        if len(buffer) <= 1:
            return paths
        if self.is_sublist_of(buffer,[['0','0'],['1','1']]):
            paths.append(-90+exact_sensor)
        if self.is_sublist_of(buffer,[['0', '0'], ['1', '0'], ['1', '1']]) or self.is_sublist_of(buffer, [['1', '1'], ['0', '1']]):
            paths.append(-135+exact_sensor)
        if self.is_sublist_of(buffer,[['0', '1'], ['1', '1'], ['1', '0']]) or self.is_sublist_of(buffer,[['1', '1'], ['1', '0']]):
            paths.append(-45+exact_sensor)
        
        for i in range(len(paths)):
            if paths[i] < -180:
                paths[i] = paths[i] + 360    
        return paths
    
    def is_sublist_of(self,_list,sublist):
        #print("list:",_list)
        #print("sublist:",sublist)
        #print("is sublist? ",any(sublist == _list[i:i+len(sublist)] for i in range(len(_list))))
        return any(sublist == _list[i:i+len(sublist)] for i in range(len(_list)))

    def remove_until_different(self,buffer):
        to_be_removed = buffer[-1]
        while buffer:
            if buffer[-1] == to_be_removed:
                buffer = buffer[:-1]
        return buffer
    
    def simplify_buffer(self,buffer):
        simplified_buffer = []
        last = None
        for b in buffer:
            if b != last:
                simplified_buffer.append(b)
            last = b
        return simplified_buffer
            
    def get_open_paths_for_intersection(self,buffer,x,y,exact_sensor):
        #print("exact_sensor:",exact_sensor)
        left_buffer = []
        right_buffer = []
        middle_buffer = []
        
        for line in buffer:
            left_buffer.append(line[:2])
            right_buffer.append(line[5:])
            middle_buffer.append(line[2:5])
            
        #print("left_buffer:",left_buffer)
        #print("right_buffer:",right_buffer)
        #print("middle_buffer:",middle_buffer)
        
        l = self.simplify_buffer(left_buffer)
        r = self.simplify_buffer(right_buffer)
        m = self.simplify_buffer(middle_buffer)
        
        #print("l:",l)
        #print("r:",r)
        #print("m:",m)
        
        open_paths = []
        lb = self.get_left_buffer_paths(l,exact_sensor)
        rb = self.get_right_buffer_paths(r,exact_sensor)
        mb = self.get_middle_buffer_paths(m,exact_sensor)

        #print("lb", lb)
        #print("rb", rb)
        #print("mb", mb)
        open_paths.extend(lb)
        open_paths.extend(rb)
        open_paths.extend(mb)
         
        #remove repeated occurences
        open_paths = list(set(open_paths))

        #print("open_paths:",open_paths)
                   
        return open_paths

    def check_out_of_line(self, buffer):
        """
            this checks if the buffer is filled only with 0s
            useful for checking if the robot is outside the lines
            returns True (outside lines) or False (still in line)
        """
        buffer_ = self.simplify_buffer(buffer)
        if len(buffer_) == 1:
            if buffer_[0] == ["0","0","0","0","0","0","0"]:
                return True
        return False
    
    def get_distance(self,c1,c2):
        return round( ( ((c1[0]-c2[0])**2) + ((c1[1]-c2[1])**2) )**(1/2), 2)
    
    def dijkstra(self, graph, start, end):
        unvisited = set(graph.keys())
        distances = {node: float('inf') for node in unvisited}
        distances[start] = 0
        path = {}

        while unvisited:
            current_node = min(unvisited, key=lambda node: distances[node])

            if distances[current_node] == float('inf'):
                break

            unvisited.remove(current_node)

            for neighbor, weight in graph[current_node].items():
                potential = distances[current_node] + weight
                if potential < distances[neighbor]:
                    distances[neighbor] = potential
                    path[neighbor] = current_node
            
        if end not in path:
            return "No path found"

        shortest_path = []

        while end != start:
            shortest_path.append(end)
            end = path[end]
            
        shortest_path.append(start)
        weight = 0

        for tuple in shortest_path:
            if tuple in distances.keys():
                weight += distances[tuple]

        print("weight ", weight)
        
        return shortest_path[::-1], weight

    def minimun_weight_path(self, intersections, graph, x, y):
        print("\n\nNO MORE PATHS:")
        #print("intersections:",intersections)
        temp_length = 99999
        possible_path = None
        for end_node, intersection_set in intersections.items():
            if intersection_set:
                #print("set:",intersection_set)
                #print("end node:",end_node)
                #print("now: ", (round(x),round(y)))
                p, peso = self.dijkstra(graph,(round(x),round(y)),end_node)
                #print("path", p)
                #print("weight", peso)
                if p == "No path found":
                    #print(p)
                    return None
                if peso < temp_length:
                    temp_length =  peso
                    possible_path = p

        #print("possible path", possible_path)
        if possible_path is None:
            self.driveMotors(0.0,0.0)
            print("we should exit now")
            self.finish()
            exit()
        goal = possible_path[1]
        self.last_goal = goal
        
        return goal


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

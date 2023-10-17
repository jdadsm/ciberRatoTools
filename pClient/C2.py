from asyncio import sleep
import sys
from croblink import *
import math
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.h = 0.0085

        self.Kp = 1
        self.Ti = 1/self.h
        self.Td = 1*self.h

        self.e_m1 = 0
        self.e_m2 = 0
        self.u_m1 = 0

        self.max_u = 5

        #list of intersections
        self.vertices = []

        #ultimo goal 
        self.last_goal = [0,0]

        #outside?
        self.outside = False

        #ultima orientacao
        self.last_orientation = 0

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    
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
            
        return u

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

        pass

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
        #print("orientation: ",orientation)
        #print("x: " + str(x), "y: " + str(y))
        self.map[y_map][x_map] = orientation

    def initialize_map(self):
        lines = 21
        columns = 49

        self.map = [[0 for x in range(columns)] for y in range(lines)] #melhor fazer com variavel global...

        self.map[10][24] = "I"
    
    def getLinePos(self,line):
        posOverLine = 0
        nActiveSensors = 0
        
        for i in range(len(line)):
            if line[i] == "1":
                posOverLine += i-3
                nActiveSensors+=1
        
        if nActiveSensors != 0:
            posOverLine = 0.08*posOverLine/nActiveSensors
        else:
            return None
        
        return posOverLine
       
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
            return 180
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
        print("Sensor:",sensor)
        new_goal = None
        possible_new_goals = [[x+2, y],[x+2, y+2],[x, y+2],[x-2, y+2],[x-2, y],[x-2, y],[x+2, y-2],[x, y-2],[x-2, y-2]]
        sensor_values = [0,45,90,135,180,-180,-45,-90,-135]
        
        for i in range(len(possible_new_goals)):
            if sensor == sensor_values[i]:
                new_goal = possible_new_goals[i]
        
        if new_goal:
            print("New goal:",new_goal)
        else:
            print("Error in get_next_goal")
        return new_goal

    def convert_orientation_to_list_index(self, orientation):
        possible_orientations = [0,45,90,135,180,-180,-135,-90,-45]
        return_indexes = [0,1,2,3,4,4,5,6,7]
        
        for i in range(len(possible_orientations)):
            if possible_orientations[i] == orientation:
                return return_indexes[i]
    
    def get_middle_buffer_paths(self,buffer,exact_sensor):
        counter = 0
        for b in buffer:
            if b.count('1') >= len(b)-1:
                counter+=1
        if counter < len(buffer)-1:
            return []
        return [exact_sensor]
    
    def get_left_buffer_paths(self,buffer,exact_sensor):
        paths = []
        if buffer == [['1','1'],['0','1'],['0','0']] or buffer == [['1', '0'], ['1', '1'], ['0', '1'], ['0', '0']]:
            paths.append(45+exact_sensor)
        elif buffer == [['0','1']]:
            paths.append(135+exact_sensor)
        elif buffer == [['0', '0'], ['1', '1'], ['0', '0']]  or buffer == [['0','0'],['1','1']]:
            paths.append(90+exact_sensor)
        
        for i in range(len(paths)):
            if paths[i] > 180:
                paths[i] = paths[i] - 360
        
        return paths
    
    def get_right_buffer_paths(self,buffer,exact_sensor):
        paths = []
        if buffer == [['0', '1'], ['1', '1'], ['1', '0'], ['0', '0']] or buffer == [['0', '0'], ['0', '1'], ['0', '0'], ['1', '1']]:
            paths.append(-45+exact_sensor)
        elif buffer == [['0','1']]:
            paths.append(-135+exact_sensor)
        elif buffer == [['0','0'],['1','1'],['0','0']] or buffer == [['0','0'],['1','1']]:
            paths.append(-90+exact_sensor)
        
        for i in range(len(paths)):
            if paths[i] < -180:
                paths[i] = paths[i] + 360    
        return paths
    
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
        print("exact_sensor:",exact_sensor)
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
        
        print("l:",l)
        print("r:",r)
        print("m:",m)
        
        open_paths = []
        open_paths.extend(self.get_left_buffer_paths(l,exact_sensor))
        open_paths.extend(self.get_right_buffer_paths(r,exact_sensor))
        open_paths.extend(self.get_middle_buffer_paths(m,exact_sensor))
         
        print("open_paths:",open_paths)
                   
        return open_paths

    def run(self):

        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.initialize_map()

        #posicao inical do carro -> I
        self.readSensors()
        pos_inicial_real = [self.measures.x, self.measures.y]

        ZEROS = ["0","0","0","0","0","0","0"]
        BUFFER_DEFAULT = ["0","0","1","1","1","0","0"]      
        BUFFER_SIZE = 7
        buffer = []
        
        for i in range(BUFFER_SIZE):
            buffer.append(BUFFER_DEFAULT)
            
        velSetPoint = 0.08
        
        sensor = self.measures.compass
        exact_sensor = self.get_exact_sensor_value(sensor)
        #print(exact_sensor)
        
        last_sensor = 0
        
        goal = self.get_next_goal(exact_sensor, 0, 0)
        
        #keys = (x,y) values = open_paths
        intersections = {}
            
        while True:
            self.readSensors()
            
            line = self.measures.lineSensor
            sensor = self.measures.compass
            #print("Sensor:",sensor)
            
            if abs(sensor) > 170:
                if (last_sensor<0 and sensor >0) or (last_sensor>0 and sensor<0):
                    sensor = last_sensor

            x = self.measures.x - pos_inicial_real[0]
            y = self.measures.y - pos_inicial_real[1]

            orientation_string = self.get_orientation_string(sensor)
            #print("orientation:",orientation_string)
            
            if abs(x-goal[0]) <= 0.05 and abs(y-goal[1]) <= 0.05:
                #print("Exact sensor value:", self.get_exact_sensor_value(sensor))
                if (round(x),round(y)) not in intersections:
                    intersections[(round(x),round(y))] = set()
                
                    open_paths = self.get_open_paths_for_intersection(buffer,x,y,self.get_exact_sensor_value(sensor))
                    
                    for path in open_paths:
                        if path not in intersections[(round(x),round(y))]:
                            intersections[(round(x),round(y))].add(path)
                    
                    print("intersections:",intersections)
                    
                    if intersections[(round(x),round(y))]:  # if there are still open paths in this intersection, choose one and explore it
                        self.last_goal = goal
                        goal = self.get_next_goal(intersections[(round(x),round(y))].pop(),goal[0],goal[1]) 
                    else:                                   # if not go back
                        # MUDAR QUANDO CONSEGUIRMOS DIFERENCIAR ENTRE TER UMA INTERSEÇÃO TOTALMENTE EXPLORADA E UM DEADEND 
                        self.last_goal,goal = goal,self.last_goal

            elif abs(x-goal[0]) <= 0.3 and abs(y-goal[1]) <= 0.3:           #esta se a aproximar, nao vale a pena usar PID que oscila demasiado
                #print("start slowing down")
                self.driveMotors(0.04, 0.04)
            else:
                print(sensor)
                    

            alpha = math.atan2(goal[1]-y,goal[0]-x)*180/math.pi
            expr = (sensor - alpha)
            #print("expr", expr)
            
            u = self.PID(0,expr*0.02)*0.5

            lPow = velSetPoint - u
            rPow = velSetPoint + u          

            self.driveMotors(lPow,rPow)

            buffer = buffer[0:BUFFER_SIZE]
            buffer = [line] + buffer
            
            last_sensor = sensor
            
            #aqui vemos ja passamos por certos locais
            if (orientation_string != "-" and orientation_string != "|"):
                if(round(x)%2 == 1 and round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation_string)
                    self.print_map_to_file()
            else:
                if (round(x)%2 == 1 or round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation_string)
                    self.print_map_to_file()

            
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

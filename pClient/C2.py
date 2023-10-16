
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

        print("em1", self.e_m1)
        
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

    
    def calculate_ones(self, line):
        """ 
        This function receives the sensor line array and returns an integer value 
            - positive value -> right array is predominant -> the robot must turn right
            - negative value -> left array is predominant -> the robot must turn left
            - zero -> right and left arrays are balanced -> the robot keeps going forward
        """
        ones_left = line[0:4].count("1")
        ones_right = line[3:7].count("1")
        #print(line)
        #print(ones_right-ones_left)
        return ones_left-ones_right


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
        pass

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
        
        for i in range(7):
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

    def check_buffer_orientation(self, buffer, right):
        # intersecoes retas
        if buffer.count(['1', '1']) == len(buffer) and len(buffer)>=2:
            if right:
                return -90
            else:
                return 90
            
        print(len(buffer))
        print("")
        if len(buffer) == 1:
            if buffer[0] == ['1','1']:
                if right:
                    return -90
                else:
                    return 90
            return None

        # intersecoes diagonais
        if buffer[0] == ['0', '1']:
            if buffer[1] == ['1', '1']:
                if right:
                    return -45
                else:
                    return 135

            if buffer[1] == buffer[0]:
                return self.check_buffer_orientation(buffer[1:], right)

        elif buffer[0] == ['1', '0']:
            if buffer[1] == ['1', '1']:
                if right:
                    return -135
                else:
                    return 45
            if buffer[1] == buffer[0]:
                return self.check_buffer_orientation(buffer[1:], right)
        
        elif buffer[0] == ['1', '1']: 
            if buffer[1] == ['1', '0']:
                if right:
                    return -45
                else:
                    return 135

            if buffer[1] == ['0', '1']:
                if right:
                    return -135
                else:
                    return 45 

            if buffer[1] == buffer[0]:
                return self.check_buffer_orientation(buffer[1:], right)
            pass

        return None
    
    def get_next_goal(self, sensor, x, y):
        """
        This function takes the position of the robot the sensor value of the line where he is at the moment and return the position to have as a next goal
        """
        if sensor == 0:
            return [x+2, y]
        if sensor == 45:
            return [x+2, y+2]
        if sensor == 90:
            return [x, y+2]
        if sensor == 135:
            return [x-2, y+2]
        if sensor == -180 or sensor == 180:
            return [x-2, y]
        if sensor == -45:
            return [x+2, y-2]
        if sensor == -90:
            return [x, y-2]
        if sensor == -135:
            return [x-2, y-2]
        return None

    def convert_orientation_to_list_index(self, ori):
        if ori == 0:
            return 0
        if ori == 45:
            return 1
        if ori == 90:
            return 2
        if ori == 135:
            return 3
        if ori == 180 or ori == -180:
            return 4
        if ori == -135:
            return 5
        if ori == -90:
            return 6
        if ori == -45:
            return 7

    #this will return the current orientation
    def get_intersection_type(self, buffertemp, ori, x, y, current):
        
        print("original buffer", buffertemp)
        
        #buffertemp [0:3] e muito a frente do ponto onde a verificacao é feita
        buffert = [line[3] for line in buffertemp[0:4]].count('1')
        
        #print("buffer: " + str(buffer))
        print("x", x)
        print("y", y)

        #buffertemp [3:8] e muito a frente do ponto onde a verificacao é feita
        buffer = [line for line in buffertemp[2:8] if line!=['0', '0', '0', '0', '0', '0', '0']]
        print("buffer: " + str(buffer))
        buffleft = [line[0:2] for line in buffer if line[0:2]!=['0', '0']]
        buffright = [line[5:7] for line in buffer if line[5:7]!=['0', '0']]

        vertice = None

        if self.vertices != []:
            for tvertice in self.vertices:
                if tvertice.check_xy(x, y):      #there is already a vertice with those coordinates
                    vertice = tvertice

        if vertice != None:                     # no need to execute further code, data is already inside the lists
            return current

        vertice = Intersection(x, y)

        if buffert == len(buffertemp[0:4]):      #segue em frente
            vertice.possible_intersections.add(ori)

        if buffertemp[0][2:5] == ['0', '0', '0'] and (ori in list): #afinal nao ha caminho em frente
            vertice.possible_intersections.remove(ori)

        if len(buffleft) >= 1:
            var = self.check_buffer_orientation(buffleft, False)
            if  var != None:
                true_int1 = self.get_exact_sensor_value(int(var)+ori)
                vertice.possible_intersections.add(true_int1)

        if len(buffright) >= 1:
            var = self.check_buffer_orientation(buffright, True)
            if  var != None:
                true_int2 = self.get_exact_sensor_value(int(var)+ori)
                vertice.possible_intersections.add(true_int2)

        if len(list) == 1:
            return list[0]
        
        elif list == []:
            print("dead end")
            return self.get_exact_sensor_value(180-ori) #go backwards

        print("intersections", list)

        # by now, the list should be filled with all intersections in a given point 
        vertice.add_all_intersections(list[1:])
        
        return list[0]
        
    def check_if_not_inline(self, buffer, outside):
        """
            assim que o buffer detetar tudo a zeros, voltar para o ultimo nó (que deverá ser o mais proximo)
            e ver as intersecoes de novo 

            devolve:
                - orientacao a ir
                - ultimo nó
        """
        if outside:
            buffer_ = [line for line in buffer if line!=['0','0','0','0','0','0','0']]
            if len(buffer_) == 0:
                return self.get_exact_sensor_value(self.last_orientation-180), self.last_goal
        pass

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

        #this should always be 0 at the beginning
        sensor = self.measures.compass
        exact_sensor = self.get_exact_sensor_value(sensor)
        #print(exact_sensor)

        goal = self.get_next_goal(exact_sensor, 0, 0)
        print(goal)
            
        while True:
            self.readSensors()
            
            line = self.measures.lineSensor
            sensor = self.measures.compass #compass - em graus 
            #print("Sensor:",sensor)

            """cada quadrado tem 2 unidades de medida -> verificar qual a orientacao no meio de cada quadrado"""
            x = self.measures.x - pos_inicial_real[0]
            y = self.measures.y - pos_inicial_real[1]

            #ajustar o robo nas linhas
            orientation_string = self.get_orientation_string(sensor)
            #print("orientation:",orientation_string)
            
            #posOverLine = self.getLinePos(line)
            #print("posOverLine:",posOverLine)
            #print(posOverLine)
            
            if abs(x-goal[0]) <= 0.1 and abs(y-goal[1]) <= 0.1:
                buffer_ = buffer[0:8]
                #print("Exact sensor value:", self.get_exact_sensor_value(sensor))
                exact_sensor = self.get_intersection_type(buffer_, self.get_exact_sensor_value(sensor), round(x), round(y), exact_sensor)
                
                self.last_goal = goal
                
                goal = self.get_next_goal(exact_sensor,goal[0],goal[1]) 
                continue

            elif abs(x-goal[0]) <= 0.3 and abs(y-goal[1]) <= 0.3:           #esta se a aproximar, nao vale a pena usar PID que oscila demasiado
                print("start slowing down")
                self.driveMotors(0.04, 0.04)

            else:
                alpha = math.atan2(goal[1]-y,goal[0]-x)*180/math.pi

                if self.get_exact_sensor_value(sensor)==180:    #existe muita disparidade
                    print("here")
                    if sensor<0:                                #na parte negativa
                        expr = (sensor - alpha)
                        pass
                    else:                                       #na parte positiva
                        pass


                expr = (sensor - alpha)
                print("expr", expr)
                
                u = self.PID(0,expr*0.02)*0.5

                lPow = velSetPoint - u
                rPow = velSetPoint + u          

                self.driveMotors(lPow,rPow)
            
            #resolver disparacao entre orientacao real ser -180 e variar muito para 180

            #estou fora das linhas? É melhor ver como esta a intersecao


            print("goal", goal)

            buffer = buffer[0:BUFFER_SIZE]
            buffer = [line] + buffer
            
            #aqui vemos ja passamos por certos locais
            if (orientation_string != "-" and orientation_string != "|"):
                if(round(x)%2 == 1 and round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation_string)
                    self.print_map_to_file()
            else:
                if (round(x)%2 == 1 or round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation_string)
                    self.print_map_to_file()

            

class Intersection():
    def __init__(self, x, y):
        self.x = x
        self.y = y

        #self.delete_later = []

        """
            list of known neighbours    
        """
        self.neighbours = set()

        """
            list of possible paths  
        """
        self.possible_intersections = set()

        """
            list of already visited paths    
        """
        self.visited_intersections = set()

    def __repr__(self) -> str:
        return  "{x: " + str(self.x) + "; " +\
                "y: " + str(self.y) + "; " +\
                "visited?: " + str(self.delete_later) + "; " +\
                "intersections: " + str(self.intersections) + "}"

    def is_visited(self):
        return self.visited

    def add_neighbour(self, intersection):
        self.neighbours.add(intersection)

    def check_xy(self, x, y):
        if self.x == x and self.y == y:
            return self
        else:
            return None

    def add_visited_orientation(self, orientation):
        self.visited_intersections.add(orientation)

    def add_possible_orientation(self, orientation):
        self.possible_intersections.add(orientation)

    def get_visited_orientations(self):
        return self.visited_intersections

    def get_possible_orientations(self):
        return self.possible_intersections
        


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

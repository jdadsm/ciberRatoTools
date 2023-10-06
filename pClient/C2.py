
from asyncio import sleep
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.h = 0.050

        self.Kp = 1
        self.Ti = 1/self.h
        self.Td = 1*self.h

        self.e_m1 = 0
        self.e_m2 = 0
        self.u_m1 = 0

        self.max_u = 10

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    """ 
    This function receives the sensor line array and returns an integer value 
        - positive value -> right array is predominant -> the robot must turn right
        - negative value -> left array is predominant -> the robot must turn left
        - zero -> right and left arrays are balanced -> the robot keeps going forward
    """
    def calculate_ones(self, line):
        
        ones_left = line[0:4].count("1")
        ones_right = line[3:7].count("1")
        #print(line)
        #print(ones_right-ones_left)
        return ones_left-ones_right

    def PID_controller(self):

        pass

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

    """this function takes an orientation value and converts it to a orientation:
        - -> horizontal line                    -
        | -> vertical line                      | 
        / -> oblique line (tan is positive)     /
        \ -> oblique line (tan is negative)     \
        0 -> empty (should not be returned under any circunstance)
    """
    def check_direction(self, ori):

        if ori < -180 or ori > 180:
            print("out of bounds")
            return 0

        if (ori <= 22.5 and ori >= -22.5) or (ori >= 157.5 and ori <= 180) or (ori>=180 or ori<=-157.5):
            return '-'
        elif (ori <= -67.5 and ori >= -112.5) or (ori >= 67.5 and ori <= 112.5):
            return '|'
        elif (ori < -112.5 and ori > -157.5) or (ori > 22.5 and ori < 67.5):
            return '/'
        elif (ori < -22.5 and ori > -67.5) or (ori > 112.5 and ori < 157.5):
            return chr(92) # \ character
        else:
            print("Orientation interval was not found...")
            return 0
        
    """put an oritentation in a certain position in the map"""
    def put_in_map(self, x, y, orientation):
        x_map = 24 + x
        y_map = 10 + y
        #print("orientation: ",orientation)
        #print("x: " + str(x), "y: " + str(y))
        self.map[y_map][x_map] = orientation

    """
        initialize map with the same characterists as the map that should be printed
        I is in the middle of the map -> position [24, 11] (should need to verify this)!!
    """
    def initialize_map(self):
        lines = 21
        columns = 49

        self.map = [[0 for x in range(columns)] for y in range(lines)] #melhor fazer com variavel global...

        self.map[10][24] = "I"
        #print(self.map)
    
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
    
    def get_objective(self, ori):
        if ori < -180 or ori > 180:
            print("out of bounds")
            return 0
        
        if (ori <= 22.5 and ori>=0) or (ori<=0 and ori>=-22.5):
            return 1
        elif (ori >= 22.5 and ori <= 67.5):
            return 2
        elif (ori >= 67.5 and ori <= 112.5):
            return 3
        elif (ori >= 112.5 and ori <= 157.5):
            return 4
        elif (ori>=157.5 and ori<= 180) or (ori<=-157.5 and ori>=-180):
            return 5
        elif (ori>=-157.5 and ori<=-112.5):
            return 6
        elif (ori>=-112.5 and ori<=-67.5):
            return 7
        elif (ori>=67.5 and ori<=-22.5):
            return 8
        else:
            print("Orientation interval was not found...")
            return 0

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
        BUFFER_SIZE = 4
        buffer = []
        
        for i in range(BUFFER_SIZE):
            buffer.append(BUFFER_DEFAULT)
            
        velSetPoint = 0.15
            
        while True:
            self.readSensors()
            
            line = self.measures.lineSensor
            sensor = self.measures.compass #compasso - em graus 
            #print("Orientation:",sensor)

            """cada quadrado tem 2 unidades de medida -> verificar qual a orientacao no meio de cada quadrado"""
            x = self.measures.x - pos_inicial_real[0]
            y = self.measures.y - pos_inicial_real[1]

            #print(str(self.check_direction(sensor)))
            #print("x: " + str(x) + ", y: " + str(y))
            orientation = self.check_direction(sensor)
            
            #obj = 
            
            posOverLine = self.getLinePos(line)
            
            print("posOverLine:",posOverLine)
            
            buffer = buffer[0:BUFFER_SIZE]
            buffer = [line] + buffer
            
            lPow = velSetPoint - self.PID(0,(sensor)/180)
            rPow = velSetPoint + self.PID(0,sensor/180)
            
            self.driveMotors(lPow,rPow)
            
            #print("orientation:",orientation)
            print(sensor)

            if x%2 == 0 and y%2 == 0:
                print(buffer)
                self.driveMotors(0.04,0.04)
                
            if line == ZEROS:
                print(buffer)
                self.driveMotors(0.0,0.0)
            
            
            
            if (orientation != "-" and orientation != "|"):
                if(round(x)%2 == 1 and round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation)
                    self.print_map_to_file()
            else:
                if (round(x)%2 == 1 or round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation)
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

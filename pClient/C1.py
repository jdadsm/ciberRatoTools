
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
        pass
        

    def run(self):

        Kp = 10;
        BANG_VALUE = 0.2;

        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.initialize_map()

        state = 'stop'
        stopped_state = 'run'

        #posicao inical do carro -> I
        self.readSensors()
        pos_inicial_real = [self.measures.x, self.measures.y]

        ZEROS = ["0","0","0","0","0","0","0"]  
        BUFFER_DEFAULT = ["0","0","1","1","1","0","0"]      
        BUFFER_SIZE = 4
        buffer = []
        for i in range(BUFFER_SIZE):
            buffer.append(BUFFER_DEFAULT)
            
        while True:
            self.readSensors()
            
            line = self.measures.lineSensor
            sensor = self.measures.compass #compasso - em graus 

            """cada quadrado tem 2 unidades de medida -> verificar qual a orientacao no meio de cada quadrado"""
            x = self.measures.x - pos_inicial_real[0]
            y = self.measures.y - pos_inicial_real[1]

            #print(str(self.check_direction(sensor)))
            #print("x: " + str(x) + ", y: " + str(y))
            orientation = self.check_direction(sensor)
            
            """             #ISTO É SÓ PARA SABER QUANDO HÁ NOISE POR AGORA
            ones = 0
            zeros = 0
            for i in line:
                if i == "1":
                    ones+=1
                else:
                    zeros+=1
            if ones != 3:
                if abs(ones-3) > 1:
                    line = buffer """
                #print("\nNoise\nOnes-"+str(ones)+"\nZeros-"+str(zeros))
            
            #print("Line:",line)
            
            #CASO PARA CURVAS APERTADAS
            if line == ZEROS:
                val = 0
                for entry in buffer:
                    temp = self.calculate_ones(entry)
                    print(temp)
                    val+= temp 
                    #val+= self.calculate_ones(entry)
                    
                print(buffer)
                print("CURVAS:",val)
                if val > 0: 
                    self.driveMotors(0.45,-0.45)
                elif val < 0:
                    self.driveMotors(-0.45,0.45)
                else:
                    self.driveMotors(0.0,0.0)
                continue
            
            buffer = buffer[0:BUFFER_SIZE]
            buffer = [line] + buffer
            
            error = self.calculate_ones(line)

            r = 0.05*error
            #l = (3 - abs(valor))*0.05 #DEPOIS MUDAR!
            if (error==0):
                self.driveMotors(0.20,0.20)
            else:
                self.driveMotors(-r, r)

            
            if (orientation != "-" and orientation != "|"):
                if(round(x)%2 == 1 and round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation)
                    self.print_map_to_file()
            else:
                if (round(x)%2 == 1 or round(y)%2 == 1):
                    self.put_in_map(round(x), round(y), orientation)
                    self.print_map_to_file()

        
            #print("r: " + str(r) + " \n l: " + str(l))

            """ if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander() """

    """put an oritentation in a certain position in the map"""
    def put_in_map(self, x, y, orientation):
        x_map = 24 + x
        y_map = 10 + y
        print("orientation: ",orientation)
        print("x: " + str(x), "y: " + str(y))
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
        return 

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)



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

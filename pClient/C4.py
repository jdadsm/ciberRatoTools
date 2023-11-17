
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
        self.h = 0.05

        self.Kp = 0.7
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

        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.readSensors()

        BUFFER_DEFAULT = ["0","0","1","1","1","0","0"]      
        BUFFER_SIZE = 5
        buffer = []
        for i in range(BUFFER_SIZE):
            buffer.append(BUFFER_DEFAULT)
            
        BUFFER_POS = [[0.0,0.0],[0.0,0.0]]
        BUFFER_COMPASS = [0,0]
            
        velSetPoint = 0.15
        
        pos_inicial_real = [self.measures.x, self.measures.y]
            
        while True:
            self.readSensors()
            
            line = self.measures.lineSensor
            
            sensor = self.measures.compass #compass - em graus 

            x = self.measures.x - pos_inicial_real[0]
            y = self.measures.y - pos_inicial_real[1]
            
            gps_pos = [x,y]
            
            print("Bússola:",sensor)
            print("Posição GPS:",gps_pos)
            
            #print("Line:",line)
            original_line = line
            for i in range(len(line)):
                if line[i] != buffer[0][i]:
                    line[i] == "1"

            posOverLine = self.getLinePos(line)
            
            #print("posOverLine:",posOverLine)
            
            if posOverLine == None:
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
                    self.driveMotors(-0.15,0.15)
                else:
                    self.driveMotors(0.15,-0.15)
                continue
            
            PID = self.PID(0, posOverLine)
            lPow = velSetPoint - PID
            rPow = velSetPoint + PID
            
            self.driveMotors(lPow,rPow)
            
            lin = (lPow + rPow)/2
            print("lin:",lin)
            print("in cosseno:",BUFFER_COMPASS[len(BUFFER_COMPASS)-1]*math.pi/180)
            xt = BUFFER_POS[len(BUFFER_POS)-1][0] + lin*math.cos(BUFFER_COMPASS[len(BUFFER_COMPASS)-1]*math.pi/180)
            yt = BUFFER_POS[len(BUFFER_POS)-1][1] + lin*math.cos(BUFFER_COMPASS[len(BUFFER_COMPASS)-1]*math.pi/180)
            
            print("Predict Posição:",[xt,yt])
            
            rot = (lPow + rPow)/0.5
            
            thetat = BUFFER_COMPASS[len(BUFFER_COMPASS)-1] + rot
            
            print("Predict Sensor:",thetat)
            
            BUFFER_POS = BUFFER_POS[1:]
            BUFFER_POS.append([xt,yt])
            
            BUFFER_COMPASS = BUFFER_COMPASS[1:]
            BUFFER_COMPASS.append(thetat)
            
            buffer = buffer[0:BUFFER_SIZE]
            buffer = [original_line] + buffer
            


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

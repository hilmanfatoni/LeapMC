################################################################################ 
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import os, sys, _thread, time, inspect, socket, threading

src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
arch_dir = '../win64'

sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))

import Leap
from argparse import ArgumentParser
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import pandas as pd
import time
import math
import csv

import serial.tools.list_ports

def getHex(dataint):
    dataHex = str()
    if dataint<16:
        dataHex="0"+format(dataint,'x')
    else: 
        dataHex=format(dataint,'x')
    return dataHex


ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

serialInst.baudrate = 115200
serialInst.port = "COM3"
serialInst.open()


myList = []
myNp =np.zeros(shape=(4,3))
M0 = np.zeros(shape=(1,3))

hd_prop = np.zeros(shape=(3,3)) #armdir; wrist position; elbow position
fr_thumb =np.zeros(shape=(5,3)) #metacarpal prev joint; metacarpal next joint, proximal next joint; intermediate next joint; distal next joint
fr_index =np.zeros(shape=(5,3))
fr_middle =np.zeros(shape=(5,3))
fr_ring = np.zeros(shape=(5,3))
fr_angle = np.zeros(shape=(5,1))
fr_angle_deg = np.zeros(shape=(5,1)) #0,1 thumb; 2 index; 3 middle; 4 ring&little
fr_cmd = np.zeros(shape=(5,1)) #0,1 thumb; 2 index; 3 middle; 4 ring&little

PrensiliaData = np.zeros(shape=(5,1)) #ThumbAb; Thumb; Index; Middle; Ring_Little
PrensiliaData_deg = np.zeros(shape=(5,1))

#plt.ioff()
'''
# Plot X,Y,Z
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(0, 0)
ax.scatter(0, 0, 0, c='red')
'''

def getDataValue(string):
    cleanString = str()
    string = str(string)
    for e in string:
        if e != '(' and e != ')':
            cleanString += e
    cleanList = [float(i) for i in cleanString.split(',')]
    return  cleanList

def VectRes(A : list, B : list ):
    return B-A

def VectDot(A : list, B : list ):
    result=A[0]*B[0]+A[1]*B[1]+A[2]*B[2]
    return result

def VectMag(A : list, B : list ):
    result=math.sqrt((B[0]-A[0])*(B[0]-A[0])+(B[1]-A[1])*(B[1]-A[1])+(B[2]-A[2])*(B[2]-A[2]))
    return result

def DegCalc(A : list, B : list, C : list ):
    result=math.acos((VectDot(VectRes(A, B), VectRes(B, C)) /( VectMag(A, B) * VectMag(B, C) )))
    return result

def DegCalc4point(A : list, B : list, C : list, D : list ):
    result=math.acos((VectDot(VectRes(A, B), VectRes(C, D)) /( VectMag(A, B) * VectMag(C, D) )))
    return result

def ConvInttoDeg(Data : int):
    result = Data * 0.35294
    return result

def SendCmd(AngleData : float):
    if math.isnan(AngleData):
        result = 0
    else:
        if AngleData<0 or AngleData > 1.5:
            result = 0
        else: 
            result = AngleData*183.45
    return (int(result))

def SendCmdThumb(AngleData : float):
    if math.isnan(AngleData):
        result = 0
    else:
        if AngleData<0 or AngleData > 1.5:
            result = 0
        else: 
            #result = -209.016*AngleData+255 #70degree         
            #result = -324.84*AngleData+255 #45degree  
            result = -292.43*AngleData+255 #50degree                     
    return (int(result))

def SendCmdThumbAb(AngleData : float):
    if math.isnan(AngleData):
        result = 0
    else:
        if AngleData<0 or AngleData > 1.5:
            result = 0
        else:
            #result = -162.42*AngleData+255 #90degree          
            #result = -209.016*AngleData+255 #70degree         
            result = -324.84*AngleData+255 #45degree  
            #result = -292.43*AngleData+255 #50degree                  
    return (int(result))

class SampleListener(Leap.Listener):

    def on_connect(self, controller):
        print ("Connected")

    def on_frame(self, controller):
        frame = controller.frame()

        print ("Timestamp: %d, FramePerSec: %d, Frame id: %d, Hands: %d, Fingers: %d" % (
        frame.timestamp, frame.current_frames_per_second, frame.id, len(frame.hands), len(frame.fingers)))

        try:            
            ##For Trial
            '''                        
            M0 = getDataValue(frame.hands[0].fingers[1].bone(0).prev_joint)
            myNp[0]=getDataValue(frame.hands[0].fingers[1].bone(0).next_joint)
            myNp[1]=getDataValue(frame.hands[0].fingers[1].bone(1).next_joint)
            myNp[2]=getDataValue(frame.hands[0].fingers[1].bone(2).next_joint)
            myNp[3]=getDataValue(frame.hands[0].fingers[1].bone(3).next_joint)
            '''
            
            #Get data from Prensilia
            #data from Motor 0
            dataTemp="4500"
            serialInst.write(bytes.fromhex(dataTemp))
            dataReceived=serialInst.read()
            PrensiliaData[0]=ord(dataReceived)
            #data from Motor 1
            dataTemp="4501"
            serialInst.write(bytes.fromhex(dataTemp))
            dataReceived=serialInst.read()
            PrensiliaData[1]=ord(dataReceived)
            #data from Motor 2
            dataTemp="4502"
            serialInst.write(bytes.fromhex(dataTemp))
            dataReceived=serialInst.read()
            PrensiliaData[2]=ord(dataReceived)
            #data from Motor 3
            dataTemp="4503"
            serialInst.write(bytes.fromhex(dataTemp))
            dataReceived=serialInst.read()
            PrensiliaData[3]=ord(dataReceived)
            #data from Motor 4
            dataTemp="4504"
            serialInst.write(bytes.fromhex(dataTemp))
            dataReceived=serialInst.read()
            PrensiliaData[4]=ord(dataReceived)

            PrensiliaData_deg=ConvInttoDeg(PrensiliaData)
            
            #print(PrensiliaData_deg)
            
            fFile.write("%d,%d,%d," % (frame.timestamp, frame.current_frames_per_second, frame.id))
            fFile.write("%f,%f,%f,%f,%f," % (PrensiliaData_deg[0],PrensiliaData_deg[1],PrensiliaData_deg[2],PrensiliaData_deg[3],PrensiliaData_deg[4]))
            #fFile.write("%f,%f,%f,%f,%f," % (PrensiliaData[0],PrensiliaData[1],PrensiliaData[2],PrensiliaData[3],PrensiliaData[4]))

            #convert and get data from LMC
            hd_prop[0] = getDataValue(frame.hands[0].arm.direction)
            hd_prop[1] = getDataValue(frame.hands[0].arm.wrist_position)
            hd_prop[2] = getDataValue(frame.hands[0].arm.elbow_position)

            fr_thumb[0] = getDataValue(frame.hands[0].fingers[0].bone(0).prev_joint)
            fr_thumb[1] = getDataValue(frame.hands[0].fingers[0].bone(0).next_joint)
            fr_thumb[2] = getDataValue(frame.hands[0].fingers[0].bone(1).next_joint)
            fr_thumb[3] = getDataValue(frame.hands[0].fingers[0].bone(2).next_joint)
            fr_thumb[4] = getDataValue(frame.hands[0].fingers[0].bone(3).next_joint)

            fr_index[0] = getDataValue(frame.hands[0].fingers[1].bone(0).prev_joint)
            fr_index[1] = getDataValue(frame.hands[0].fingers[1].bone(0).next_joint)
            fr_index[2] = getDataValue(frame.hands[0].fingers[1].bone(1).next_joint)
            fr_index[3] = getDataValue(frame.hands[0].fingers[1].bone(2).next_joint)
            fr_index[4] = getDataValue(frame.hands[0].fingers[1].bone(3).next_joint)          
            
            fr_middle[0] = getDataValue(frame.hands[0].fingers[2].bone(0).prev_joint)
            fr_middle[1] = getDataValue(frame.hands[0].fingers[2].bone(0).next_joint)
            fr_middle[2] = getDataValue(frame.hands[0].fingers[2].bone(1).next_joint)
            fr_middle[3] = getDataValue(frame.hands[0].fingers[2].bone(2).next_joint)
            fr_middle[4] = getDataValue(frame.hands[0].fingers[2].bone(3).next_joint)

            fr_ring[0] = getDataValue(frame.hands[0].fingers[3].bone(0).prev_joint)
            fr_ring[1] = getDataValue(frame.hands[0].fingers[3].bone(0).next_joint)
            fr_ring[2] = getDataValue(frame.hands[0].fingers[3].bone(1).next_joint)
            fr_ring[3] = getDataValue(frame.hands[0].fingers[3].bone(2).next_joint)
            fr_ring[4] = getDataValue(frame.hands[0].fingers[3].bone(3).next_joint)

            #angle calculation
            fr_angle [0] = DegCalc4point(hd_prop[1], fr_ring[0], hd_prop[1], fr_thumb[0])
            fr_angle [1] = DegCalc4point(fr_thumb[1], fr_thumb[2], fr_index[0], fr_index[1])
            fr_angle [2] = DegCalc(fr_index[0], fr_index[1], fr_index[2])
            fr_angle [3] = DegCalc(fr_middle[0], fr_middle[1], fr_middle[2])
            fr_angle [4] = DegCalc(fr_ring[0], fr_ring[1], fr_ring[2])

            #fFile.write("%f,%f,%f,%f,%f" % (fr_angle[0],fr_angle[1],fr_angle[2],fr_angle[3],fr_angle[4]))
            fr_angle_deg [0]=math.degrees(fr_angle[0])
            fr_angle_deg [1]=math.degrees(fr_angle[1])
            fr_angle_deg [2]=math.degrees(fr_angle[2])
            fr_angle_deg [3]=math.degrees(fr_angle[3])
            fr_angle_deg [4]=math.degrees(fr_angle[4])
            fFile.write("%f,%f,%f,%f,%f" % (fr_angle_deg[0],fr_angle_deg[1],fr_angle_deg[2],fr_angle_deg[3],fr_angle_deg[4]))
            fFile.write("\n")

            ##For Plot
            '''
            ax.clear()
            X, Y, Z = myNp[:,0], myNp[:,1], myNp[:,2]
            ax.scatter(X, Y, Z, c='red')
            for i in range (1,4):
                ax.plot([X[i-1],X[i]], [Y[i-1],Y[i]], [Z[i-1],Z[i]], c='red', label='parametric curve')

            plt.draw()
            '''

            ##For Trial
            '''
            AngleAzu = DegCalc(M0, myNp[0], myNp[1]) #degree between metacarpal and proximal phalanges
            #AngleAzu = DegCalc(myNp[0], myNp[1], myNp[2])
            if math.isnan(AngleAzu):
                RangeAzu = 0
            else:
                if AngleAzu<0 or AngleAzu > 1.5:
                    RangeAzu = 0
                else: 
                    RangeAzu = AngleAzu*250/1.40

            RangeCommand = int (RangeAzu)
            print(RangeCommand)
            '''

            #SendCommand to Motor 0 / thumb ab/ad
            dataTemp="4400"+getHex(SendCmdThumbAb(fr_angle[0]))           
            serialInst.write(bytes.fromhex(dataTemp))
            #SendCommand to Motor 1 / thumb flex
            dataTemp="4401"+getHex(SendCmdThumb(fr_angle[1]))
            serialInst.write(bytes.fromhex(dataTemp))
            #SendCommand to Motor 2 / index flex
            dataTemp="4402"+getHex(SendCmd(fr_angle[2]))
            serialInst.write(bytes.fromhex(dataTemp))
            #SendCommand to Motor 3 / middle  flex
            dataTemp="4403"+getHex(SendCmd(fr_angle[3]))
            serialInst.write(bytes.fromhex(dataTemp))
            #SendCommand to Motor 4 / ring and little finger flex
            dataTemp="4404"+getHex(SendCmd(fr_angle[4]))
            serialInst.write(bytes.fromhex(dataTemp))
                        
            time.sleep(0.2)
                        
        except SystemError:
            pass

def main():
    #Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    #Have the sample listener receive events from the controller
    controller.add_listener(listener)

    #keep process running in the background
    if not controller.is_policy_set(Leap.Controller.POLICY_BACKGROUND_FRAMES):
        controller.set_policy(Leap.Controller.POLICY_BACKGROUND_FRAMES)

    #Keep this process running until Enter is pressed
    print ("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        #Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    #open file for joint angle
    if not os.path.isdir("rawData"):
        os.mkdir("rawData")

    countFile = 1
    while os.path.exists("rawData/Data%s.txt" % countFile):
        countFile += 1

    fFile = open("rawData/Data%s.txt" % countFile, "w")
    fFile.write("Timestamp,FramePerSec,FrameID,")
    fFile.write("Thumb_Ab,Thumb,Index,Middle,Ring_Little,")
    fFile.write("LMC_Thumb Ab,LMC_Thumb,Index_Meta-Proximal,Middle_Meta-Proximal,Ring_Little_Meta-Proximal\n")
       
    main() 

    fFile.close()
    
    '''
    print(myNp)
    X, Y, Z = myNp[:,0], myNp[:,1], myNp[:,2]
    ax.scatter(X, Y, Z, c='red')
    for i in range (1,4):
        ax.plot([X[i-1],X[i]], [Y[i-1],Y[i]], [Z[i-1],Z[i]], c='red', label='parametric curve')

    plt.draw()
    plt.pause(20)
    '''
    
    with open("rawData/Data%s.txt" % countFile, "r") as file:
        # Create a CSV reader object
        reader = csv.reader(file)
        # Skip the header row
        next(reader)
        # Read the data into a list
        dataText = list(reader)
    
    dataTextFloat = [[float(x) for x in sublist] for sublist in dataText]
    # Print the data
    #print(dataText)
    #print(dataTextFloat)    

    num_rows = len(dataTextFloat)
    ite = np.arange(0,num_rows,1)
    ite = ite.reshape(-1,1)

    listLMC =[]   
    for row in dataTextFloat:
        new_row = row[3:8]
        listLMC.append(new_row)

    listPrensilia =[]
    for row in dataTextFloat:
        new_row = row[8:13]
        listPrensilia.append(new_row)    

    dataLMC = np.array(listLMC)
    dataPrensilia = np.array(listPrensilia)

    #print(dataLMC)
    #print(dataPrensilia)
    #print(dataPrensilia)

    dataLMC=np.concatenate((ite,dataLMC),axis=1)
    dataPrensilia=np.concatenate((ite,dataPrensilia),axis=1)
    
    #print(dataLMC)
    #print(dataPrensilia)     
    #print(ite)  
    
    fig, axs = plt.subplots(5)

    axs[0].set_title('Thumb Abduction / Adduction')
    axs[0].set_yticks(np.arange(0,90,10))
    axs[0].set_ylabel('Degree')
    axs[0].plot(dataLMC[:,0],dataLMC[:,1], label='LMC')
    axs[0].plot(dataPrensilia[:,0],dataPrensilia[:,1], label='Prensilia')
    
    axs[1].set_title('Thumb Flexion')
    axs[1].set_yticks(np.arange(0,90,10))
    axs[1].set_ylabel('Degree')
    axs[1].plot(dataLMC[:,0],dataLMC[:,2], label='LMC')
    axs[1].plot(dataPrensilia[:,0],dataPrensilia[:,2], label='Prensilia')
    
    axs[2].set_title('Index Finger Flexion')
    axs[2].set_yticks(np.arange(0,90,10))
    axs[2].set_ylabel('Degree')
    axs[2].plot(dataLMC[:,0],dataLMC[:,3], label='LMC')
    axs[2].plot(dataPrensilia[:,0],dataPrensilia[:,3], label='Prensilia')
    
    axs[3].set_title('Middle Finger Flexion')
    axs[3].set_ylabel('Degree')
    axs[3].set_yticks(np.arange(0,90,10))
    axs[3].plot(dataLMC[:,0],dataLMC[:,4], label='LMC')
    axs[3].plot(dataPrensilia[:,0],dataPrensilia[:,4], label='Prensilia')
    
    axs[4].set_title('Ring / Little Finger Flexion')
    axs[4].set_ylabel('Degree')
    axs[4].set_yticks(np.arange(0,90,10))
    axs[4].plot(dataLMC[:,0],dataLMC[:,5], label='LMC')
    axs[4].plot(dataPrensilia[:,0],dataPrensilia[:,5], label='Prensilia')
    
    
    fig.suptitle('Joint Angle of Fingers from Leap Motion Controller vs from Prensilia Hand Robot')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[3].legend()
    axs[4].legend()
    plt.show()
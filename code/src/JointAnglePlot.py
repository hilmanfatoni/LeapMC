import numpy as np
import matplotlib.pyplot as plt
import math
import csv

if __name__ == "__main__":
    #open file for joint angle

    namaFile = 4    
    with open("rawData/Data%s.txt" % namaFile, "r") as file:
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

    dataLMC=np.concatenate((ite,dataLMC),axis=1)
    dataPrensilia=np.concatenate((ite,dataPrensilia),axis=1)
    
    #print(dataLMC)
    #print(dataPrensilia)     
    #print(ite)  

    
    fig, axs = plt.subplots(5, figsize=(12, 8))
    
    fig.subplots_adjust(hspace=0.6)

    axs[0].set_title('Thumb Abduction / Adduction')
    axs[0].set_ylim([0, 150])
    axs[0].set_yticks(np.arange(0,150,25))
    axs[0].set_ylabel('Degree')
    axs[0].plot(dataLMC[:,0],dataLMC[:,1], label='LMC')
    axs[0].plot(dataPrensilia[:,0],dataPrensilia[:,1], label='Prensilia')
    
    axs[1].set_title('Thumb Flexion')
    axs[1].set_ylim([0, 90])
    axs[1].set_yticks(np.arange(0,90,25))
    axs[1].set_ylabel('Degree')
    axs[1].plot(dataLMC[:,0],dataLMC[:,2], label='LMC')
    axs[1].plot(dataPrensilia[:,0],dataPrensilia[:,2], label='Prensilia')
    
    axs[2].set_title('Index Finger Flexion')
    axs[2].set_ylim([0, 90])
    axs[2].set_yticks(np.arange(0,90,25))
    axs[2].set_ylabel('Degree')
    axs[2].plot(dataLMC[:,0],dataLMC[:,3], label='LMC')
    axs[2].plot(dataPrensilia[:,0],dataPrensilia[:,3], label='Prensilia')
    
    axs[3].set_title('Middle Finger Flexion')
    axs[3].set_ylim([0, 90])
    axs[3].set_ylabel('Degree')
    axs[3].set_yticks(np.arange(0,90,25))
    axs[3].plot(dataLMC[:,0],dataLMC[:,4], label='LMC')
    axs[3].plot(dataPrensilia[:,0],dataPrensilia[:,4], label='Prensilia')
    
    axs[4].set_title('Ring / Little Finger Flexion')
    axs[4].set_ylim([0, 90])
    axs[4].set_ylabel('Degree')
    axs[4].set_yticks(np.arange(0,90,25))
    axs[4].plot(dataLMC[:,0],dataLMC[:,5], label='LMC')
    axs[4].plot(dataPrensilia[:,0],dataPrensilia[:,5], label='Prensilia')
    
    
    fig.suptitle('Joint Angle of Fingers from Leap Motion Controller vs from Prensilia Hand Robot')
    axs[0].legend(loc='upper left')
    axs[1].legend(loc='upper left')
    axs[2].legend(loc='upper left')
    axs[3].legend(loc='upper left')
    axs[4].legend(loc='upper left')

    fig.canvas.manager.set_window_title('Data%s' % namaFile)
    plt.show()

    fig.savefig('Data%s.jpg' % namaFile, dpi=300, bbox_inches='tight')

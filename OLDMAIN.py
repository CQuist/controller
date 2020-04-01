#!/usr/bin/env python

import sys
import time
import numpy as np
import simROS
import cpg
import cpgV2
import csv
import numpy as np
import random

dataFilePath = '/home/quist/Documents/MasterThesis/footContactDataFolder/testWithRandomVariation.csv'
newFile = False

def rescale(newMax, newMin, parameter):
    return (((newMax-newMin) * (parameter+0.2)) / (0.2+0.2)) + newMin

def angleBetweenVectors(currentPos, desiredPos):
    currentPosMagnitute = np.sqrt( pow(currentPos[0],2) + pow(currentPos[1],2) )
    desiredPosMagnitute = np.sqrt( pow(desiredPos[0],2) + pow(desiredPos[1],2) )
    currentPosNormalized = [ currentPos[1]/currentPosMagnitute, currentPos[0]/currentPosMagnitute ]
    desiredPosNormalized = [ desiredPos[0]/desiredPosMagnitute, desiredPos[1]/desiredPosMagnitute ]
    dotProduct = currentPosNormalized[0] * desiredPosNormalized[0] + currentPosNormalized[1] * desiredPosNormalized[1]
    angle = np.arccos(dotProduct)

    #adding one extra element to the vectors so the now are 3d vecotrs (x, y, 1). this is done because cross prodoct can only be performed on 3d vectors
    crossProduct = [currentPosNormalized[1] * 1 - 1 * desiredPosNormalized[1], 1 * desiredPosNormalized[0] - desiredPosNormalized[0] * 1, currentPosNormalized[0] * desiredPosNormalized[1] - currentPosNormalized[1] * desiredPosNormalized[0]]
    if(crossProduct[2] < 0.0):       # if z is negative, the angle is negative
        angle = -angle

    #ROS_INFO_STREAM("angle: " << angle);
    if(angle > 3.1415):
        angle = -(angle - 3.1415);          #subtrack pi and invert, to get the shortest way around

    #ROS_INFO_STREAM("angle return: " << angle);
    return angle

def saveData(footContactForceData, angle):
    with open(dataFilePath, 'a+') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        wr.writerow(([angle] + footContactForceData))
        print("Data printed for Slope at: ", angle)


def main(argv):
    # Init ROS communication
    ros_handle = simROS.SIMROS(argv)
    time.sleep(0.2)  # wait for connections to form
    ros_handle.synchronousSimulation(True)

    # Init CPG
    # MI:
    # wave gait 0.01 - 0.04
    # tetrapod gait 0.05 - 0.06
    # caterpillar gait 0.07 - 0.10# M
    # tripod gait 0.15 - 0.19
    tripodGait = 0.18

    MI = tripodGait
    cpg_handle = cpgV2.CPG(MI)

    # Commands
    # forward = 1, 1/0, -1, -1
    # backwards = 1, 1/0, 1, 1
    # turn left = 1, 1/0, 1, -1
    # turn right = 1, 1/0, -1 , 1

    i1 = 0
    i2 = 0
    i3 = 0
    i4 = 0

    targetAngle = 0
    #targetAngles0 = list(range(1, 41))
    #targetAngles0 = list(range(0, -25, -1))
    #targetAngles1 = list(range(-25, 25, 1))
    #targetAngles2 = list(range(25, -1, -1))
    #targetAngles = targetAngles0 + targetAngles1 + targetAngles2

    numberOfMeasurements = 50
    
    targetAngle = 0
    targetAngles = list(np.random.randint(-25, 26, numberOfMeasurements))
    print(targetAngles)
    
    randomVar = []
    for i in range(18):
        randomVar.append(list(np.random.uniform(-.035, .035, numberOfMeasurements)))
    
    if newFile:
        with open(dataFilePath, 'w+') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            wr.writerow(["Inclination", "LFx", "LFy", "LFz", "LMx", "LMy", "LMz", "LHx", "LHy", "LHz", "RFx", "RFy", "RFz", "RMx", "RMy", "RMz", "RHx", "RHy", "RHz"])

    counter = 0
    # Run controller
    while True:
        
        # If terminate signal is true then exit program
        if ros_handle.terminate():
            print('[ INFO Python main script stopped]')
            ros_handle.synchronousSimulation(False)
            # wait for msg to send
            time.sleep(0.1)
            del ros_handle
            sys.exit(0)

        # Get CPG output
        RF, RM, RH, LF, LM, LH = cpg_handle.get_output()

        # Get feedback and sensory information from simulation

        # T range from -0.45 to 0.45
        # C range from -1.3(stand) to -0.4
        motor_positions = [LF[0], #TC1
                           LF[1], #CF1
                           LF[2], #FT1
                           LM[0], #TC2
                           LM[1], #CF2
                           LM[2],
                           LH[0], #TC3
                           LH[1], #CF3
                           LH[2],
                           RF[0], #TC4
                           RF[1], #CF4
                           RF[2],
                           RM[0], #TC5
                           RM[1], #CF5
                           RM[2],
                           RH[0], #TC6
                           RH[1], #CF6
                           RH[2]]


        #default cf = 2.059
        #tc and ft = 0


        # Step the CPG
        cpg_handle.step(i1, i2, i3, i4)

        if counter == 150:
            saveData(ros_handle.force, targetAngle)
            print("Runs remaining: " ,len(targetAngles))
            if len(targetAngles) <= 0:
                ros_handle.stopSim()
                time.sleep(2)
            else:
                targetAngle = targetAngles.pop(0)
                motor_positions = [randomVar[0].pop(),
                           (randomVar[1].pop() + 2.059),
                           randomVar[2].pop(),
                           randomVar[3].pop(),
                           (randomVar[4].pop() + 2.059),
                           randomVar[5].pop(),
                           randomVar[6].pop(),
                           (randomVar[7].pop() + 2.059),
                           randomVar[8].pop(),
                           randomVar[9].pop(),
                           (randomVar[10].pop() + 2.059),
                           randomVar[11].pop(),
                           randomVar[12].pop(),
                           (randomVar[13].pop() + 2.059),
                           randomVar[14].pop(),
                           randomVar[15].pop(),
                           (randomVar[16].pop() + 2.059),
                           randomVar[17].pop()]
            counter = 0
            ros_handle.setSlopeAngle(targetAngle)
            ros_handle.setLegMotorPosition(motor_positions)

        
        counter = counter + 1

        # Trigger one time step (50ms) in simulation
        ros_handle.triggerSim()


if __name__ == '__main__':
    main(sys.argv[1:])


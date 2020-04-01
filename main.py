#!/usr/bin/env python3

import sim
import sys
import time
import numpy as np
import simROS
import cpgV2
import csv
import numpy as np
import random

dataFilePath = '/home/quist/Documents/MasterThesis/footContactDataFolder/vortexTest.csv'
newFile = False

def saveData(footContactForceData, angle):
    with open(dataFilePath, 'a+') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        wr.writerow(([angle] + footContactForceData))
        print("Data printed for Slope at: ", angle)

def main(argv):
    # https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm --> how to remote API
    # Refer to https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStart

    sim.simxFinish(-1) # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID == -1:
        print('Failed connecting to remote API server')
    else:
        print('Connected to remote API client')

        numberOfRuns = 10
        numberOfMeasurements = 1

        variationSize = 0.001

        for i in range(numberOfRuns):

            count = 0
            dt = 0.0167
            sim.simxSynchronous(clientID, True)

            ################################################### init variables ###################################################

            ros_handle = simROS.SIMROS(argv)
       
            targetAngle = 0
            #targetAngles = list(np.random.randint(-25, 26, numberOfMeasurements))
            targetAngles = list(np.zeros(numberOfMeasurements))
            print(targetAngles)

            randomVar = []
            for i in range(18):
                randomVar.append(list(np.random.uniform(-variationSize, variationSize, numberOfMeasurements)))
        
            if newFile:
                with open(dataFilePath, 'w+') as myfile:
                    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                    wr.writerow(["Inclination", "LFx", "LFy", "LFz", "LMx", "LMy", "LMz", "LHx", "LHy", "LHz", "RFx", "RFy", "RFz", "RMx", "RMy", "RMz", "RHx", "RHy", "RHz"])

            counter = 0

            Done = False
            #Start the simulation
            sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
            ################################################### Control Loop ###################################################
            while not Done:
                #code here


                if counter == 150:
                    saveData(ros_handle.force, targetAngle)
                    print("Runs remaining: " ,len(targetAngles))
                    if len(targetAngles) <= 0:
                        Done = True
                    else:
                        targetAngle = targetAngles.pop(0)
                        motor_positions = [0,
                                            randomVar[1].pop() + 2.059,
                                            randomVar[2].pop(),
                                            0,
                                            randomVar[4].pop() + 2.059,
                                            randomVar[5].pop(),
                                            0,
                                            randomVar[7].pop() + 2.059,
                                            randomVar[8].pop(),
                                            0,
                                            randomVar[10].pop() + 2.059,
                                            randomVar[11].pop(),
                                            0,
                                            randomVar[13].pop() + 2.059,
                                            randomVar[14].pop(),
                                            0,
                                            randomVar[16].pop() + 2.059,
                                            randomVar[17].pop()]
                        counter = 0
                        ros_handle.setSlopeAngle(targetAngle)
                        ros_handle.setLegMotorPosition(motor_positions)

            
                counter = counter + 1

                #Step the simulation
                sim.simxSynchronousTrigger(clientID)


            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
            sim.simxGetPingTime(clientID)
        
        sim.simxFinish(clientID)


if __name__ == '__main__':
    #main(sys.argv[1:])
    simulationTimeTopicName='sim_control/simulationTime'
    startSimulationName='sim_control/startSimulation'
    pauseSimulationName='sim_control/pauseSimulation'
    stopSimulationName='sim_control/stopSimulation'
    enableSyncModeName='sim_control/enableSyncMode'
    triggerNextStepName='sim_control/triggerNextStep'
    simulationStepDoneName='sim_control/simulationStepDone'
    simulationStateName='sim_control/simulationState'

    terminateControllerName='sim_control/terminateController'
    plotterName='sim_control/plotter'

    MotorTopicName='morf_sim/multi_joint_command'
    jointPositionsName='morf_sim/joint_positions'
    jointTorquesName='morf_sim/joint_torques'
    jointVelocitiesName='morf_sim/joint_velocities'
    testParametersName='sim_control/testParameters'

    footContactForceName='morf_sim/footContact_force'
    footContactForceXName='morf_sim/footContact_forceX'
    footContactForceYName='morf_sim/footContact_forceY'
    footContactForceZName='morf_sim/footContact_forceZ'

    robotPosName='morf_sim/robotPos'
    robotOrientationName='morf_sim/robotOrientation'
    slopeTopicName='morf_sim/slopeAngle'

    main([MotorTopicName, simulationTimeTopicName, terminateControllerName, startSimulationName, pauseSimulationName, stopSimulationName, enableSyncModeName, triggerNextStepName, simulationStepDoneName, simulationStateName, plotterName, slopeTopicName, jointPositionsName, jointTorquesName, jointVelocitiesName, testParametersName, robotPosName, robotOrientationName, footContactForceName, footContactForceXName, footContactForceYName, footContactForceZName])

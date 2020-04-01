#!/usr/bin/env python
import numpy as np


class CPG(object):
    def __init__(self, MI):
        self.tau = 15 # 0.6s
        self.tauL = 46#48 # 2s
        # Configuration parameters of the CPG
        self.activityH1 = 0
        self.activityH2 = 0
        self.outputH1 = 0.01
        self.outputH2 = 0.01
        self.biasH1 = 0
        self.biasH2 = 0

        self.outputPcpg1 = 0
        self.outputPcpg2 = 0

        self.w11 = 1.4
        self.w22 = 1.4
        self.w12 = - (0.18 + MI)
        self.w21 = (0.18 + MI)

        self.periodCheck1 = 0
        self.current1 = 0
        self.old1 = 0
        self.upCount1 = 0
        self.downCount1 = 0
        self.slope1 = []
        self.high1 = -1
        self.sawToothOutput1 = 0
        #self.bufferSlope1 = []
        #self.bufferSlope12 = []

        self.periodCheck2 = 0
        self.current2 = 0
        self.old2 = 0
        self.upCount2 = 0
        self.downCount2 = 0
        self.slope2 = []
        self.high2 = -1
        self.sawToothOutput2 = 0
        
        #self.buffer = False
        #self.bufferMode = 1


        self.delayLineOutput = []
        #Top Joint
        self.TR3 = []
        self.TR2 = []
        self.TR1 = []
        self.TL3 = []
        self.TL2 = []
        self.TL1 = []
        #Mid joint
        self.CR3 = []
        self.CR2 = []
        self.CR1 = []
        self.CL3 = []
        self.CL2 = []
        self.CL1 = []
        #foot joint
        self.FR3 = []
        self.FR2 = []
        self.FR1 = []
        self.FL3 = []
        self.FL2 = []
        self.FL1 = []

        for i in range(self.tau):
            self.TR2.append(0)
            self.TR1.append(0)
            self.CR2.append(0)
            self.CR1.append(0)
            self.FR2.append(0)
            self.FR1.append(0)
        for i in range(self.tau):
            self.TR1.append(0)
            self.CR1.append(0)
            self.FR1.append(0)
        for i in range(self.tauL):
            self.TL3.append(0)
            self.TL2.append(0)
            self.TL1.append(0)
            self.CL3.append(0)
            self.CL2.append(0)
            self.CL1.append(0)
            self.FL3.append(0)
            self.FL2.append(0)
            self.FL1.append(0)
        for i in range(self.tau):
            self.TL2.append(0)
            self.TL1.append(0)
            self.CL2.append(0)
            self.CL1.append(0)
            self.FL2.append(0)
            self.FL1.append(0)
        for i in range(self.tau):
            self.TL1.append(0)
            self.CL1.append(0)
            self.FL1.append(0)


        for i in range(3):      #small correction
            self.TL1.append(0)
            self.TL2.append(0)
            self.TL3.append(0)
            self.TR1.append(0)
            self.TR2.append(0)
            self.TR3.append(0)


        self.RF = [0, 0, 0]
        self.RM = [0, 0, 0]
        self.RH = [0, 0, 0]
        self.LF = [0, 0, 0]
        self.LM = [0, 0, 0]
        self.LH = [0, 0, 0]

        self.ready = 0
        

    def pcpg1(self, input):
        if input >= 0.85:
            tempOutputPcpg = 1
        elif input < 0.85:
            tempOutputPcpg = -1

        self.old1 = self.current1
        self.current1 = tempOutputPcpg

        if self.current1 - self.old1 == 2: #go high
            if not self.upCount1 == 0 and not self.downCount1 == 0:
                self.periodCheck1 = 1
                #self.bufferSlope1.clear()
                #self.bufferSlope12.clear()
                for i in range(1, self.upCount1+1):
                    self.slope1.append(-1 + (float((i*2))/float(self.upCount1)))
                    #self.bufferSlope1.append(-1 + ((i*2)/self.upCount1))
                for i in range(1, self.downCount1+1):
                    self.slope1.append(1-(float((i*2))/float(self.downCount1)))
                    #self.bufferSlope1.append(1-((i*2)/self.downCount1))
            self.upCount1 = 1
            self.downCount1 = 1
            self.high1 = 1
        elif self.current1 - self.old1 == -2: # go low
            self.high1 = 0
        else:
            if self.high1 == 1:
                self.upCount1 = self.upCount1 + 1
            elif self.high1 == 0:
                self.downCount1 = self.downCount1 + 1

        #if self.periodCheck1 == 1:
        #    if self.buffer == True:
        #        if self.bufferMode == 1:
        #            if len(self.bufferSlope1) > 0:
        #                self.sawToothOutput1 = self.bufferSlope1.pop(0)
        #                self.bufferSlope12.append(self.sawToothOutput1)
        #            else:
        #                self.bufferMode = 2
        #                self.buffer = False
        #        if self.bufferMode == 2:
        #            if len(self.bufferSlope12) > 0:
        #                self.sawToothOutput1 = self.bufferSlope12.pop(0)
        #                self.bufferSlope1.append(self.sawToothOutput1)
        #            else:
        #                self.bufferMode = 1
        #                self.buffer = False
        #    elif len(self.slope1) > 0:
        #        self.sawToothOutput1 = self.slope1.pop(0)
        #    else:
        #        self.buffer = True

        if self.periodCheck1 == 1:
            if len(self.slope1) > 0:
                self.sawToothOutput1 = self.slope1.pop(0) 

        return self.sawToothOutput1

    def pcpg2(self, input):
        if input >= 0.85:
            tempOutputPcpg = 1
        elif input < 0.85:
            tempOutputPcpg = -1

        self.old2 = self.current2
        self.current2 = tempOutputPcpg

        if self.current2 - self.old2 == 2: #go high
            if not self.upCount2 == 0 and not self.downCount2 == 0:
                self.periodCheck2 = 1
                for i in range(1, self.upCount2+1):
                    self.slope2.append(-1 + (float((i*2))/float(self.upCount2)))
                for i in range(1, self.downCount2+1):
                    self.slope2.append(1-(float((i*2))/float(self.downCount2)))
            self.upCount2 = 1
            self.downCount2 = 1
            self.high2 = 1
        elif self.current2 - self.old2 == -2: # go low
            self.high2 = 0
        else:
            if self.high2 == 1:
                self.upCount2 = self.upCount2 + 1
            elif self.high2 == 0:
                self.downCount2 = self.downCount2 + 1

        if self.periodCheck2 == 1:
            if len(self.slope2) > 0:
                self.sawToothOutput2 = self.slope2.pop(0)
        
        return self.sawToothOutput2

    def neuralMotorControl(self, CP1, CP2, i1, i2, i3, i4):
        A = 1.7246
        B = -2.48285
        C = -1.7246
        H1 = np.tanh(CP1)
        H2 = np.tanh(CP2)
        H3 = np.tanh(i2*(-1) + 1)
        H4 = np.tanh(i2)
        H5 = np.tanh(H1*0.5 + H3*(-5))
        H6 = np.tanh(H2*0.5 + H4*(-5))
        H7 = np.tanh(H2*0.5 + H3*(-5))
        H8 = np.tanh(H1*0.5 + H4*(-5))
        H9 = np.tanh(H5*0.5 + 0.5)
        H10 = np.tanh(H6*0.5 + 0.5)
        H11 = np.tanh(H7*0.5 + 0.5)
        H12 = np.tanh(H8*0.5 + 0.5)
        H13 = np.tanh(H9*3 + H10*3 + (-1.35)) #output for Motor Neurons
        H14 = np.tanh(H11*3 + H12*3 + (-1.35)) #output for Motor Neurons
        H15 = np.tanh(H14*1.75)
        H16 = np.tanh(i3*5)
        H17 = np.tanh(H14*1.75)
        H18 = np.tanh(i4*5)
        H19 = np.tanh(H15*A + H16*A + B)
        H20 = np.tanh(H15*C + H16*C + B)
        H21 = np.tanh(H15*A + H16*C + B)
        H22 = np.tanh(H15*C + H16*A + B)
        H23 = np.tanh(H17*A + H18*A + B)
        H24 = np.tanh(H17*C + H18*C + B)
        H25 = np.tanh(H17*A + H18*C + B)
        H26 = np.tanh(H17*C + H18*A + B)
        H27 = np.tanh(H19*0.5 + H20*0.5 + H21*(-0.5) + H22*(-0.5))
        H28 = np.tanh(H23*0.5 + H24*0.5 + H25*(-0.5) + H26*(-0.5))

        return [H13, H14, H27, H28] 

    def piecewiseLinearFunction(self, input, segments):
        #f(x) = slope * x + b
        # segment -> 0 min, 1 max, 2 slope, 3 bias
        for segment in segments:
            if input > segment[0] and input <= segment[1]:
                return segment[2] * input + segment[3]
        
        return input
    
    def motorNeurons(self, H13, H14, H27, H28, i1, i2, i3, i4):
        #M1 = np.tanh(H13*5 + (-0.5))     #using tanh here, articel says its piecewise linear but code says that its tanh
        #M2 = np.tanh(H14*(-5) + (-0.5))
        #M3 = np.tanh(H14*5 + (-0.5))
        #M4 = np.tanh(H27*(-2.5))
        #M5 = np.tanh(H28*(-2.5))
        
        linearT = [[-5, 5, -0.33, 0]]
        #linearC = [[-2.5, 1, -0.01, -0.4], [1, 5, -0.6, -0.4]]
        linearF = [[-5, 5, 1, 0]]
        linearC = [[-5, 1, 0.01, -1.3], [1, 5, 0.1, -0.4]]


        # the piecewiseLinearFunction should just map the extrem of the NMC neurons that has the output -1 to 1 and map it to for instance 20 to 50 degree
        # scale and clip


        M1 = self.piecewiseLinearFunction((H13*5 + (-0.5)), linearF)  
        M2 = self.piecewiseLinearFunction((H14*(-5) + (-0.5)), linearF)
        M3 = self.piecewiseLinearFunction((H14*5 + (-0.5)), linearC)
        M4 = self.piecewiseLinearFunction((H27*(-2.5)), linearT)
        M5 = self.piecewiseLinearFunction((H28*(-2.5)), linearT)

        self.FL3.append(M2*1.5 * i3*(-1) * i1)
        self.FL2.append(M1*(-1.2) * i1)
        self.FL1.append(M2*(-1.5) * i3*(-1) * i1)
        self.FR3.append(M2*1.5 * i4*(-1) * i1)
        self.FR2.append(M1*(-1.2) * i1)
        self.FR1.append(M2*(-1.5) * i4*(-1) * i1)
        self.CL3.append(M3 * i1)
        self.CL2.append(M3 * i1)
        self.CL1.append(M3 * i1)
        self.CR3.append(M3 * i1)
        self.CR2.append(M3 * i1)
        self.CR1.append(M3 * i1)
        self.TL3.append(M4 * i1)
        self.TL2.append(M4 * i1)
        self.TL1.append(M4 * i1)
        self.TR3.append(M5 * i1)
        self.TR2.append(M5 * i1)
        self.TR1.append(M5 * i1)

        RF = self.TR1.pop(0), self.CR1.pop(0), self.FR1.pop(0)
        RM = self.TR2.pop(0), self.CR2.pop(0), self.FR2.pop(0)
        RH = self.TR3.pop(0), self.CR3.pop(0), self.FR3.pop(0)
        LF = self.TL1.pop(0), self.CL1.pop(0), self.FL1.pop(0)
        LM = self.TL2.pop(0), self.CL2.pop(0), self.FL2.pop(0)
        LH = self.TL3.pop(0), self.CL3.pop(0), self.FL3.pop(0)

        return RF, RM, RH, LF, LM, LH

    def step(self, i1, i2, i3, i4):
        self.activityH1 = self.w11 * self.outputH1 + self.w12 * self.outputH2 + self.biasH1
        self.activityH2 = self.w22 * self.outputH2 + self.w21 * self.outputH1 + self.biasH2
        self.outputH1 = np.tanh(self.activityH1)
        self.outputH2 = np.tanh(self.activityH2)

        self.outputPcpg1 = self.pcpg1(self.outputH1)
        self.outputPcpg2 = self.pcpg2(self.outputH2)

        H13, H14, H27, H28 = self.neuralMotorControl(self.outputPcpg1, self.outputPcpg2, i1, i2, i3, i4)
        self.RF, self.RM, self.RH, self.LF, self.LM, self.LH = self.motorNeurons(H13, H14, H27, H28, i1, i2, i3, i4)

        if self.ready < 140:
            self.ready = self.ready +1

    def setMI(self, MI):
        self.w11 = 1.4
        self.w22 = 1.4
        self.w12 = - (0.18 + MI)
        self.w21 = (0.18 + MI)

    def get_output(self):
        #return [self.outputPcpg1, self.current, self.outputH1]
        #return [self.outputPcpg1, self.outputPcpg2]
        return self.RF, self.RM, self.RH, self.LF, self.LM, self.LH

    def isReady(self):
        return self.ready >= 140
def calculateCG(BLW, BRW, FW):
    """ KNOWN VALUES """
    distF = [0,0] #front wheel
    distF[0] = 6.375 #distance from the nose
    distF[1] = 0 #distance from the centerline

    distLR = [0,0] #left rear wheel
    distLR[0] = 30.375
    distLR[1] = -12.01

    distRR = [0,0] #right rear wheel
    distRR[0] = 30.375
    distRR[1] = 12.01

    mac = 16.4417 # from matlab code 16.4417 for 35 deg
    np = 30.17 # from matlab code 16.4417 for 35 deg
    
    """CALCULATIONS"""
    reply = ''
    try:
        CG = [0,0]
        for i in range(2):
            CG[i] = (FW*distF[i] + BLW*distLR[i] + BRW*distRR[i])/(FW + BLW + BRW)
        rep1 = "CG from nose (in):\t" + str(CG[0])
        rep2 = "\nCG from centerline (in):\t" + str(CG[1])
        rep3 = "\nStatic Margin:\t" + str((np - CG[0])/mac)
        reply = rep1 + rep2 + rep3
    except:
        reply = 'error with calc_CG function\tmake sure inputs are correct'
    
    return reply
print(calculateCG(7.01,7.30,1.71))
print(calculateCG(6.61,6.57,2.41))
'''
print(calculateCG(6.79, 6.75, 2.21))
print(calculateCG(6.93, 6.88, 2.05))
print(calculateCG(7.06, 7.01, 1.92))
print(calculateCG(7.19, 7.14, 1.76))
print(calculateCG(7.31, 7.28, 1.63))
print(calculateCG(6.96, 6.92, 2.01))
print(calculateCG(7.24, 7.2, 1.72))
print(calculateCG(7.51, 7.48, 1.41))
print(calculateCG(7.77, 7.75, 1.1))
print(calculateCG(8.01, 8, 0.84))
print(calculateCG(6.87, 6.82, 2.1))
print(calculateCG(7.06, 7.01, 1.91))
print(calculateCG(7.2, 7.16, 1.75))
print(calculateCG(7.33, 7.27, 1.6))
print(calculateCG(7.47, 7.42, 1.45))
print(calculateCG(7.61, 7.54, 1.3))
print(calculateCG(7.24, 7.17, 1.71))
print(calculateCG(7.51, 7.46, 1.41))
print(calculateCG(7.78, 7.74, 1.1))
print(calculateCG(8.04, 7.99, 0.81))
print(calculateCG(8.3, 8.26, 0.53)) '''

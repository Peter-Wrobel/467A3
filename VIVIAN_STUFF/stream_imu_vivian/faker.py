

value7 = 0

for i in range ( 1, 350):
    name = "data" + str(i) + ".imu"
    f= open(name,"w+")

    value1 = 0
    value2 = 0
    value3 = 0
    value4 = 0
    value5 = 0
    value6 = 0 
    value7 += 0.01
    if i < 10 :
        value1 = 0.1
        value2 = 0
        value3 = 0
        value4 = 0
        value5 = 0
        value6 = 0


    elif i <  80:
        value1 = 0.0
        value2 = 0
        value3 = 0
        value4 = 0
        value5 = 0
        value6 = 0


    elif i <  90:
        value1 = -0.1
        value2 = 0
        value3 = 0
        value4 = 0
        value5 = 0
        value6 = 0

    elif i <  141:
        value1 = 0.01
        value2 = 0.01
        value3 = 0
        value4 = 0.26
        value5 = -0.02
        value6 = 0
    
    elif i <  151:
        value1 = 0.1
        value2 = 0.0
        value3 = 0
        value4 = 0.0
        value5 = 0.0
        value6 = 0

    elif i <  220:
        value1 = 0
        value2 = 0.0
        value3 = 0
        value4 = 0.0
        value5 = 0.0
        value6 = 0

    elif i <  228:
        value1 = -0.1
        value2 = 0.0
        value3 = 0
        value4 = 0.0
        value5 = 0.0
        value6 = 0

    elif i <  249:
        value1 = 0.01
        value2 = 0.01
        value3 = 0
        value4 = -0.26
        value5 = -0.02
        value6 = 0


    if i < 260 :
        value1 = 0.1
        value2 = 0
        value3 = 0
        value4 = 0
        value5 = 0
        value6 = 0

    if i < 320 :
        value1 = 0
        value2 = 0
        value3 = 0
        value4 = 0
        value5 = 0
        value6 = 0

    elif i <  340:
        value1 = -0.1
        value2 = 0.0
        value3 = 0
        value4 = 0.0
        value5 = 0.0
        value6 = 0

    else: 
        value1 = 0
        value2 = 0.0
        value3 = 0
        value4 = 0.0
        value5 = 0.0
        value6 = 0

    f.write( "%.3f " % value1)
    f.write( "%.3f " % value2)
    f.write( "%.3f " % value3)
    f.write( "%.3f " % value4)
    f.write( "%.3f " % value5)
    f.write( "%.3f " % value6)
    f.write( "%.3f " % value7)
    #  "%.3f" % value2,"%.3f" % value3 "%.3f" % value4 "%.3f" % value5 "%.3f" % value6)
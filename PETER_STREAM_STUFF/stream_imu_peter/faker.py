

value7 = 0

for i in range ( 1, 306):
    name = "data" + str(i) + ".imu"
    f= open(name,"w+")

    value1 = 0
    value2 = 0
    value3 = 0
    value4 = 0
    value5 = 0
    value6 = 0 
    value7 += 0.01
    if i < 36 :
        value1 = 0.0
        value2 = 0
        value3 = 1
        value4 = 0
        value5 = 0
        value6 = 0


    elif i <  62:
        value1 = 0.09
        value2 = -0.03
        value3 = 0
        value4 = 0.16
        value5 = -0.0
        value6 = 0


    elif i <  190:
        value1 = -0.02
        value2 = 0.03
        value3 = 0
        value4 = -0.26
        value5 = -0.02
        value6 = 0

    elif i <  244:
        value1 = 0.01
        value2 = 0.01
        value3 = 0
        value4 = 0.26
        value5 = -0.02
        value6 = 0
    
    elif i <  244:
        value1 = -0.01
        value2 = 0.01
        value3 = 0
        value4 = 0.0
        value5 = -0.02
        value6 = 0




    f.write( "%.3f " % value1)
    f.write( "%.3f " % value2)
    f.write( "%.3f " % value3)
    f.write( "%.3f " % value4)
    f.write( "%.3f " % value5)
    f.write( "%.3f " % value6)
    f.write( "%.3f " % value7)
    #  "%.3f" % value2,"%.3f" % value3 "%.3f" % value4 "%.3f" % value5 "%.3f" % value6)
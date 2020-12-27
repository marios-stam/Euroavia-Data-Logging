file = open("test.txt", "r")
content=file.read()
measurements=content.split('\n')


class Log:
    def __init__(self, alt,temp,acceleration,orientation,pressure):
        #self.time = time
        self.alt=float(alt)
        self.temp = float(temp)
        self.orientation=[float(i) for i in orientation]
        self.acceleration=[float(i) for i in acceleration]
        self.pressure=float(pressure)
    
    

Logs=[]
for i in range(len(measurements)):
    l=measurements[i].split(' ')
    Logs.append(Log(l[0], (int(l[1])+int(l[2]))/2 ,[ l[3],l[4],l[5] ] ,[ l[6],l[7],l[8] ], l[9] ) )


#time=[i.time for i in Logs]



        	

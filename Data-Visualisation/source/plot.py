import numpy as np
import matplotlib.pyplot as plt
import process_txt as txt

logs=txt.Logs
alts,temps,pressures,accel_y=[],[],[],[]
for i in logs :
    print(i.alt)
    alts.append(i.alt)
    temps.append(i.temp)
    pressures.append(i.pressure)
    accel_y.append(i.acceleration[1])
if __name__=='main':
    plt.subplot(321)
    plt.title('Altitude')
    plt.plot(alts)

    plt.subplot(322)
    plt.title('Pressure')
    plt.plot(pressures)

    plt.subplot(312)
    plt.title('Temperature')
    plt.plot(temps)

    plt.subplot(313)
    plt.title('Accel-Y')
    plt.plot(accel_y)

    plt.subplots_adjust(hspace=0.5)
    plt.show()
        

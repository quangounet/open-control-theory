from numpy import *
from pylab import * 
from control.matlab import *   
import time
ion()


# Bode plots
sys = tf([1,-100],[10,0.1,1])
freq = logspace(-5,5,2000)
amp, ang, _ = bode(sys,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp,'r',linewidth=2)
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
axis([freq[0],freq[-1],-120,120])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang,'r',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi/2,0,pi/2,pi],["$-\pi/2$","$0$","$\pi/2$","$\pi$"])
axis([freq[0],freq[-1],-pi,3*pi/2])
tight_layout()
#savefig("../notes/fig/exbode.pdf")

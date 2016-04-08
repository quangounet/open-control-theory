from numpy import *
from pylab import * 
from control.matlab import *   
import time
ion()


# Proportional control, step response
sysp1 = tf([2], [1, 2, 2])
sysp2 = tf([2,2], [1, 2, 2])
Tend = 10
zvect1, tvect1 = step(sysp1, T = arange(0, Tend, 0.01))
zvect2, tvect2 = step(sysp2, T = arange(0, Tend, 0.01))
clf()
plot([0,Tend],[1,1], "k--", linewidth = 2)
plot(tvect1, zvect1, "r", linewidth = 2, label = "PV")
plot(tvect2, zvect2, "g--", linewidth = 2, label = "PD")
grid("on")
axis([0,Tend,-0.1,1.5])
xlabel("t", fontsize = 20)
ylabel("c(t)", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/stepresp.pdf")



# Bode plots
a = 1.2
sys = tf([1],[1,0.2,1.01+a,0.1+0.1*a,a])
freq = logspace(-5,5,5000)
amp, ang, _ = bode(sys,freq,Plot=False,dB=True,deg=False)
#amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp,'r',linewidth=2)
#plot(freq,amp2,'g',linewidth=2)
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
axis([freq[0],freq[-1],-120,120])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang,'r',linewidth=2)
#plot(freq,ang2,'g',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi/2,0,pi/2,pi],["$-\pi/2$","$0$","$\pi/2$","$\pi$"])
axis([freq[0],freq[-1],-pi,3*pi/2])
tight_layout()
#savefig("../notes/fig/exbode.pdf")

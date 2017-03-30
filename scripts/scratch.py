from numpy import *
from pylab import * 
from control.matlab import *   
import time
ion()


# # Proportional control, step response
# sysp1 = tf([2], [1, 2, 2])
# sysp2 = tf([2,2], [1, 2, 2])
# Tend = 10
# zvect1, tvect1 = step(sysp1, T = arange(0, Tend, 0.01))
# zvect2, tvect2 = step(sysp2, T = arange(0, Tend, 0.01))
# clf()
# plot([0,Tend],[1,1], "k--", linewidth = 2)
# plot(tvect1, zvect1, "r", linewidth = 2, label = "PV")
# plot(tvect2, zvect2, "g--", linewidth = 2, label = "PD")
# grid("on")
# axis([0,Tend,-0.1,1.5])
# xlabel("t", fontsize = 20)
# ylabel("c(t)", fontsize = 20)
# legend(prop={'size':18})
# savefig("../notes/fig/stepresp.pdf")





# Root locus
sys = tf([1],[1,5,8,6,0])
rvect, kvect = rlocus(sys, klist = arange(0,40,0.01))
axes().set_aspect("equal")
#savefig("../notes/fig/rlocusex.pdf")




# Bode plots
#sys = tf([3,5,300],[1])

# band pass
R1 = 1
R2 = 10
C1 = 1e-3
C2 = 1e-6
w1 = 1/(R1*C1)
w2 = 1/(R2*C2)
sys = tf([-R2/w1,0],[R1/(w1*w2),R1*(1/w1+1/w2),R1])


# low pass
sys = tf([1],[10,1])

# high pass
sys = tf([10,0],[10,1])

# homework 3
sys = tf(4*array([1,1,25]),array([1,100,0,0]))


freq = logspace(-2,4,5000)
amp, ang, _ = bode(sys,freq,Plot=False,dB=True,deg=False)
#amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp,'r',linewidth=2)
#plot([w1,w1],[-100,100],"k")
#plot([w2,w2],[-100,100],"k")
#plot(freq,amp2,'g',linewidth=2)
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
axis([freq[0],freq[-1],-80,80])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang,'r',linewidth=2)
#plot([w1,w1],[-100,100],"k")
#plot([w2,w2],[-100,100],"k")
#plot(freq,ang2,'g',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi,-pi/2,0],["$-\pi$","-$\pi/2","0$"])
axis([freq[0],freq[-1],-pi,0])
tight_layout()
#savefig("../notes/fig/exbode.pdf")

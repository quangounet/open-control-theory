from numpy import *
from pylab import * 
from control.matlab import *   
import time
ion()


#### Exam 2019 ####

sys = tf([1, 0, 0, 0.01], [1, 10, 0,0])
freq = logspace(-5, 5, 5000)
amp, ang, _ = bode(sys, freq, Plot=False, dB=True, deg=False)
figure(0)
clf()
plot(freq, amp, 'r', linewidth=2)
grid("on")
xscale('log')
xlabel("Frequency (rad/s)", fontsize = 20)
ylabel("Gain (dB)", fontsize = 20)
axis([freq[0], freq[-1], -40, 140])


sys = tf([1,4,5],[1,6,13])
rvect, kvect = rlocus(sys, klist = arange(0,1000,0.1))
close('all')
clf()
grid("on")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0,yvect0,"m", linewidth = 3)
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "$K=0$")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect0[-1],yvect0[-1],"og",markersize=10,label = "$K=\infty$")
plot(xvect1[-1],yvect1[-1],"og",markersize=10)
axis([-6,1,-3,3])
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
axes().set_aspect("equal")


sys = tf([1],[1,5,9,5])
rvect, kvect = rlocus(sys, klist = arange(0,1000,0.1))

sys = tf([1], [1, 5, 100])
freq = logspace(-5, 5, 5000)
amp, ang, _ = bode(sys, freq, dB=True, deg=False)


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

freq = logspace(-5,5,5000)
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
axis([freq[0],freq[-1],-50,30])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang,'r',linewidth=2)
#plot([w1,w1],[-100,100],"k")
#plot([w2,w2],[-100,100],"k")
#plot(freq,ang2,'g',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi,0,pi],["$-\pi$","0","$\pi$"])
axis([freq[0],freq[-1],-pi,pi])
tight_layout()
#savefig("../notes/fig/exbode.pdf")

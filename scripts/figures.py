# -*- coding: utf-8 -*-
# Copyright (C) 2015 Quang-Cuong Pham <cuong.pham@normalesup.org>
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option, any later version.
#
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


from numpy import *
from pylab import * 
from control.matlab import *   
import time
ion()



########################## Time response ################################

m, k = 2, 0.5


# Proportional control, step response
K1 = 0.1
K2 = 0.5
sysp1 = tf([k*K1], [m, 0, k*K1])
sysp2 = tf([k*K2], [m, 0, k*K2])
Tend = 100
zvect1, tvect1 = step(sysp1, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(sysp2, T = arange(0, Tend, 0.1))
clf()
plot([0,Tend],[1,1], "k--", linewidth = 2)
plot(tvect1, zvect1, "r", linewidth = 2, label = "K=0.1")
plot(tvect2, zvect2, "g--", linewidth = 2, label = "K=0.5")
grid("on")
axis([0,Tend,-0.1,2.1])
xlabel("Time (s)", fontsize = 20)
ylabel("Altitude (m)", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/stepresp.pdf")


# Proportional derivative control, step response
KP1 = 0.1
KP2 = 0.5
KD1 = 0.2
KD2 = 0.8
syspd1 = tf([k*KD1,k*KP1], [m, k*KD1, k*KP1])
syspd2 = tf([k*KD1,k*KP2], [m, k*KD1, k*KP2])
syspd3 = tf([k*KD2,k*KP2], [m, k*KD2, k*KP2])
Tend = 100
zvect1, tvect1 = step(syspd1, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(syspd2, T = arange(0, Tend, 0.1))
zvect3, tvect3 = step(syspd3, T = arange(0, Tend, 0.1))
clf()
plot([0,Tend],[1,1], "k--", linewidth = 2)
plot(tvect1, zvect1, "r", linewidth = 2, label = "$K_D=0.2$, $K_P=0.1$")
plot(tvect2, zvect2, "g--", linewidth = 2, label = "$K_D=0.2$, $K_P=0.5$")
plot(tvect3, zvect3, "b:", linewidth = 4, label = "$K_D=0.8$, $K_P=0.5$")
grid("on")
axis([0,Tend,-0.1,2.1])
xlabel("Time (s)", fontsize = 20)
ylabel("Altitude (m)", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/steprespd.pdf")


# PD control, unit parabolic response
KP1 = 0.1
KP2 = 0.5
KD1 = 0.2
KD2 = 0.8
sysref = tf([1], [1,0,0])
syspd1 = tf([k*KD1,k*KP1], [m, k*KD1, k*KP1,0,0])
syspd2 = tf([k*KD1,k*KP2], [m, k*KD1, k*KP2,0,0])
syspd3 = tf([k*KD2,k*KP2], [m, k*KD2, k*KP2,0,0])
Tend = 100
zvectref, tvectref = step(sysref, T = arange(0, Tend, 0.1))
zvect1, tvect1 = step(syspd1, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(syspd2, T = arange(0, Tend, 0.1))
zvect3, tvect3 = step(syspd3, T = arange(0, Tend, 0.1))
clf()
plot(tvectref, zvectref, "k", linewidth = 2, label = "Unit parabola")
plot(tvect1, zvect1, "r", linewidth = 2, label = "$K_D=0.2$, $K_P=0.1$")
plot(tvect2, zvect2, "g--", linewidth = 2, label = "$K_D=0.2$, $K_P=0.5$")
plot(tvect3, zvect3, "b:", linewidth = 4, label = "$K_D=0.8$, $K_P=0.5$")
grid("on")
axis([0,40,0,500])
xlabel("Time (s)", fontsize = 20)
ylabel("Altitude (m)", fontsize = 20)
legend(loc=2,prop={'size':18})
savefig("../notes/fig/parabres.pdf")


# PD control, unit parabolic error
KP1 = 0.1
KP2 = 0.5
KD1 = 0.2
KD2 = 0.8
syserrpd1 = tf([m], [m, k*KD1, k*KP1])
syserrpd2 = tf([m], [m, k*KD1, k*KP2])
syserrpd3 = tf([m], [m, k*KD2, k*KP2])
Tend = 100
zvect1, tvect1 = step(syserrpd1, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(syserrpd2, T = arange(0, Tend, 0.1))
zvect3, tvect3 = step(syserrpd3, T = arange(0, Tend, 0.1))
clf()
plot(tvect1, zvect1, "r", linewidth = 2, label = "$K_D=0.2$, $K_P=0.1$")
plot(tvect2, zvect2, "g--", linewidth = 2, label = "$K_D=0.2$, $K_P=0.5$")
plot(tvect3, zvect3, "b:", linewidth = 4, label = "$K_D=0.8$, $K_P=0.5$")
grid("on")
axis([0,Tend,-10,90])
xlabel("Time (s)", fontsize = 20)
ylabel("Altitude error (m)", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/paraberr.pdf")



# Unit parabolic error compensated
KP = 0.5
KD = 0.8
pc = 0.05
zc = 0.005
nolagerr = tf([m], [m, k*KD, k*KP])
lagerr = tf([m,m*zc],[m,m*zc+k*KD,k*KP+k*pc*KD,k*pc*KP])

Tend = 100
zvect1, tvect1 = step(nolagerr, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(lagerr, T = arange(0, Tend, 0.1))

clf()
plot(tvect1, zvect1, "b", linewidth = 2, label = "Uncompensated")
plot(tvect2, zvect2, "m--", linewidth = 2, label = "Compensated")
grid("on")
axis([0,Tend,-5,15])
xlabel("Time (s)", fontsize = 20)
ylabel("Altitude error (m)", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/paraberrcomp.pdf")


# Unit step response compensated
KP = 0.5
KD = 0.8
pc = 0.05
zc = 0.005
nolagres = tf([k*KD,k*KP], [m, k*KD, k*KP])
lagres = tf([k*KD,k*KD*pc+k*KP,pc*k*KP],[m,m*zc+k*KD,k*KP+k*pc*KD,k*pc*KP])
Tend = 100
zvect1, tvect1 = step(nolagres, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(lagres, T = arange(0, Tend, 0.1))
clf()
plot(tvect1, zvect1, "b", linewidth = 2, label = "Uncompensated")
plot(tvect2, zvect2, "m--", linewidth = 2, label = "Compensated")
grid("on")
axis([0,Tend,-0.1,2.1])
xlabel("Time (s)", fontsize = 20)
ylabel("Altitude (m)", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/parabrescomp.pdf")



############################# Root locus #################################


# Root locus example
sys = tf([1,0],[1,2,5])
rvect, kvect = rlocus(sys, klist = arange(0,10,0.001))
close('all')
clf()
grid("on")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0,yvect0,"m", linewidth = 3, label = "K= 0...10")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "K=0")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect0[1000],yvect0[1000],"sc",markersize=10,label = "K=1")
plot(xvect1[1000],yvect1[1000],"sc",markersize=10)
plot(xvect0[5000],yvect0[5000],"pb",markersize=12,label = "K=5")
plot(xvect1[5000],yvect1[5000],"pb",markersize=12)
axis([-10,2,-4,4])
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
axes().set_aspect("equal")
savefig("../notes/fig/rlocusex.pdf")



# Root locus PD control
m, k = 2, 0.5
sys = tf([k,k],[m,0,0])
rvect, kvect = rlocus(sys, klist = arange(0,50,0.1))
close('all')
clf()
grid("on")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0,yvect0,"m", linewidth = 3, label = "K= 0...50")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "K=0")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect0[100],yvect0[100],"sc",markersize=10,label = "K=10")
plot(xvect1[100],yvect1[100],"sc",markersize=10)
plot(xvect0[160],yvect0[160],"pb",markersize=12,label = "K=16")
plot(xvect1[160],yvect1[160],"pb",markersize=12)
plot(xvect0[200],yvect0[200],"hy",markersize=12,label = "K=20")
plot(xvect1[200],yvect1[200],"hy",markersize=12)
axis([-5,2,-3,3])
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
axes().set_aspect("equal")
savefig("../notes/fig/rlocusex2.pdf")


# Root locus PD control (animated)
m, k = 2, 0.5
sys = tf([k,k],[m,0,0])
rvect, kvect = rlocus(sys, klist = arange(0,50,0.1))
close('all')
for i in range(0,50,1):
    k = i*10
    clf()
    grid("on")
    xvect0 = array([real(r[0]) for r in rvect])
    yvect0 = array([imag(r[0]) for r in rvect])
    xvect1 = array([real(r[1]) for r in rvect])
    yvect1 = array([imag(r[1]) for r in rvect])
    plot([-100,100],[0,0],"k")
    plot([0,0],[-100,100],"k")
    axis([-6,2,-3,3])
    xlabel("Real", fontsize = 20)
    ylabel("Imag", fontsize = 20)
    axes().set_aspect("equal")
    x = round(xvect0[k],2)
    y = round(-yvect0[k]+1e-10,2)
    label = "$K="+str(i)+",\ s_1="+str(x)+"+j"+str(y)+"$"
    p1 = plot(xvect0[k],yvect0[k],"or",markersize=10,label = label)
    p2 = plot(xvect1[k],yvect1[k],"or",markersize=10)
    legend(loc=2 , prop={'size':18})
    draw()
    if i<10:
        ss = "0"+str(i)
    else:
        ss = str(i)
    savefig("anim/rlocuspd-"+ss+".png")
plot(xvect0,yvect0,"m", linewidth = 3)
plot(xvect1,yvect1,"m", linewidth = 3)
savefig("anim/rlocuspd-50.png")
savefig("anim/rlocuspd-51.png")
savefig("anim/rlocuspd-52.png")
savefig("anim/rlocuspd-53.png")
savefig("anim/rlocuspd-54.png")
savefig("anim/rlocuspd-55.png")
# Execute in the terminal ffmpeg -f image2 -r 6 -i 'rlocuspd-%02d.png' output.mp4


# Root locus start/end
sys = tf([1,2,5],[1,3,20])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.1))
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
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "K=0 (poles)")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect0[599],yvect0[599],"og",markersize=10,label = "K=$\infty$ (zeros)")
plot(xvect1[599],yvect1[599],"og",markersize=10)
k = 10
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
axis([-8,3,-5,5])
axes().set_aspect("equal")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/rlocusda.pdf")


# Asymptotes
sys = tf([1,0.5],[1,5,31,55,100])
rvect, kvect = rlocus(sys, klist = arange(0,1200,0.1))
close('all')
clf()
grid("on")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
axis([-11,4,-8,8])
axes().set_aspect("equal")
plot(xvect0,yvect0,"m", linewidth = 3)
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "K=0 (poles)")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10,label = "K=$\infty$ (zeros)")
k = 800
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
k = 200
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect3[k]],[yvect3[k]],[xvect3[k+3]-xvect3[k]],[yvect3[k+3]-yvect3[k]],color="m",angles="xy",scale_units="xy")
k = 585
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect3[k]],[yvect3[k]],[xvect3[k+3]-xvect3[k]],[yvect3[k+3]-yvect3[k]],color="m",angles="xy",scale_units="xy")
K = 100
plot([-1.5,-1.5+K], [0,K*tan(pi/3)],"k--",linewidth=2, label="Asymptotes")
plot([-1.5,-1.5+K], [0,-K*tan(pi/3)],"k--",linewidth=2)
plot([-1.5,-1.5-K], [0,0],"k--",linewidth=2)
axes().set_aspect("equal")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/rlocusasymp.pdf")






######################## Lead controllers design #######################

# Uncompensated
# (s+1)(s+2)(s+4) = s^3+7s^2+14s+8

sys = tf([1],[1,7,14,8])
sdes = -2+1.9j
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
close('all')
clf()
grid("on")
axis([-6,1,-3,3])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0,yvect0,"m", linewidth = 3, label = "Root locus")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot([real(sdes)],[imag(sdes)],"sb",markersize=10,label = "$s_\mathrm{desired}$")
plot([real(sdes)],[-imag(sdes)],"sb",markersize=10)
k = 800
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 2000
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 10
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/design-uncomp.pdf")


# PD
D = poly1d([1,7,14,8])
sdes = -2+1.9j
adef = mod(angle(D(sdes)) - pi,2*pi)
zc = -real(sdes)+imag(sdes)/tan(adef)
K = norm(D(sdes)/(sdes+zc))
Kp =  K*zc/D(0)
print "K =", K
print "Kp =", Kp
print "ESS =", 1/(1+Kp)

sys = tf([1],[1,7,14,8])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
sys = tf([1,zc],D.coeffs)
rvect, kvect = rlocus(sys, klist = arange(0,100,0.005))
close('all')
clf()
grid("on")
axis([-6,1,-3,3])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (PD)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect0[-1],yvect0[-1],"og",markersize=10,label = "Zero")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot([real(sdes)],[imag(sdes)],"sb",markersize=10,label = "$s_\mathrm{desired}$")
plot([real(sdes)],[-imag(sdes)],"sb",markersize=10)
k = 400
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 500
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 10
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/design-PD.pdf")



# Lead zc=1.5
D = poly1d([1,7,14,8])
sdes = -2+1.9j
zc = 1.5
adef = mod(angle(sdes+zc)-angle(D(sdes)) - pi,2*pi)
pc = -real(sdes)+imag(sdes)/tan(adef)
K = norm(D(sdes)*(sdes+pc)/(sdes+zc))
Kp =  K*zc/(D(0)*pc)
print "pc =",pc
print "K =", K
print "Kp =", Kp
print "ESS =", 1/(1+Kp)
D2=D*poly1d([1,pc])
sys = tf([1],[1,7,14,8])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
sys = tf([1,zc],D2.coeffs)
rvect, kvect = rlocus(sys, klist = arange(0,60,0.005))
close('all')
clf()
grid("on")
axis([-6,1,-3,3])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (lead)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10,label = "Zero")
plot([real(sdes)],[imag(sdes)],"sb",markersize=10,label = "$s_\mathrm{desired}$")
plot([real(sdes)],[-imag(sdes)],"sb",markersize=10)
k = 200
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 1000
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 200
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 100
quiver([xvect3[k]],[yvect3[k]],[xvect3[k+1]-xvect3[k]],[yvect3[k+1]-yvect3[k]],color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/design-lead15.pdf")


# Lead zc=1
D = poly1d([1,7,14,8])
sdes = -2+1.9j
zc = 1
adef = mod(angle(sdes+zc)-angle(D(sdes)) - pi,2*pi)
pc = -real(sdes)+imag(sdes)/tan(adef)
K = norm(D(sdes)*(sdes+pc)/(sdes+zc))
Kp =  K*zc/(D(0)*pc)
print "pc =",pc
print "K =", K
print "Kp =", Kp
print "ESS =", 1/(1+Kp)
D2=D*poly1d([1,pc])
sys = tf([1],[1,7,14,8])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
sys = tf([1,zc],D2.coeffs)
rvect, kvect = rlocus(sys, klist = arange(0,60,0.005))
close('all')
clf()
grid("on")
axis([-6,1,-3,3])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (lead)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10,label = "Zero")
plot([real(sdes)],[imag(sdes)],"sb",markersize=10,label = "$s_\mathrm{desired}$")
plot([real(sdes)],[-imag(sdes)],"sb",markersize=10)
k = 200
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 1000
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 100
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/design-lead10.pdf")



# Lead zc=0.5
D = poly1d([1,7,14,8])
sdes = -2+1.9j
zc = 0.5
adef = mod(angle(sdes+zc)-angle(D(sdes)) - pi,2*pi)
pc = -real(sdes)+imag(sdes)/tan(adef)
K = norm(D(sdes)*(sdes+pc)/(sdes+zc))
Kp =  K*zc/(D(0)*pc)
print "pc =",pc
print "K =", K
print "Kp =", Kp
print "ESS =", 1/(1+Kp)
D2=D*poly1d([1,pc])
sys = tf([1],[1,7,14,8])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
sys = tf([1,zc],D2.coeffs)
rvect, kvect = rlocus(sys, klist = arange(0,60,0.001))
close('all')
clf()
grid("on")
axis([-6,1,-3,3])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (lead)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10,label = "Zero")
plot([real(sdes)],[imag(sdes)],"sb",markersize=10,label = "$s_\mathrm{desired}$")
plot([real(sdes)],[-imag(sdes)],"sb",markersize=10)
k = 1000
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 2500
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 100
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 100
quiver([xvect3[k]],[yvect3[k]],[xvect3[k+1]-xvect3[k]],[yvect3[k+1]-yvect3[k]],color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/design-lead05.pdf")



######################## Lag controllers design #######################

# Uncompensated
# s(s+3)(s+4) = s^3+7s^2+12s+0

sys = tf([1],[1,7,12,0])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
close('all')
clf()
grid("on")
axis([-5,1,-2.5,2.5])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0,yvect0,"m", linewidth = 3, label = "Root locus")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect1[2000],yvect1[2000],"sb",markersize=10,label = "K=20")
plot(xvect2[2000],yvect2[2000],"sb",markersize=10)
k = 200
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 1000
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 200
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':18})
savefig("../notes/fig/design-uncomp2.pdf")


# PI
# s(s+3)(s+4) = s^3+7s^2+12s+0

sys = tf([1],[1,7,12,0])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
sys = tf([1,0.2],[1,7,12,0,0])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.005))
close('all')
clf()
grid("on")
axis([-5,1,-2.5,2.5])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (PI)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect1u[2000],yvect1u[2000],"hc",markersize=12,label = "K=20 (uncomp)")
plot(xvect2u[2000],yvect2u[2000],"hc",markersize=12)
plot(xvect1[2000*2],yvect1[2000*2],"sb",markersize=10,label = "K=20 (PI)")
plot(xvect2[2000*2],yvect2[2000*2],"sb",markersize=10)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10, label = "Zero")
k = 500
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 2000
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 1100
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
k = 800
quiver([xvect2[k]],[yvect2[k]],array([xvect2[k+1]-xvect2[k]]),array([yvect2[k+1]-yvect2[k]]),color="m",angles="xy",scale_units="xy")
quiver([xvect3[k]],[yvect3[k]],[xvect3[k+1]-xvect3[k]],[yvect3[k+1]-yvect3[k]],color="m",angles="xy",scale_units="xy")
k = 1459
quiver([xvect2[k]],[yvect2[k]],array([xvect2[k+1]-xvect2[k]]),array([yvect2[k+1]-yvect2[k]]),color="m",angles="xy",scale_units="xy")
k = 1459
quiver([xvect3[k]],[yvect3[k]],array([xvect3[k+1]-xvect3[k]]),array([yvect3[k+1]-yvect3[k]]),color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':15})
savefig("../notes/fig/design-PI.pdf")


# PI2
# s(s+3)(s+4) = s^3+7s^2+12s+0

sys = tf([1],[1,7,12,0])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
sys = tf([1,0.1],[1,7,12,0,0])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.005))
close('all')
clf()
grid("on")
axis([-5,1,-2.5,2.5])
axes().set_aspect("equal")
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (PI)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect1u[2000],yvect1u[2000],"hc",markersize=12,label = "K=20 (uncomp)")
plot(xvect2u[2000],yvect2u[2000],"hc",markersize=12)
plot(xvect1[2000*2],yvect1[2000*2],"sb",markersize=10,label = "K=20 (PI)")
plot(xvect2[2000*2],yvect2[2000*2],"sb",markersize=10)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10, label = "Zero")
k = 500
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 2000
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 1100
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
k = 1000
quiver([xvect2[k]],[yvect2[k]],array([xvect2[k+1]-xvect2[k]]),array([yvect2[k+1]-yvect2[k]]),color="m",angles="xy",scale_units="xy")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':15})
savefig("../notes/fig/design-PI2.pdf")

# Response
N = poly1d([1])
sterm = poly1d([1,0])
D = poly1d([1,7,12,0])
D2 = sterm*D
K = 20
sys = tf((K*N).coeffs,(K*N+D).coeffs)
N = poly1d([1,0.2])
sys1 = tf((K*N).coeffs,(K*N+D2).coeffs)
N = poly1d([1,0.1])
sys2 = tf((K*N).coeffs,(K*N+D2).coeffs)

Tend = 20
zvect, tvect = step(sys, T = arange(0, Tend, 0.1))
zvect1, tvect1 = step(sys1, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(sys2, T = arange(0, Tend, 0.1))

clf()
plot(tvect, zvect, "r", linewidth = 2, label = "Uncomp")
plot(tvect1, zvect1, "g--", linewidth = 2, label = "PI, $z_c=0.2$")
plot(tvect2, zvect2, "b:", linewidth = 4, label = "PI, $z_c=0.1$")
grid("on")

axis([0,Tend,-0.1,1.5])
xlabel("Time (s)", fontsize = 20)
ylabel("Step response", fontsize = 20)
legend(loc=4,prop={'size':18})
savefig("../notes/fig/design-PI-resp.pdf")

K = 20
N = poly1d([1])
sys = tf(D.coeffs,(sterm*(K*N+D)).coeffs)
N = poly1d([1,0.2])
sys1 = tf(D2.coeffs,(sterm*(K*N+D2)).coeffs)
N = poly1d([1,0.1])
sys2 = tf(D2.coeffs,(sterm*(K*N+D2)).coeffs)

Tend = 40
zvect, tvect = step(sys, T = arange(0, Tend, 0.1))
zvect1, tvect1 = step(sys1, T = arange(0, Tend, 0.1))
zvect2, tvect2 = step(sys2, T = arange(0, Tend, 0.1))
clf()
plot(tvect, zvect, "r", linewidth = 2, label = "Uncomp")
plot(tvect1, zvect1, "g--", linewidth = 2, label = "PI, $z_c=0.2$")
plot(tvect2, zvect2, "b:", linewidth = 4, label = "PI, $z_c=0.1$")
grid("on")


axis([0,Tend,-0.1,1])
xlabel("Time (s)", fontsize = 20)
ylabel("Ramp error", fontsize = 20)
legend(prop={'size':18})
savefig("../notes/fig/design-PI-resp2.pdf")


# Lag

D = poly1d([1,7,12,0])
sys = tf([1],[1,7,12,0])
rvect, kvect = rlocus(sys, klist = arange(0,60,0.01))
xvect0u = array([real(r[0]) for r in rvect])
yvect0u = array([imag(r[0]) for r in rvect])
xvect1u = array([real(r[1]) for r in rvect])
yvect1u = array([imag(r[1]) for r in rvect])
xvect2u = array([real(r[2]) for r in rvect])
yvect2u = array([imag(r[2]) for r in rvect])
zc = 0.2
pc = 0.05
D2 = D*poly1d([1,pc])
sys = tf([1,zc],D2.coeffs)
rvect, kvect = rlocus(sys, klist = arange(0,60,0.001))
xvect0 = array([real(r[0]) for r in rvect])
yvect0 = array([imag(r[0]) for r in rvect])
xvect1 = array([real(r[1]) for r in rvect])
yvect1 = array([imag(r[1]) for r in rvect])
xvect2 = array([real(r[2]) for r in rvect])
yvect2 = array([imag(r[2]) for r in rvect])
xvect3 = array([real(r[3]) for r in rvect])
yvect3 = array([imag(r[3]) for r in rvect])
close('all')
clf()
grid("on")
axes().set_aspect("equal")
axis([-5,1,-2.5,2.5])
plot([-100,100],[0,0],"k")
plot([0,0],[-100,100],"k")
plot(xvect0u,yvect0u,"m--", linewidth = 2, label = "RL (uncomp)")
plot(xvect1u,yvect1u,"m--", linewidth = 2)
plot(xvect2u,yvect2u,"m--", linewidth = 2)
plot(xvect0,yvect0,"m", linewidth = 3, label = "RL (lag)")
plot(xvect1,yvect1,"m", linewidth = 3)
plot(xvect2,yvect2,"m", linewidth = 3)
plot(xvect3,yvect3,"m", linewidth = 3)
plot(xvect1u[2000],yvect1u[2000],"hc",markersize=12,label = "K=20 (uncomp)")
plot(xvect2u[2000],yvect2u[2000],"hc",markersize=12)
plot(xvect1[2000*10],yvect1[2000*10],"sb",markersize=10,label = "K=20 (lag)")
plot(xvect2[2000*10],yvect2[2000*10],"sb",markersize=10)
plot(xvect0[0],yvect0[0],"xr",markeredgewidth=3,markersize=10,label = "Poles")
plot(xvect1[0],yvect1[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect2[0],yvect2[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[0],yvect3[0],"xr",markeredgewidth=3,markersize=10)
plot(xvect3[-1],yvect3[-1],"og",markersize=10, label = "Zero")
k = 500*5
quiver([xvect0[k]],[yvect0[k]],[xvect0[k+1]-xvect0[k]],[yvect0[k+1]-yvect0[k]],color="m",angles="xy",scale_units="xy")
k = 2000*5
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
quiver([xvect2[k]],[yvect2[k]],[xvect2[k+1]-xvect2[k]],[yvect2[k+1]-yvect2[k]],color="m",angles="xy",scale_units="xy")
k = 1100*5
quiver([xvect1[k]],[yvect1[k]],[xvect1[k+1]-xvect1[k]],[yvect1[k+1]-yvect1[k]],color="m",angles="xy",scale_units="xy")
k = 700*5
quiver([xvect2[k]],[yvect2[k]],array([xvect2[k+1]-xvect2[k]]),array([yvect2[k+1]-yvect2[k]]),color="m",angles="xy",scale_units="inches")
quiver([xvect3[k]],[yvect3[k]],array([xvect3[k+1]-xvect3[k]]),array([yvect3[k+1]-yvect3[k]]),color="m",angles="xy",scale_units="inches")
k = 1320*5
quiver([xvect2[k]],[yvect2[k]],array([xvect2[k+1]-xvect2[k]])/100,array([yvect2[k+1]-yvect2[k]])/100,color="m",angles="xy",scale_units="inches")
k = 1304*5
quiver([xvect3[k]],[yvect3[k]],array([xvect3[k+1]-xvect3[k]]),array([yvect3[k+1]-yvect3[k]]),color="m",angles="xy",scale_units="inches")
xlabel("Real", fontsize = 20)
ylabel("Imag", fontsize = 20)
legend(loc=2 , prop={'size':16})

savefig("../notes/fig/design-lag.pdf")

axis([-0.9,0.1,-0.5,0.5])
savefig("../notes/fig/design-lag-zoom.pdf")



########################## Frequency analysis ################################

# Simple response
T = 10
omega = 3
A = 1.3
phi = -1.4
trange = arange(0,T,0.01)
sin1 = [sin(omega*t) for t in trange]
sin2 = [A*sin(omega*t+phi) for t in trange]
clf()
plot(trange,sin1,"r",linewidth=2)
plot(trange,sin2,"b--",linewidth=2)
plot([0,T],[0,0],"k")
grid("on")
axis([0,T,-A*1.2,A*1.2])
savefig("../notes/fig/exoutput.pdf")


# Time response
sys = tf([1,0],[1,3,2])
T = arange(0,10,0.01)
R = [sin(3*t) for t in T]
C,_,_ = lsim(sys,R,T)
clf()
grid("on")
plot(T,R,"r",linewidth=2,label="Input")
plot(T,C,"b--",linewidth=2,label="Response")
legend(loc=4 , prop={'size':18})
xlabel("Time (s)", fontsize = 20)
ylabel("Signal", fontsize = 20)
axis([T[0],T[-1],-1.1,1.1])
savefig("../notes/fig/sine-resp.pdf")

T = arange(0,20,0.01)
R = [2*sin(3*t)+cos(5*t+2) for t in T]
C,_,_ = lsim(sys,R,T)
clf()
grid("on")
plot(T,R,"r",linewidth=2,label="Input")
plot(T,C,"b--",linewidth=2,label="Response")
legend(loc=4 , prop={'size':18})
xlabel("Time (s)", fontsize = 20)
ylabel("Signal", fontsize = 20)
axis([T[0],T[-1],-3.5,3.5])
savefig("../notes/fig/sine-resp2.pdf")

# Bode plots
sys = tf([1,0],[1,2])
freq = logspace(-5,5,500)
amp, ang, _ = bode(sys,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp,'r',linewidth=2)
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
axis([freq[0],freq[-1],-120,10])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang,'r',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([0,pi/4,pi/2],["$0$","$\pi/4$","$\pi/2$"])
axis([freq[0],freq[-1],-0.1,pi/2+0.1])
tight_layout()
savefig("../notes/fig/exbode.pdf")

# Bode const
freq = logspace(-5,5,500)
amp1 = array([20 for f in freq])
amp2 = array([-40 for f in freq])
ang = array([0 for f in freq])
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="$G(s)=10$")
plot(freq,amp2,'g--',linewidth=2,label="$G(s)=0.01$")
legend(prop={'size':14})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-100,120,20))
axis([freq[0],freq[-1],-100,100])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang,'r',linewidth=2)
plot(freq,ang,'g--',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi/2,0,pi/2],["$-\pi/2$","$0$","$\pi/2$"])
axis([freq[0],freq[-1],-pi/2-0.5,pi/2+0.5])
tight_layout()
savefig("../notes/fig/bode-const.pdf")

# Bode D-I
freq = logspace(-5,5,500)
amp1 = array([20*log10(f) for f in freq])
amp2 = array([-20*log10(f) for f in freq])
ang1 = array([pi/2 for f in freq])
ang2 = array([-pi/2 for f in freq])
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="$G(s)=s$")
plot(freq,amp2,'g--',linewidth=2,label="$G(s)=1/s$")
legend(bbox_to_anchor=(0., 1.02, 1., -.3),prop={'size':14})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-100,120,20))
axis([freq[0],freq[-1],-100,100])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
plot(freq,ang2,'g--',linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi/2,0,pi/2],["$-\pi/2$","$0$","$\pi/2$"])
axis([freq[0],freq[-1],-pi/2-0.5,pi/2+0.5])
tight_layout()
savefig("../notes/fig/bode-di.pdf")

# Ex slope/intercept
freq = logspace(-2,2,500)
amp1 = array([40+20*log10(f) for f in freq])
figure(0)
clf()
plot(freq,amp1,'r',linewidth=2,label="$G(s)=s$")
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-100,120,20))
axis([freq[0],freq[-1],-20,100])
tight_layout()
savefig("../notes/fig/bode-slope-intercept.pdf")



# Bode first order
a = 0.1
freq = logspace(-5,5,500)
sys1 = tf([1,a],[1])
sys2 = tf([1],[1,a])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="$G(s)=s+0.1$")
#plot(freq,amp2,'g--',linewidth=2,label="$G(s)=\\frac{1}{s+0.1}$")
plot([a,a],[-1000,1000],"k-.",linewidth=2)
#legend(bbox_to_anchor=(0., 1.02, 1., -.3),prop={'size':14})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-100,120,20))
axis([freq[0],freq[-1],-100,100])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
#plot(freq,ang2,'g--',linewidth=2)
plot([a,a],[-10,10],"k-.",linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi/2,0,pi/2],["$-\pi/2$","$0$","$\pi/2$"])
axis([freq[0],freq[-1],-pi/2-0.5,pi/2+0.5])
tight_layout()
#savefig("../notes/fig/bode-first.pdf")
savefig("../notes/fig/bode-first2.pdf")


# Bode second order
omegan = 10
zeta1 = 0.1
zeta2 = 0.8
freq = logspace(-5,5,500)
sys1 = tf([omegan*omegan],[1,2*zeta1*omegan,omegan*omegan])
sys2 = tf([omegan*omegan],[1,2*zeta2*omegan,omegan*omegan])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="$\omega_n=10,\ \zeta=0.1$")
plot(freq,amp2,'g--',linewidth=2,label="$\omega_n=10,\ \zeta=0.8$")
plot([omegan,omegan],[-1000,1000],"k-.",linewidth=2)
legend(loc=3,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-100,40])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
plot(freq,ang2,'g--',linewidth=2)
plot([omegan,omegan],[-10,10],"k-.",linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi,-pi/2,0],["$-\pi$","$-\pi/2$","$0$"])
axis([freq[0],freq[-1],-pi-0.5,0.5])
tight_layout()
savefig("../notes/fig/bode-second.pdf")


# Bode comlex
freq = logspace(-4,4,2000)
sys1 = tf([-5,-0.5],[1,0.1,49,0,0])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)

lognd = 20*log10(0.5/49)
logA = 20*log10(5)

omega1 = []
amps1 = []
angs1 = []
for expo in [-2,-1.5,-1.15,-1.1,-1.05,-1,-0.95,-0.9,-0.85,-0.5, 0,0.5,0.7,0.75,0.8,0.85,0.9,0.95,1,1.5,1.85]:
    omega = 10**expo
    jo = omega*1j
    Gjo = (-5*jo-0.5)/(jo**4+0.1*jo**3+49*jo**2)
    omega1.append(omega)
    amps1.append(20*log10(norm(Gjo)))
    angs1.append(angle(Gjo))

# Full
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,color=[0.75,0.75,0.75],linewidth=6,label="Exact plot")
K = -4
plot([1,10**K],[lognd,lognd-40*K],"b--",linewidth=2,label="Asymptotes")
K = 4
plot([1,10**K],[logA,logA-60*K],"b--",linewidth=2)
plot(omega1,amps1,'go',markersize=5,label="Sampled points")
plot(omega1,amps1,'g',linewidth=2)
legend(loc=3,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-200,120,20))
axis([freq[0],freq[-1],-200,100])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,color=[0.75,0.75,0.75],linewidth=6)
plot(omega1,angs1,'go',markersize=5)
plot(omega1,angs1,'g',linewidth=2)
K = -4
plot([1,10**K],[0,0],"b--",linewidth=2)
K = 4
plot([1,10**K],[-pi/2,-pi/2],"b--",linewidth=2)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-pi/2,0,pi/2],["$-\pi/2$","0","$\pi/2$"])
axis([freq[0],freq[-1],-pi/2-0.5,pi/2+0.5])
tight_layout()
savefig("../notes/fig/bode-complex.pdf")

# Step by step
figure(0)
clf()
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-200,120,20))
axis([freq[0],freq[-1],-200,100])
grid("on")
K = -4
plot([1,10**K],[lognd,lognd-40*K],"b--",linewidth=2,label="Asymptotes")
savefig("../notes/fig/bode-complex-amp1.pdf")
K = 4
plot([1,10**K],[logA,logA-60*K],"b--",linewidth=2)
savefig("../notes/fig/bode-complex-amp2.pdf")
plot(omega1,amps1,'go',markersize=5,label="Sampled points")
plot(omega1,amps1,'g',linewidth=2)
savefig("../notes/fig/bode-complex-amp3.pdf")

figure(1)
yticks([-pi/2,0,pi/2],["$-\pi/2$","0","$\pi/2$"])
axis([freq[0],freq[-1],-pi/2-0.5,pi/2+0.5])
ylabel("Phase shift (rad)", fontsize = 20)
xscale('log')
grid("on")
K = -4
plot([1,10**K],[0,0],"b--",linewidth=2)
savefig("../notes/fig/bode-complex-ang1.pdf")
K = 4
plot([1,10**K],[-pi/2,-pi/2],"b--",linewidth=2)
savefig("../notes/fig/bode-complex-ang2.pdf")
plot(omega1,angs1,'go',markersize=5)
plot(omega1,angs1,'g',linewidth=2)
savefig("../notes/fig/bode-complex-ang3.pdf")

# Reverse engineering
figure(0)
clf()
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-200,120,20))
axis([freq[0],freq[-1],-200,100])
grid("on")
plot(freq,amp1,color=[0.6,0.6,0.6],linewidth=4)
savefig("../notes/fig/bode-complex-amp-0.pdf")
K = -4
plot([1,10**K],[lognd,lognd-40*K],"b--",linewidth=2,label="Asymptotes")
savefig("../notes/fig/bode-complex-fit1.pdf")
K = 4
plot([1,10**K],[logA,logA-60*K],"b--",linewidth=2)
savefig("../notes/fig/bode-complex-fit2.pdf")


# sampling
for expo in [-2,-1.5,-1,-0.5,0,0.5,1,1.5,1.85]:
    omega = 10**expo
    jo = omega*1j
    Gjo = (-5*jo-0.5)/(jo**4+0.1*jo**3+49*jo**2)
    omega1.append(omega)
    amps1.append(20*log10(norm(Gjo)))
    angs1.append(angle(Gjo))
    print expo, "&", round(omega,2), "&", round(20*log10(norm(Gjo)),2), "&", round(angle(Gjo),2), "\\\\"

for expo in [-1.15,-1.1,-1.05,-1,-0.95,-0.9,-0.85]:
    omega = 10**expo
    jo = omega*1j
    Gjo = (-5*jo-0.5)/(jo**4+0.1*jo**3+49*jo**2)
    omega2.append(omega)
    amps2.append(20*log10(norm(Gjo)))
    angs2.append(angle(Gjo))
    print expo, "&", round(omega,2), "&", round(20*log10(norm(Gjo)),2), "&", round(angle(Gjo),2), "\\\\"

for expo in [0.7,0.75,0.8,0.85,0.9,0.95,1]:
    omega = 10**expo
    jo = omega*1j
    Gjo = (-5*jo-0.5)/(jo**4+0.1*jo**3+49*jo**2)
    omega3.append(omega)
    amps3.append(20*log10(norm(Gjo)))
    angs3.append(angle(Gjo))
    print expo, "&", round(omega,2), "&", round(20*log10(norm(Gjo)),2), "&", round(angle(Gjo),2), "\\\\"


# Bode type 0
freq = logspace(-5,5,500)
sys1 = tf([4,0.04],[1,1,100])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
lognd = 20*log10(0.04/100)
logA = 20*log10(4)
figure(0)
clf()
plot(freq,amp1,'r',linewidth=2,label="Gain plot")
K1,K2 = -5 , 1
plot([10**K1,10**K2],[lognd,lognd],"g--",linewidth=2,label="Fitted left asymptote")
K1,K2 = -1, 5
plot([10**K1,10**K2],[logA-20*K1,logA-20*K2],"b-.",linewidth=3,label="Fitted right asymptote")
legend(loc=2,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-100,20])
tight_layout()
savefig("../notes/fig/bode-type0.pdf")


# Bode type 1
freq = logspace(-5,5,500)
sys1 = tf([30,10,1,1],[1,1,20,0])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
lognd = 20*log10(1/20.)
logA = 20*log10(30)
figure(0)
clf()
plot(freq,amp1,'r',linewidth=2,label="Gain plot")
K1,K2 = -5 , 1
plot([10**K1,10**K2],[lognd-20*K1,lognd-20*K2],"g--",linewidth=2,label="Fitted left asymptote")
K1,K2 = -1, 5
plot([10**K1,10**K2],[logA,logA],"b-.",linewidth=3,label="Fitted right asymptote")
legend(prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,90,20))
axis([freq[0],freq[-1],-50,80])
tight_layout()
savefig("../notes/fig/bode-type1.pdf")


# Bode Nyquist criterion 
# 200/(s+2)(s+4)(s+5) = 200/(s^3+11s^2+38s+40)
# 500/(s+2)(s+4)(s+5) = 500/(s^3+11s^2+38s+40)
freq = logspace(-5,5,500)
sys1 = tf([200],[1,11,38,40])
sys2 = tf([600],[1,11,38,40])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="200/[(s+2)(s+4)(s+5)]")
plot(freq,amp2,'g--',linewidth=2,label="600/[(s+2)(s+4)(s+5)]")
plot([6.16,6.16],[-1000,1000],'k',linewidth=1)
plot([freq[0],freq[-1]],[4,4],'g',linewidth=1)
plot([freq[0],freq[-1]],[-5.5,-5.5],'r',linewidth=1)
plot([6.16],[-5.5],"ro",markersize=5)
plot([6.16],[4],"go",markersize=5)
legend(loc=3,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-100,40])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
plot(freq,ang2,'g--',linewidth=2)
plot([freq[0],freq[-1]],[-pi,-pi],'k',linewidth=1)
plot([6.16,6.16],[-100,100],'k',linewidth=1)
plot([6.16],[-pi],"ko",markersize=5)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-3*pi/2,-pi,-pi/2,0],["$-3\pi/2$","$-\pi$","$-\pi/2$","$0$"])
axis([freq[0],freq[-1],-3*pi/2-0.5,0.5])
tight_layout()
savefig("../notes/fig/bode-nyq.pdf")


# Bode Nyquist criterion 
# 200/(s+2)(s+4)(s+5) = 200/(s^3+11s^2+38s+40)
# 500/(s+2)(s+4)(s+5) = 500/(s^3+11s^2+38s+40)
freq = logspace(-5,5,500)
sys1 = tf([200],[1,11,38,40])
sys2 = tf([600],[1,11,38,40])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="200/[(s+2)(s+4)(s+5)]")
plot(freq,amp2,'g--',linewidth=2,label="600/[(s+2)(s+4)(s+5)]")
plot([6.16,6.16],[-1000,1000],'k',linewidth=1)
plot([freq[0],freq[-1]],[4,4],'g',linewidth=1)
plot([freq[0],freq[-1]],[-5.5,-5.5],'r',linewidth=1)
plot([6.16],[-5.5],"ro",markersize=5)
plot([6.16],[4],"go",markersize=5)
legend(loc=3,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-100,40])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
plot(freq,ang2,'g--',linewidth=2)
plot([freq[0],freq[-1]],[-pi,-pi],'k',linewidth=1)
plot([6.16,6.16],[-100,100],'k',linewidth=1)
plot([6.16],[-pi],"ko",markersize=5)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-3*pi/2,-pi,-pi/2,0],["$-3\pi/2$","$-\pi$","$-\pi/2$","$0$"])
axis([freq[0],freq[-1],-3*pi/2-0.5,0.5])
tight_layout()
savefig("../notes/fig/bode-nyq.pdf")

# Bode Nyquist criterion (phase)
# 200/(s+2)(s+4)(s+5) = 200/(s^3+11s^2+38s+40)
# 500/(s+2)(s+4)(s+5) = 500/(s^3+11s^2+38s+40)
freq = logspace(-5,5,500)
sys1 = tf([200],[1,11,38,40])
sys2 = tf([600],[1,11,38,40])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="200/[(s+2)(s+4)(s+5)]")
plot(freq,amp2,'g--',linewidth=2,label="600/[(s+2)(s+4)(s+5)]")
plot([7.5,7.5],[-1000,1000],'g',linewidth=1)
plot([4.5,4.5],[-1000,1000],'r',linewidth=1)
plot([freq[0],freq[-1]],[0,0],'k',linewidth=1)
legend(loc=3,prop={'size':16})
plot([4.5],[0],"ro",markersize=5)
plot([7.5],[0],"go",markersize=5)
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-100,40])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
plot(freq,ang2,'g--',linewidth=2)
plot([7.5,7.5],[-1000,1000],'g',linewidth=1)
plot([4.5,4.5],[-1000,1000],'r',linewidth=1)
plot([freq[0],freq[-1]],[-2.7,-2.7],'r',linewidth=1)
plot([freq[0],freq[-1]],[-3.35,-3.35],'g',linewidth=1)
plot([4.5],[-2.7],"ro",markersize=5)
plot([7.5],[-3.35],"go",markersize=5)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-3*pi/2,-pi,-pi/2,0],["$-3\pi/2$","$-\pi$","$-\pi/2$","$0$"])
axis([freq[0],freq[-1],-3*pi/2-0.5,0.5])
tight_layout()
savefig("../notes/fig/bode-nyq2.pdf")


# Gain margin 
# 200/(s+2)(s+4)(s+5) = 200/(s^3+11s^2+38s+40)
freq = logspace(-1,2,500)
sys1 = tf([200],[1,11,38,40])
sys2 = tf([600],[1,11,38,40])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="200/[(s+2)(s+4)(s+5)]")
#plot(freq,amp2,'g--',linewidth=2,label="600/[(s+2)(s+4)(s+5)]")
plot([6.16,6.16],[-1000,1000],'k',linewidth=1)
#plot([freq[0],freq[-1]],[4,4],'g',linewidth=1)
plot([freq[0],freq[-1]],[-5.5,-5.5],'k',linewidth=1)
plot([6.16],[-5.5],"ro",markersize=5)
#plot([6.16],[4],"go",markersize=5)
#legend(loc=3,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-20,20])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
#plot(freq,ang2,'g--',linewidth=2)
plot([freq[0],freq[-1]],[-pi,-pi],'k',linewidth=1)
plot([6.16,6.16],[-100,100],'k',linewidth=1)
plot([6.16],[-pi],"ro",markersize=5)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-3*pi/2,-pi,-pi/2,0],["$-3\pi/2$","$-\pi$","$-\pi/2$","$0$"])
axis([freq[0],freq[-1],-3*pi/2-0.5,0.5])
tight_layout()
savefig("../notes/fig/bode-gain-margin.pdf")


# Gain margin 
# 200/(s+2)(s+4)(s+5) = 200/(s^3+11s^2+38s+40)
freq = logspace(-1,2,500)
sys1 = tf([200],[1,11,38,40])
sys2 = tf([600],[1,11,38,40])
amp1, ang1, _ = bode(sys1,freq,Plot=False,dB=True,deg=False)
amp2, ang2, _ = bode(sys2,freq,Plot=False,dB=True,deg=False)
figure(0)
clf()
ax1 = subplot(211) 
plot(freq,amp1,'r',linewidth=2,label="200/[(s+2)(s+4)(s+5)]")
#plot(freq,amp2,'g--',linewidth=2,label="600/[(s+2)(s+4)(s+5)]")
plot([4.5,4.5],[-1000,1000],'k',linewidth=1)
#plot([freq[0],freq[-1]],[4,4],'g',linewidth=1)
plot([freq[0],freq[-1]],[0,0],'k',linewidth=1)
plot([4.5],[0],"ro",markersize=5)
#plot([6.16],[4],"go",markersize=5)
#legend(loc=3,prop={'size':16})
grid("on")
xscale('log')
ylabel("Gain (dB)", fontsize = 20)
yticks(arange(-140,70,20))
axis([freq[0],freq[-1],-20,20])
ax2 = subplot(212,sharex=ax1)
plot(freq,ang1,'r',linewidth=2)
#plot(freq,ang2,'g--',linewidth=2)
plot([freq[0],freq[-1]],[-2.7,-2.7],'k',linewidth=1)
plot([4.5,4.5],[-100,100],'k',linewidth=1)
plot([4.5],[-2.7],"ro",markersize=5)
grid("on")
ylabel("Phase shift (rad)", fontsize = 20)
xlabel("Frequency (rad/s)", fontsize = 20)
yticks([-3*pi/2,-pi,-pi/2,0],["$-3\pi/2$","$-\pi$","$-\pi/2$","$0$"])
axis([freq[0],freq[-1],-3*pi/2-0.5,0.5])
tight_layout()
savefig("../notes/fig/bode-phase-margin.pdf")

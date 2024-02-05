clear all
close all

syms s
syms c1 c2 c3 l1 l2 l3 r1 r2 positive

%Modeling System
Z1 = 1/(c1*s) + s*l1;
Z2 = r1;
Z3 = r2 + s*l2;
Z4 = 1/(c2*s);
Z5 = l3*s + 1/(c3*s);

Z12 = simplify(1/(1/Z1+1/Z2));
Z345 = simplify(1/(1/Z3+1/Z4+1/Z5));
Zout = simplify(Z345/(Z12+Z345));

Gsym = Zout/Z3;
G = (subs(Gsym,{c1,c2,c3,l1,l2,l3,r1,r2},{.01,.005,0.001,.001,.05,.01,100,100}));

%Creating ol and cl state-space, zpk, and  tf forms
[num,den] = numden(G);
num = sym2poly(num);
den = sym2poly(den);
sys = tf(num,den)
sys_cl = feedback(sys,1);
[A,B,C,D] = tf2ss(num,den);
[Z,P,gain]  = ss2zp(A,B,C,D);
[num_cl, den_cl] = tfdata(sys_cl,'v');
[A2,B2,C2,D2] = tf2ss(num_cl,den_cl);
[Z2,P2,gain2]  = ss2zp(A2,B2,C2,D2);

%%
%laplace domain controller
K = zpk([-1.33+566j, -1.33-566j],[-1, -4000],1);


%Frequency domain controller
K_lag = tf([1/1000, 1], [1 1]);

[num_controller, den_controller] =  tfdata(K,'v');

%%

%plot ol and cl system
figure
step(sys)
figure
step(sys_cl);
figure
nyquist(sys_cl)
figure
margin(sys)
info = stepinfo(sys)

%%
%plot laplace domain controller
sys2 = 1000*K*sys;
sys2_cl = feedback(sys2,1);
figure
step(sys2_cl)
figure
nyquist(sys2_cl)
info2 = stepinfo(sys2_cl)
figure
margin(sys2)
%%
%plot frequency domain controller
sys3 = 50*K_lag*K*sys;
sys3_cl = feedback(sys3,1);
figure
step(sys3_cl)
info3 = stepinfo(sys3_cl)
figure
margin(sys3)

%%
% Closed loop system
dist = 0;
noise = 0;
sinwave = 0;
ref = 1;
runtime = 0.05;
k = 1000;

sim("Feedback_Control.slx")
figure, hold on
plot(t,yout)
plot(t,reference)
title("Closed Loop Step Response")
xlabel("Time (s)")
ylabel("Amplitude")
legend("yout","reference")
hold off

%plot error

e = abs((yout-reference)/reference*100);
figure
plot(t,e)
title("Error in closed loop system")
xlabel("Time (s)")
ylabel("Percent Error")
%%
% constant Disturbance 
close all
dist = 1;
noise = 0;
sinwave = 0;
ref = 1;
runtime = 0.05;
k= 1000;

sim("Feedback_Control.slx")
figure, hold on
plot(t,yout)
plot(t,reference)
title("Closed Loop Step Response with constant disturbance")
xlabel("Time (s)")
ylabel("Amplitude")
legend("yout","reference")
hold off

%plot error

e = abs((yout-reference)/reference*100);
figure
plot(t,e)
title("Error with constant disturbance")
xlabel("Time (s)")
ylabel("Percent Error")
%%
% Sinwave Disturbance 
close all
dist = 1;
noise = 0;
sinwave = 0;
ref = 1;
runtime = 0.05;
k= 1000;

sim("Feedback_Control.slx")
figure, hold on
plot(t,yout)
plot(t,reference)
title("Closed Loop Step Response with sin disturbance")
xlabel("Time (s)")
ylabel("Amplitude")
legend("yout","reference")
hold off

%plot error

e = abs((yout-reference)/reference*100);
figure
plot(t,e)
title("Error with sin disturbance")
xlabel("Time (s)")
ylabel("Percent Error")

%%
% Measurement Noise
dist = 0;
noise = 1;
sinwave = 0;
ref = 1;
runtime = 0.05;
k = 2500;

sim("Feedback_Control.slx")
figure, hold on
plot(t,yout)
plot(t,reference)
title("Closed Loop Step Response with measurement noise")
xlabel("Time (s)")
ylabel("Amplitude")
legend("yout","reference")
hold off

%plot error
e = abs((yout-reference)/reference*100);
figure
plot(t,e,color="#0072BD")
title("Error with measurement noise ")
xlabel("Time (s)")
ylabel("Percent Error")

%%
% Sin Reference
dist = 0;
noise = 0;
sinwave = 0;
ref = 0;
runtime = 3;
k=1000;

sim("Feedback_Control.slx")
figure, hold on
plot(t,yout)
plot(t,reference,'r--')
title("Closed Loop Step Response with sin reference")
xlabel("Time (s)")
ylabel("Amplitude")
legend("yout","reference")
hold off

%plot error

e = abs((yout-reference)*100);
figure
plot(t,e)
title("Error with sin reference")
xlabel("Time (s)")
ylabel("Percent Error")

%%
syms r y K_sym Gd d G_sym n e s1 s2

eqn_dy = [r-y==s1, K_sym*s1 - Gd*d == s2, y == G_sym*s2];
eqn_ne = [r-y,K_sym*G_sym*e - n];
S = solve(eqn_dy,[s1,s2,y])
S2 = solve(eqn_ne,[e,y])

%%
G_d = sys;
G_dy = sys*(G_d+K)/(1+K*sys);
figure 
bode(G_dy)

G_ne = 1/(K*sys);
figure
bode(G_ne)




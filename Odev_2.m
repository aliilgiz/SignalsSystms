%% 1.soru  
clc
clear all
close all
%a þýkký
fs = 100000;
t = 0:1/fs:1.5;
x1 = 2*sawtooth(2*pi*100*t);
subplot(1,2,1)
plot(t,x1)
axis([0 0.2 -2.2 2.2])
xlabel('Time (sec)')
ylabel('Amplitude') 
title('Sawtooth Periodic Wave')
%b þýkký
fs = 1000000;
x2 = square(2*pi*20*t);
subplot(1,2,2)
plot(t,x2)
axis([0 0.2 -1.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude')
title('Square Periodic Wave')
%c þýkký
fs = 100000;
t = -1:1/fs:1;
x1 = tripuls(t,100e-3);
subplot(2,1,1)
plot(t,x1)
axis([-0.1 0.1 -0.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude')
title('Triangular Aperiodic Pulse')
%d þýkký
fs = 10000;
x2 = rectpuls(t,50e-3);
subplot(2,1,2)
plot(t,x2)
axis([-0.1 0.1 -0.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude')
title('Rectangular Aperiodic Pulse')
%e þýkký
tc = gauspuls('cutoff',50e3,0.5,[],-60); 
t1 = -tc : 1e-6 : tc; 
y1 = gauspuls(t1,50e3,0.6);
plot(t1*1e3,y1)
xlabel('Time (ms)')
ylabel('Amplitude')
title('Gaussian Pulse')
%f þýkký
fs = 200E9;                   
D = [2.5 10 17.5]' * 1e-9;   
t = 0 : 1/fs : 2500/fs;       
w = 2e-9;                     
yp = pulstran(t,D,@rectpuls,w);
subplot(2,1,1)
plot(t*1e9,yp);
axis([0 25 -0.2 1.2])
xlabel('Time (ns)')
ylabel('Amplitude')
title('Rectangular Train')
%g þýkký
T = 0 : 1/50e3 : 10e-3;
D = [0 : 1/1e3 : 10e-3 ; 0.6.^(0:10)]';
Y = pulstran(T,D,@gauspuls,10E3,.5);
plot(T*1e3,Y)
xlabel('Time (ms)')
ylabel('Amplitude')
title('Gaussian Pulse Train')
%% 2.soru
clear all;
close all;
clc;
%CT unit impulse
N=15;
n=-N:1:N;
y=[zeros(1,N),ones(1,1),zeros(1,N)];
subplot(2,1,1);
plot(n,y);
ylabel('amplitude');
xlabel('time--->');
title('unit impulse DT signal');
display(y);
%DT unit impulse
subplot(2,1,2);
stem(n,y);
ylabel('amplitude');
xlabel('number of samples--->');
title('unit impulse DT signal');
display(y);
%CT Unit ramp
t = (-30:1:30)';
ramp = t.*unitstep;
subplot(6,1,5);
plot(t,ramp,'r','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('unit ramp DT signal');
display(ramp);
%DT Unit ramp
subplot(6,1,6);
stem(t,ramp,'r','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('unit ramp DT signal');
display(ramp);  
%CT Unit Step
t = (-30:1:30)';
unitstep = zeros(size(t)); 
unitstep(t>=1) = 1; 
subplot(6,1,3);
plot(t,unitstep,'g','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('unit step CT signal');
display(unitstep);
%DT Unit Step
subplot(6,1,4);
stem(t,unitstep,'g','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('unit step DT signal');
display(unitstep);

%% 3.soru
%3-1
fs=1000 %fs=100 hz
f=1 %1 Hz
t=0:1/fs:1
y1=sin(2*pi*f*t)
subplot(3,1,1)
plot(t,y1,'LineWidth',2)
title('x[n] sinyalinin DT Sin grafiði')
grid on
y2=cos(2*pi*f*t)
subplot(3,1,2)
plot(t,y2,'LineWidth',2)
title('x[n] sinyalinin DT Cos grafiði')
grid on
%3-2
n=-50:1:50
a=cos(2*pi*n/36)
b=sin(2*pi*n/36)
subplot(3,1,3)
stem(a,b)
title('x[n] sinyalinin CT grafiði')
%% 4.soru
x=t.*(t.^2+3)
t=-0:10:100;
g_even=(g(x)+g(-x))/2
g_odd=(g(x)-g(-x))/2
plot(x,g_odd)
title('g(t) Sinyalinin Tek-Çift Grafiði')
%% 5.soru
n=-100:100
y=(0.9.^abs(n)).*sin(2*pi*n/4)
sum(abs(y.^2))
title('x[n] Sinyalinin Enerjisi')
%% 6.soru
f=1000 %1 kHz
fs=2000 %fs=2 kHz
t=0:1/fs:5
y1=sin(2*pi*f*t)
figure
subplot(2,1,1)
plot(t,y1,'LineWidth',1)
title('CT Signal')
fs1=50000
t_samp1=0:1/10:5
y2=sin(2*pi*f*t_samp1)
subplot(2,1,2)
stem(t_samp1,y2)
title('DT Signal')

f=1000  %1 kHz
fs=2000 %fs=2 kHz
t=0:1/fs:5
y1=cos(2*pi*f*t)
figure
subplot(2,1,1)
plot(t,y1,'LineWidth',1)
title('CT Signal')
fs1=50000
t_samp1=0:1/10:5
y2=cos(2*pi*f*t_samp1)
subplot(2,1,2)
stem(t_samp1,y2)
title('DT Signal')
%% 7.soru
n = 0:100;
x =4+cos(2*pi*n/24);
x0 = downsample(x,2,0);
x1 = downsample(x,2,1);
figure
stem(x)
title('Original Signal')
xlabel('4+cos(2*pi*n/24)')
ylabel('Cos Signal')
%n=2 için
n = 0:100;
x = 4+cos(2*pi*n/24);
x0 = downsample(x,2,0);
subplot(3,1,1)
stem(x)
title('Original Signal')

y0 = upsample(x0,2,0);
subplot(3,1,2)
stem(y0)
title('N=2 Signal')
xlabel('4+cos(2*pi*n/24)')
ylabel('Cos Signal')
%n=10 için
n = 0:100;
x = 4+cos(2*pi*n/24);
x0 = downsample(x,10,0);
subplot(3,1,1)
stem(x)
ylim([0.5 3.5])
title('Original Signal')

y0 = upsample(x0,10,0);
subplot(3,1,2)
stem(y0)
ylabel('N=10 Signal')
xlabel('4+cos(2*pi*n/24)')
ylabel('Cos Signal')

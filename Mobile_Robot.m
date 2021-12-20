
close all
clear all

load datareference.mat xr yr phr wr
%load agv_ref_junction

%system dynamics
hagv= 0.200;    %width of robot, m
wagv= 0.250;    %length of robot, m
b= 0.180;    %wheel distance, m
r= 0.065/2;    %wheel radius, m
d= 0.080;    %mobile robot center to robot's tracking sensor, m
bl= 0.026;   %line width

vr = 1.5;
%wr = vr/0.5;

%sampling time and data time
dt=0.02;
dt1 = 0.005;
t(1)=0;
n= length(xr);
m = round(dt/dt1);

PWM = [0 0];
%Transfer function
%T = k11 / (s+k12)
k11 = 229.9; k21 = 5.931*10^-6;
K_tf1 = [k11 k21];
k12 = 247.4; k22 = 0.000104;
K_tf2 = [k12 k22];

%PID Parameter
Kp1 = 0.3; Ki1 = 0.3626; Kd1 = 0;
Kp2 = 0.2683; Ki2 = 0.3117; Kd2 = 0;
Kp = [Kp1 Kp2];
Ki = [Ki1 Ki2];
Kd = [Kd1 Kd2];


%Fuzzy Controller
f = readfis('Fuzzy_Controller.fis');
u(1) = 0;

% tracking point
xc(1)=xr(1)-0.005;
yc(1)=yr(1)-0.005;
phc(1)=phr(1)+10*pi/180;
dxc(1) = 0;
dyc(1) = 0;
dphc(1) = 0;
% Robot center
ph(1)=phc(1);
w(1)=0;
v(1)=0;
w1(1) = 0;
w2(1) = 0;
x(1)=xc(1)-d*cos(ph(1));
y(1)=yc(1)-d*sin(ph(1));
%e(1) = SS(ref(xc(1,m),yc(1,m),phic(1,m)));


%=== for error of tracking ===
E0=[cos(ph(1)) sin(ph(1)) 0;
    -sin(ph(1)) cos(ph(1)) 0;
              0         0    1] *[xr(1)-xc(1);
                                 yr(1)-yc(1);
                                 phr(1)-phc(1)];
e1(1)=E0(1);
e2(1)=E0(2);
e3(1)=E0(3);
e(1)=e2(1);
%=== for wheel dynamics ===
vwl(1)=0;
vwr(1)=0;


%Main loop
for i=2:n
   t(i)=(i-1)*dt;
   
   %Tracking point
   dxc(i) = v(i-1)*cos(ph(i-1)) - w(i-1)*d*sin(ph(i-1));
   dyc(i) = v(i-1)*sin(ph(i-1)) + w(i-1)*d*cos(ph(i-1));
   dphc(i) = w(i-1);
   xc(i) = dxc(i-1)*dt + xc(i-1);
   yc(i) = dyc(i-1)*dt + yc(i-1);
   phc(i) = dphc(i-1)*dt + phc(i-1);
   ph(i) = phc(i);
   
   
   %error dynamics
%    e(i) = SS(ref(xc(i-1,end), yc(i-1,end), phic(i-1,end)));
%    errREF(i) = 0;
%    e(i) = - e(i);
%    de(i) = e(i) - e(i-1);
   Ae=[cos(ph(i))  sin(ph(i))  0;  -sin(ph(i))  cos(ph(i))  0;  0  0  1];
   E = Ae*[xr(i)-xc(i);yr(i)-yc(i);phr(i)-phc(i)];
   e1(i) = 0;
   e2(i) = E(2);
   e3(i) = E(3);
   
   
%    E=([-1;0;0]*v(i-1) + [sin(e2(i-1));-d-e1(i-1);-1]*w(i-1) + [vr*cos(e2(i-1)); vr*e3(i-1); wr(i-1)])*dt+E0;
%    e1(i)= 0; % = 0 
%    e2(i)=E(2);
%    e3(i)=E(3);
%    E0=E;
    
   %controller dynamics
   %w(i)=wr(i)+k2*vr*e2(i)+k3*sin(e3(i));
   %v(i)=vr*cos(e3(i))+k1*e1(i);
   %Kp = 250; Ki = -3; Kd = 0.87;
   v(i) = vr;
   e(i) = e2(i) ;
   de(i) = (e(i) - e(i-1));
   
   %Limit e and de range
   if e(i) > 0.005
       ef(i) = 0.005;
   elseif e(i) < -0.005
       ef(i) = -0.005;
   else
       ef(i) = e(i);
   end
   
   if de(i) > 0.0025
       def(i) = 0.0025;
   elseif de(i) < -0.0025
       def(i) = -0.0025;
   else
       def(i) = de(i);
   end
   
%    Wf(i,:) = (evalfis([ef(i) def(i)],f)) %rad/s
%    wRm(i, :) = Wf(i,:)*9.6*60/(2*pi);  %Wheel dynamics
   
%    %PWM
%    Esum = [0 0];
%    xc(i,1) = xc(i-1,end);
%    yc(i,1) = yc(i-1,end);
%    phic(i,1) = phic(i-1,end);
%    Wm(1,:) = Wm(end,:);
%    Pwm(1,:) = PWM(end,:);
%    v(1) = v(end);
%    w(1) = w(end);
%    
%    %Response of motor
%    
%    for j = 2:m
%        Wm_dot(j,:) = K_tf1.*Pwm(j-1,:) - K_tf2.*Wm(j-1,:);
%        Wm(j,:) = Wm_dot(j,:)*dt1 + Wm(j-1,:);
%        Ew(j,:) = wRm(i,:) - Wm(j,:);
%        Esum = Esum + Ew(j,:);       
%        Pwm(j,:) = Kp.*Ew(j,:) + Ki.*Esum*dt1 + Kd.*(Ew(j,:)-Ew(j-1,:));
%            if Pwm(j,1) > 255
%                Pwm(j,1) = 255;
%            elseif Pwm(j,1) < 0
%                Pwm(j,1) = 0;
%            end
%            if Pwm(j,2) > 255
%                Pwm(j,2) = 255;
%            elseif Pwm(j,2) < 0
%                Pwm(j,2) = 0;
%            end
%            
%          PWM = [PWM; Pwm(j,:)];  
%          Wm(j,:) = Wm(j,:)*(2*pi)/60/9.6;   %rad/s
%          if xc(i,j-1) > 1000 || xc(i,j-1) < -1000
%               w(j) = (r/b)*(Wm(j,2) - Wm(j,1)) + wr;            
%          else
%               w(j) = (r/b)*(Wm(j,2) - Wm(j,1));            
%          end
%          v(j) = (Wm(j,1)+Wm(j,2))*r/2;
%          v_ref(i) = vr;
%    % Tracking point
%         dxc(i,j) = v(j-1)*cos(phic(i,j-1)) - w(j-1)*d*sin(phic(i,j-1));
%         dyc(i,j) = v(j-1)*sin(phic(i,j-1)) + w(j-1)*d*cos(phic(i,j-1));
%         dphic(i,j) = w(j-1);
%         
%         xc(i,j) = dxc(i,j)*dt1 + xc(i,j-1);
%         yc(i,j) = dyc(i,j)*dt1 + yc(i,j-1);
%         phic(i,j) = dphic(i,j)*dt1 + phic(i,j-1);
%         phi(i,j) = phic(i,j);
%         
%         phi(i,j)=phic(i,j);
%         x(i,j)=xc(i,j)-d*cos(phi(i,j));
%         y(i,j)=yc(i,j)-d*sin(phi(i,j));
%         delete(run);
%         run = trectangle1(x(i,j), y(i,j),phi(i,j)*180/pi,wagv, hagv, 'green');
%         plot(xc(i,j),yc(i,j),'r.-');
%         drawnow;
%         Wm(j,:) = Wm(j,:)*9.6*60/(2*pi);
%    end
%         Wm_plot(i,:) = Wm(m,:);
%         W_plot(i) = w(m);
%         V_plot(i) = v(m);
   
   u = (evalfis([ef(i) def(i)], f));
   w1(i) = u(1);
   w2(i) = u(2);
%    
   %w(i) = Kp*e2(i) + Ki*(e2(i) + e2(i-1)) + Kd *(e2(i) - e2(i-1)) + wr(i);
   %w0(i) = (r/b)*(w1(i) - w2(i));
   w(i) = (r/b)*(w1(i) - w2(i)) + wr(i);
   
   
%    if w(i) > w_max
%        w(i) = w_max;
%    else if w(i) < - w_max
%        w(i) = -w_max;
%        end
%    end
   
   %tracking point dynamics
%    ph(i)= phr(i)-e3(i);
%    Ae=[cos(ph(i))  sin(ph(i))  0;  -sin(ph(i))  cos(ph(i))  0;  0  0  1];
%    er= inv(Ae)*E;
%    xc(i)= xr(i)-er(1);
%    yc(i)= yr(i)-er(2);
%    phc(i)=phr(i)-er(3);
   
   %wheel dynamics [rpm]
%    vwl(i)=(1/r)*(v(i)+b*w(i))*(60/(2*pi));
%    vwr(i)=(1/r)*(v(i)-b*w(i))*(60/(2*pi));
   
   %robot center
   x(i)=xc(i)-d*cos(ph(i));
   y(i)=yc(i)-d*sin(ph(i));
   
end;


plot(t,e1,'r',t, e2*1000);title('Reponse of error using Fuzzy controller'); xlabel('Time(s)'),ylabel('Error(mm)');
figure;plot(xc,yc,'-.');	       
xlabel('X Coordinate (m)'); ylabel('Y Coordinate (m)');
axis equal; hold on;
plot(xr,yr,'lineWidth',5/(71/25.4));

for i=1:1:n
    
    trectangle(xc(i),yc(i),phc(i)*180/pi,wagv,hagv,'red');
end;

% for i = 1:1:n
%     trectangle(xc(i,j), yc(i), phic(i)*180/pi, wa
legend('Model','Reference');
title('Path following simulation');
%figure;	plot(t,w1*60/(2*pi),t,w2*60/(2*pi),'--');              xlabel('Time (s)'); ylabel('Wheel Velocity (rpm)');




close all
clear all

%load agv_ref_junction
%load datareference.mat xr yr phr wr

%system dynamics
hagv= 200;    %width of robot, mm
wagv= 250;    %length of robot, mm
b= 180;       %wheel distance, mm
r= 32.5;       %wheel radius, mm
d= 80;       %mobile robot center to robot's tracking sensor, m
bl= 26;      %line width, mm
vr = 1000;         %velocity, mm/s

%sampling time and data time
dt=0.05;        %Sampling time of robot
dt1 = 0.005;    %Sampling time of motor
t(1)=0;
[xr yr phr wr] = draw(vr,dt);
n=length(xr);   %Number of samples
m = round(dt/dt1);

%draw(vr,dt);

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
f = readfis('Fuzzy_Controller3.fis');
u(1) = 0;

% tracking point
xc(1)=xr(1);              %Initial position         
yc(1)=yr(1);
phc(1)=phr(1)+10*pi/180;        %Initial angle
dxc(1) = 0;
dyc(1) = 0;
dphc(1) = 0;

% Robot Center
ph(1) = phc(1);
w(1) = 0;
v(1) = 0;
w_left(1) = 0;
w_right(1) = 0;
w_left_ref(1) = 46;
w_right_ref(1) = 46;
x(1)=xc(1)-d*cos(ph(1));
y(1)=yc(1)-d*sin(ph(1));
e(1) = 0;
e_ref(1) = 0;
run = trectangle1(xc(1), yc(1), phc(1)*180/pi, wagv, hagv,'green');

%----------------------------MAIN PROGRAM---------------------------
for i = 2:n
    t(i) = (i-1)*dt;
    
    %Error dynamics 
    e_ref(i) = 0;
    
    % Tracking point
    dxc(i) = v(i-1)*cos(ph(i-1)) - w(i-1)*d*sin(ph(i-1));
    dyc(i) = v(i-1)*sin(ph(i-1)) + w(i-1)*d*cos(ph(i-1));
    dphc(i) = w(i-1);
    xc(i) = dxc(i-1)*dt + xc(i-1);
    yc(i) = dyc(i-1)*dt + yc(i-1);
    phc(i) = dphc(i-1)*dt + phc(i-1);
    ph(i) = phc(i);

    % Error dynamics
    Ae=[cos(ph(i))  sin(ph(i))  0;  -sin(ph(i))  cos(ph(i))  0;  0  0  1];
    E = Ae*[xr(i)-xc(i);yr(i)-yc(i);phr(i)-phc(i)];
    e2(i) = E(2);
    %e(i) = Error(ref1(xc(i), yc(i), phc(i)));
    %e_ref(i) = 0;
    %e(i) = Error(e2(i));
    e(i) = e2(i);
    de(i) = e(i) - e(i-1);
    
    %Check error range
    if e(i) > 10
        e(i) = 9.5;
    elseif e(i) < -10
        e(i) = -9.5;
    else
        e(i) = e(i);
    end
    
    %Check change of error range
    if de(i) > 6
        de(i) = 5;
    elseif de(i) < -6
        de(i) = -5;
    else 
        de(i) = de(i);
    end
    
    w_left_ref (i) = 46;
    w_right_ref (i) = 46;
    % Fuzzy Controller
    u = evalfis([e(i) de(i)], f);
    w_left(i) = w_left_ref(i) + u(1);
    w_right(i) = w_right_ref(i) + u(2);
    
    % Velocity
    w(i) = (r/b)*(w_left(i) - w_right(i)) + wr(i);
    %v(i) = (w_left(i) + w_right(i))*r/2;
    v(i) = vr;
    
    % Robot Center
    x(i) = xc(i) - d*cos(ph(i));
    y(i) = yc(i) - d*sin(ph(i));
    
    %Plot
    delete(run);
    run = trectangle1(xc(i), yc(i), phc(i)*180/pi, wagv, hagv,'green');
    plot(xc(i),yc(i),'r.-');
    drawnow;
end

legend('Reference Line', 'Mobile robot tracking line');
figure;
hold on;
title('Response of the Error using Fuzzy controller');
xlabel('Time(s)');
ylabel('System error (mm)');
plot(t,e_ref,'r','LineWidth',2);
plot(t,e,'b','LineWidth',1.5);
% xlim([0,n*dt]);
% ylim([min(e2),max(e2)]);

figure(3);
title('Response of the motor 1');
xlabel('Time(s)');
ylabel('Angular velocity rpm');
hold on;
% plot(t,w1r,'r','LineWidth',1);
plot(t,440 + w_left,'r','LineWidth',1);
xlim([0,n*dt+1]);


hold on;
title('Response of the motor');
xlabel('Time(s)');
ylabel('Angular velocity rpm');
hold on;

plot(t, 440 + w_right,'b','LineWidth',1);
xlim([0,n*dt+1]);




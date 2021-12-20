clc;
clear all;



%CREATE RPM AND %PWM DIAGRAM
% figure(1);
% plot([10:5:100],[0 0 45 60 70 80 89 108 143 185 228 270 315 352 394 434 470 520 560]);
% xlabel('Van toc (rpm)');
% ylabel('%PWM');
% title('Do thi quan he giua tin hieu vao va tin hieu ra dong co 1');
% hold on;
% grid on;
% 
% figure(2);
% plot([10:5:100],[0 0 78 143 182 210 240 265 291 315 350 377 390 420 447 473 490 520 570]);
% xlabel("Van toc (rpm)");
% ylabel("%PWM");
% title('Do thi quan he giua tin hieu vao va tin hieu ra dong co 2');
% grid on;

%CREATE RPM REFERENCE
% dt = 0.02;
% t(1) = 0;
% 
% for i = 2:50
%     t(i) = i*dt;
% end
% 
% ref(1) = 0;
% 
% for k = 2:50
%     if (k > 0 & k <= 2)
%         ref(k) = k*20;
%     elseif (k > 2 & k <= 10)
%         ref(k) = 40;
%     elseif (k > 10 & k <= 20)
%         ref(k) = 55;
%     elseif (k > 20 & k <= 30)
%         ref(k) = 70;
%     elseif (k > 30 & k <= 40)
%         ref(k) = 85;
%     elseif (k > 40)
%         ref(k) = 100;
%     end
% end

% figure;
% xlim([0 1.2]);
% ylim([0 120]);
% plot(t,ref(1:50));
% xlabel('Time (s)');
% ylabel('%PWM');
% title('Do thi tin hieu %PWM cap cho dong co theo thoi gian');

%-----------------------------------------------------------
%RPM RESPONSE 
data = xlsread('DC_Motor_Data');
time = data(1:50,1);
ref1 = data(1:21,31);
pwm1 = data(1:21,32);

ref2 = data(1:21,35);
pwm2 = data(1:21,36);

motor1_res = data(1:50, 24);
motor2_res = data(1:50, 25);
pwm = data(1:50, 28);
pwm_ref = data(1:21,34);

% --------------------------------------------------
% % PWM ??NG C? 1
figure;
plot(pwm_ref, pwm1, 'r');
xlabel("%PWM");
ylabel("RPM");
title("Do thi dac tuyen %PWM- RPM cua dong co 1");

% -----------------------------------------------------
% % PWM ??NG C? 1
figure;
plot(pwm_ref, pwm2, 'r');
xlabel("%PWM");
ylabel("RPM");
title("Do thi dac tuyen %PWM- RPM cua dong co 2");

%---------------------------------------------------
% CHECK PWM 1
figure;
plot(pwm_ref, ref1,'r');
hold on;
plot(pwm_ref, pwm1,'b');
xlabel("%PWM");
ylabel("RPM");
title("Kiem chung ham truyen dong co 1");

%-----------------------------------------
%CHECK PWM 2
figure;
plot(pwm_ref, ref2,'r');
hold on;
plot(pwm_ref, pwm2,'b');
xlabel("%PWM");
ylabel("RPM");
title("Kiem chung ham truyen dong co 2");


% motor1_res = motor1_res';
% motor2_res = motor2_res';
% rpm1  = data(1:50,15);
% rpm2 = data(1:50,16);
% time = time';
% rpm1 = rpm1';
% rpm2 = rpm2';
% 
% rpm3 = data(:, 16);
% rpm3 = rpm3';
% 
% % %NON-PID MOTOR'S RESPONSES
% figure(3);
% plot(time, motor1_res,'r');
% xlabel("Time (s)");
% ylabel("Velocity (rpm)");
% title("Response of motor 1");
% figure(4);
% plot(time, motor2_res,'b');
% xlabel("Time (s)");
% ylabel("Velocity (rpm)");
% title("Response of motor 2");

%PID MOTOR'S RESPONSES
% figure(4);
% plot(time,rpm1,'r');
% hold on;
% legend('Motor 1');
% figure;
% plot(time,rpm2,'b');
% xlabel('Time (s)');
% ylabel('RPM');
% %%title('PWM = 100%');
% legend('Motor 2');



%TRANSFER FUNCTION OF DC MOTOR
s = tf('s');
% % dc_01 = 229.9/(s+5.931e-6);
% % dc_02 = 247.4/(s + 0.000104);
% dc_01 = 3395/(s+0.0009313);
% dc_02 = 5944/(s + 0.0002343);
dc_01 = 103.8/(s + 46.51);
dc_02 = 106.8/(s + 47.98);

% %STEP RESPONSE
% figure;
% res1 = feedback(dc_01,1);
% display(dc_01);
% step(dc_01);
% title('Step response of motor 1');
% figure;
% res2 = feedback(dc_02,1);
% step(dc_02);
% title('Step response of motor 2');

% e1 = ilaplace(dc_01);
% e2 = ilaplace(dc_02);
% display(e1);
% display(e2);

% % PID RESPONSE
% figure;
% Kp1 = 0.3;Ki1 = 0.362;Kp2 = 0.2682;Ki2 = 0.3117;
% u1 = pid(Kp1,Ki1);
% u2 = pid(Kp2, Ki2);
% motor1 = feedback(dc_01*u1,1);
% step(motor1);
% title("PID controller of motor 1");
% display(stepinfo(motor1));
% hold on;
% grid on;
% figure;
% motor2 = feedback(dc_02*u2,1);
% step(motor2);
% title("PID controller of motor 2");
% grid on;
% display(stepinfo(motor2));
% 
a = 105.9;
b = 46.51;
% pwm(1) = 0;
a1 = 106.8;
b1 = 47.98;
for j = 1:21
    pwm(j) = 5*j/100*255;
    rpm(1) = 0;
    rpm_dot(1) = 0;
    for i = 2:50
        dt = 0.005;
        rpm_dot(i) = a*pwm(j) - b*rpm(i-1);
        rpm(i) = rpm_dot(i-1)*dt + rpm(i-1);
    end
    r1(j) = rpm(end);
%     e(j) = abs(rpm1(j) - r1(j)');
end
% % for i = 1: 172
% %     ref(i) = 80;
% % end
% 
% % PID ACTUAL RESPONSE OF MOTOR
% pid_res1 = data(1:50,15);
% pid_res2 = data(1:50,16);
% %plot(time, ref);
% hold on;
% plot(time, pid_res1,'r');xlim([0 0.25]);
% plot(time, pid_res2,'b');xlim([0 0.25]);
% %ylim([0 120]);
% xlabel('Time');
% ylabel('Rpm');



        
        
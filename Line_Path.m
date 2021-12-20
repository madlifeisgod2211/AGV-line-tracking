%For AGV line tracking control
%For EatMe Shuttle Project

close all
clear all

dt=0.02;	%s
b=0.026;    %path width
%Vr=0.82;
Vr = 1.5;   %refernce velocity, m/s
x0=0;y0=0;  %path start point

kc=100/3*pi/4;           %number of step in the curver line
Rr=kc*Vr*dt*(4/(2*pi));	%radius of curver, m
Lr=kc*Vr*dt;            %length of curver, m
ks=200/3;                %number of step in the straight line
L=ks*Vr*dt;             %length of straight line, m


%corrdinate of section points
x1=x0;                  y1=y0;
x2=x1+L;                y2=y1;
x3=x2;                  y3=y2;
x4=x3+Rr;               y4=y3+Rr;
x5=x4;                  y5=y4;
x6=x5-Rr;               y6=y5+Rr;
x7=x6-L;                y7=y6;
x8=x7-Rr;               y8=y7-Rr;

%LINE 1: STRAIGHT
xr1(1)=x1;
yr1(1)=y1;
phr1(1)=0;
wr1(1)=0;
n1=ks;
for i=2:n1
   xr1(i)=xr1(i-1)+Vr*dt;
   yr1(i)=yr1(i-1);
   phr1(i)=0;
   wr1(i)=0;
end;

%LINE 2: CURVE
xr2(1)=x3;
yr2(1)=y3;
phr2(1)=0;
wr2(1)=Vr/Rr;
n2=kc;

for i=2:n2
   if i==2 i23=0; end;
   xr2(i)=x3+Rr*sin(Vr*i23*dt/Rr);
   yr2(i)=y3+Rr*(1-cos(Vr*i23*dt/Rr));
   phr2(i)=Vr*i23*dt/Rr;
   wr2(i)=Vr/Rr;
   i23=i23+1;
end;

%LINE 3: CURVE
xr3(1) = x5;
yr3(1) = y5;
phr3(1)= 0;
wr3(1) = Vr/Rr;
n3 = kc;

for i=2:n3
   if i==2 i23=0; end;
   xr3(i)= x5 - Rr*(1 - cos(Vr*i23*dt/Rr));
   yr3(i)= y5 + Rr* sin(Vr*i23*dt/Rr);
   phr3(i)= pi/2 + Vr*i23*dt/Rr;
   %phr3(i) = 0;
   wr3(i)=Vr/Rr;
   i23=i23+1;
end;

%LINE 4: STRAIGHT
xr4(1) = x6;
yr4(1) = y6;
phr4(1) = pi;
wr4(1) = 0;
n4 = ks;

for i=2:n4
   xr4(i)=xr4(i-1)-Vr*dt;
   yr4(i)=yr4(i-1);
   phr4(i)=0;
   wr4(i)=0;
end;

%LINE 5: CURVE
xr5(1) = x7;
yr5(1) = y7;
phr5(1)= 0;
wr5(1) = Vr/Rr;
n5 = kc;

for i=2:n5
   if i==2 i23=0; end;
   xr5(i)= x7 - Rr*sin(Vr*i23*dt/Rr);
   yr5(i)= y7 - Rr*(1- cos(Vr*i23*dt/Rr));
   phr5(i)= pi + Vr*i23*dt/Rr;
   wr5(i)=Vr/Rr;
   i23=i23+1;
end;

%LINE 7: CURVE
xr6(1) = x8;
yr6(1) = y8;
phr6(1)= 3*pi/2;
wr6(1) = Vr/Rr;
n6 = kc;

for i=2:n6
   if i==2 i23=0; end;
   xr6(i)= x8 + Rr*(1-cos(Vr*i23*dt/Rr));
   yr6(i)= y8 - Rr*sin(Vr*i23*dt/Rr);
   phr6(i)=3*pi/2+Vr*i23*dt/Rr;
   wr6(i)=Vr/Rr;
   i23=i23+1;
end;
 % xr6(1,14) = 0;
 % yr6(1,14) = 0;

plot([xr1,xr2,xr3,xr4,xr5,xr6],[yr1,yr2,yr3,yr4,yr5,yr6],'lineWidth',5/(71/25.4)); %lindwidth 5mm
hold on
plot(x1,y1,'*',x2,y2,'*',x3,y3,'*',x4,y4,'*',x5,y5,'*',x6,y6,'*',x7,y7,'*',x8,y8,'*');
axis equal

xr=[xr1,xr2,xr3,xr4,xr5,xr6];
yr=[yr1,yr2,yr3,yr4,yr5,yr6];


phr=[phr1,phr2,phr3,phr4,phr5,phr6];
wr=[wr1,wr2,wr3,wr4,wr5,wr6];

n=n1+n2+n3;
dwr(1)=0;
for i=2:n
  dwr(i)=(wr(i)-wr(i-1))/dt;
end;

vr=Vr;

save datareference.mat xr yr phr wr dwr vr x2 y4

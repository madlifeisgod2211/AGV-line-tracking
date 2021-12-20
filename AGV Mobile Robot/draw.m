function [xR yR phiR wR]= draw(vR,tsamp)

R=500;
xR(1)=1500;
yR(1)=0;
wR(1)=0;
phiR(1)=pi/2;
a=xR(1);
i=2;

% A --> B
while a>1000
    xR(i)=1000+500*cos((vR*tsamp*i/R));
    yR(i)=500*sin((vR*tsamp*i/R));
    phiR(i)=pi/2+vR*tsamp*(i)/R;
    a=xR(i);
    wR(i)=vR/R;
    if xR(i)<1000
        xR(i)=999;
    end
    i=i+1;
end
% B --> C
h=i-1;
while (a<1000 && a >-1000)
        xR(i)=1000-vR*tsamp*(i-h);
        yR(i)=500;
        phiR(i)=pi;
        a=xR(i);
        wR(i)=0;
        if xR(i)<-1000
            xR(i)=-1000;
        end
       i=i+1;
end
% C --> D
j=i-1;
while a<=-1000
    xR(i)=-1000+500*cos(pi/2+(vR*tsamp*(i-j)/R));
    yR(i)=500*sin(pi/2+(vR*tsamp*(i-j)/R));
    phiR(i)=pi+vR*tsamp*(i-j)/R;
    a=xR(i);
    wR(i)=vR/R;
    if xR(i)>-1000
       xR(i)=-1000;
    end
    i=i+1;
end
% D --> E
while (a<1000)
        xR(i)=xR(i-1)+vR*tsamp;
        yR(i)=-500;
        phiR(i)=0;
        a=xR(i);
        b=yR(i);
        wR(i)=0;
    if xR(i)>1000
       xR(i)=1000;
    end
        i=i+1;
end
% E --> F
k=i-1;
while (a>=1000 && b<0)
    xR(i)=1000+500*cos(pi/2-(vR*tsamp*(i-k)/R));
    yR(i)=-500*sin(pi/2-(vR*tsamp*(i-k)/R));
    phiR(i)=vR*tsamp*(i-k)/R;
    wR(i)=vR/R;
    a=xR(i);
    if (yR(i) > 0)
        break;
    end
    i=i+1;
end
 figure(1);
 hold on;
 title('Mobile robot tracking line');
 xlabel('X');
 ylabel('Y');
 plot(xR,yR,'b','LineWidth',3.5);
 xlim([-1800,1800]);
 ylim([-700,700]);
%  legend('Reference Line');
end


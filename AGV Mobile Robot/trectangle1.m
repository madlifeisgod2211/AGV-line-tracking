%for drawing a rectangle
%Tien 2014/08/26
%
%syntax     trectangle(x,y,phi,w,h,color)
%
%x,y        rectangle center cordinate, m
%phi        rectangle angle, deg
%w          width of rectangle, m
%h          height of rectangle, m
%color      circle color

function figure = trectangle1(x,y,phi,w,h,color)

    phi=phi*pi/180;
    alp=atan(h/w);
    r=sqrt(h*h+w*w)/2;
    x1=x+r*cos(alp+phi);   y1=y+r*sin(alp+phi);
    x2=x-r*cos(alp-phi);   y2=y+r*sin(alp-phi);
    x3=x-r*cos(alp+phi);   y3=y-r*sin(alp+phi);
    x4=x+r*cos(alp-phi);   y4=y-r*sin(alp-phi);
    hold on
    axis equal
    %plot([x1,y1],[x2,y2],[x2,y2],[x3,y3],[x3,y3],[x4,y4],[x4,y4],[x1,y1],color);
    figure(1) = plot([x1,x2],[y1,y2],color);
    figure(2) = plot([x2,x3],[y2,y3],color);
    figure(3) = plot([x3,x4],[y3,y4],color);
    figure(4) = plot([x4,x1],[y4,y1],color);
          
end

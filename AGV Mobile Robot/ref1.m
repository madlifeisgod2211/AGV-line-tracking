function e = ref(xc,yc,phic)
e = 0;

    if(xc > 1)
        m = yc + cot(phic)*xc;         %Sai l?ch theo ph??ng y
        a1 = 1 + (cot(phic))^2;
        a2 = 2 + 2*m*cot(phic); 
        a3 = 0.75 + m^2;  
        delta = abs(a2^2 - 4*a1*a3);
        x_R1 = (a2 + sqrt(delta))/(2*a1);
        x_R2 = (a2 - sqrt(delta))/(2*a1);     
        y_R1 = -cot(phic)*x_R1 + m;
        y_R2 = -cot(phic)*x_R2 + m;
        e1 = - (x_R1 - xc)*sin(phic)+(y_R1 - yc)*cos(phic);
        e2 =  - (x_R2 - xc)*sin(phic)+(y_R2 - yc)*cos(phic);
        
        if (abs(e1) > abs(e2))
            e = e2;
        else
            e = e1;
        end
    %Error on the top circular line
    elseif(xc <= 1 && xc > -1 && yc > 0)   
        x_R = yc*tan(phic) + xc - 0.5*tan(phic);
        y_R = 0.5;
        e = -sin(phic)*(x_R-xc)+cos(phic)*(y_R-yc);
    %Error on the left circular line
    elseif(xc <= -1)
        m = yc + cot(phic)*xc;
        a1 = 1 + (cot(phic))^2;
        a2 = -2 + 2*m*cot(phic); 
        a3 = 0.75 + m^2;   
        delta = sqrt(a2^2-4*a1*a3);
        x_R1 = (a2 + delta)/(2*a1);
        x_R2 = (a2 - delta)/(2*a1);     
        y_R1 = -cot(phic)*x_R1 + m;
        y_R2 = -cot(phic)*x_R2 + m;
        e1 = - (x_R1 - xc)*sin(phic) + (y_R1 - yc)*cos(phic) ;
        e2 = - (x_R2 - xc)*sin(phic) + (y_R2 - yc)*cos(phic);
        if (abs(e1) > abs(e2))
            e = e2;
        else
            e = e1;
        end
    %Error on the bottom circular line
    elseif(xc >= -1 && xc <= 1 && yc < 0)
        x_R = yc*tan(phic) + xc + 0.5*tan(phic);
        y_R = -0.5;
        e = -sin(phic)*(x_R-xc)+cos(phic)*(y_R-yc);
    end 
end
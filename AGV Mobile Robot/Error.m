function sensor = SS(e)
e = e*1000;
a = 0.7666;
b = 0.6359;
sensor = (a*e+b)/1000;
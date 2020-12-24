function e = noise(t,i)
% Exploration signals.

ampl = [0.72,  0.7, 0.72;
          0.7, 0.66, 0.68;
         0.68, 0.72,  0.7;
         0.57, 0.58, 0.69];
     
e1 = ampl(i,1)*(sin(3*pi*t) + sin(7*t) + sin(11*t) +...
     sin(15*t) + sin(17*t) + sin(sqrt(11)*t));
 
e2 = ampl(i,2)*(sin(sqrt(2)*t) + sin(3*t) + sin(7*t) +...
     sin(11*t) + sin(13*t) + sin(15*t));
 
e3 = ampl(i,3)*(sin(2*t) + sin(7*pi*t) + sin(sqrt(6)*t) +...
     sin(sqrt(10)*t) + sin(11*pi*t) + sin(13*t));
 
e = [e1, e2, e3];
end
syms theta1 x1 y1 dtheta1 dx1 dy1 ddtheta1 ddx1 ddy1 theta2 x2 y2 dtheta2 dx2 dy2 ddtheta2 ddx2 ddy2 
syms Tin bp r kp Ip F Mpd A a Tout Mw1 Mw2 b k
f(1)=Tin-bp*(dx1/r-dtheta1)-kp*(x1/r-theta1)-Ip*ddtheta1==0;
f(2)=bp*(dx1-dtheta1*r)+kp*(x1-theta1*r)-F-Mpd*ddx1==0;
f(3)=dy1*a-dx1*A==0;
f(4)=bp*(dx2/r-dtheta2)-kp*(x2/r-theta2)-Ip*ddtheta2-Tout==0;
f(5)=bp*(dx2-dtheta2*r)+kp*(x2-theta2*r)-F-Mpd*ddx2==0;
f(6)=-(F*a/A)-(b*dy2)-(k*(y2-y1))-Mw2*ddy2==0;
f(7)=F*a/A-b*dy1-k*(y2-y1)-Mw1*ddy1==0;
[A,B]=equationsToMatrix(f,[theta1 x1 y1 dtheta1 dx1 dy1 theta2 x2 y2 dtheta2 dx2 dy2])
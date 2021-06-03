function F = ctlrRDHTforce2(t,X,c,p,tau_des)

    tau_out = [0  0  0    0   p.kp*p.r    p.bp*p.r    -p.kp*p.r^2     -p.bp*p.r^2]*X;
    F = -c.k*(tau_out-tau_des) + tau_des;

%         F = tau_des;
    if F > 20
        F = 20;
    elseif F < -20
        F = -20;
    end

end
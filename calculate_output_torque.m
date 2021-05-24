function tau_out = calculate_output_torque(X, p)
    tau_out = [0  0   0   0   p.kp*p.r   p.bp*p.r   -p.kp*p.r^2   -p.bp*p.r^2]*X;
end
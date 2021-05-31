function tau_out = calculate_output_torque(t_vec, X_vec, p, traj_fun)
    [~, ~, ddy] = traj_fun(t_vec);
    tau_out = [0  0   0   0   p.kp*p.r    p.bp*p.r   -p.kp*p.r^2   -p.bp*p.r^2]*X_vec + p.Ip*ddy;
end
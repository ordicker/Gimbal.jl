
function gimbal_conntroller(;
                            full=true,
                            J = 0.029,
                            k_s = 0.2,
                            k_v = 0.545,
                            ω_brk = 0.074,
                            T_c = 0.106,
                            T_brk = 0.177,
                            K_dc = 6878.7,
                            Z1 = 0.01,
                            Z2 = 0.029,
                            P2 = 0.16,
                            Z3 = 0.06,
                            P3 = 0.009,
                            Z4 = 0.01,
                            bw = 465.0)
    @parameters t
    @named pid1 = PID_factory(kp=Z1,ki=1.0,kd=0.0)
    @named lag = lead_lag_factory(k=1.0, zero=Z2, pole=P2)
    @named lead = lead_lag_factory(k=1.0, zero=Z3, pole=P3)
    @named pid2 = PID_factory(kp=Z4,ki=1.0,kd=0.0)
    @named notch1 = notch(ω=100.0,gain=0.1,width=0.3)
    @named notch2 = notch(ω=330.0,gain=0.1,width=0.3)
    @named notch3 = notch(ω=380.0,gain=0.1,width=0.3)
    @named notch4 = notch(ω=475.0,gain=0.1,width=0.3)
    @named p = gimbal_plant(;J,k_s,k_v,ω_brk,T_c,T_brk)
    @named gyro = sensor(bw=bw)
    tspan = (0.0,5.0)
    if full
        connections = [
            p.ω_body ~ 16*π^2/180*sin(2π*2t)
            gyro.input ~ p.ω
            pid1.e ~ -(180/π)*gyro.output
            lead.input ~ pid1.u
            lag.input ~ lead.output 
            pid2.e ~ lag.output
            notch1.input ~ K_dc*pid2.u
            notch2.input ~ notch1.output
            notch3.input ~ notch2.output
            notch4.input ~ notch3.output
            p.T ~ 0.339*min(max(notch4.output,-2.5),2.5)]
        @named _connected = ODESystem(connections, t)
        @named connected = compose(_connected,
                                   [pid1, pid2, lead, lag,
                                    notch1, notch2, notch3, notch4,
                                    p, gyro])
        sys = alias_elimination(connected)
        prob = ODEProblem(sys,[],tspan,jac=true)
    else
        connections = [
            p.ω_body ~ 16*π^2/180*sin(2π*2t)
            pid1.e ~ -(180/π)*p.ω
            lead.input ~ pid1.u
            lag.input ~ lead.output 
            pid2.e ~ lag.output
            p.T ~ 0.339*min(max(K_dc*pid2.u,-2.5),2.5)]
        @named _connected = ODESystem(connections, t)
        @named connected = compose(_connected,
                                   [pid1, pid2, lead, lag,p])
        sys = structural_simplify(dae_index_lowering(connected))
        prob = ODEProblem(sys,[],tspan,jac=true)
    end


    return  prob, p, sys 
end



export gimbal_conntroller



function controller_factory(;name)
    @parameters t
    @variables u(t) e(t) Ie(t) De(t)
    D = Differential(t)
    eqs= [
        D(u) ~ 0 # force u to be a state, apply control law in callback
        D(Ie) ~ e
        D(e) ~ De
    ]
    ODESystem(eqs, t; name, defaults=[u=>0.0,e=>0.0, Ie=>0.0,De=>0.0])
end


function gimbal_discrete_conntroller(;
                            J = 0.029,
                            k_s = 0.2,
                            k_v = 0.545,
                            ω_brk = 0.074,
                            T_c = 0.106,
                            T_brk = 0.177,
                            bw = 465.0)
    @parameters t

    @named controller = controller_factory()
    @named p = gimbal_plant(;J,k_s,k_v,ω_brk,T_c,T_brk)
    @named gyro = sensor(bw=bw)
    tspan = (0.0,1.0)
    connections = [
        p.ω_body ~ 16*π^2/180*sin(2π*2t)
        controller.e ~ -(180/π)*p.ω
        p.T ~ 0.339*min(max(controller.u,-2.5),2.5)]
    @named _connected = ODESystem(connections, t)
    @named connected = compose(_connected,
                               [controller, p])
    #sys = structural_simplify(dae_index_lowering(connected))

    sys = dae_index_lowering(connected)

    prob = ODEProblem(sys,[],tspan,jac=true)

    indexof(sym,syms) = findfirst(isequal(sym),syms)
    function cb!(int)
        e = prob.f.observed(controller.e, int.u, int.p, int.t)
        int.u[indexof(controller.u, states(sys))] = -0.1*e
    end

    sol = solve(prob, Rodas5(), callback=PeriodicCallback(cb!, 0.001; initial_affect=true))
    return sol,controller
    #return  prob, p, sys, cb!
end



#export gimbal_discrete_conntroller

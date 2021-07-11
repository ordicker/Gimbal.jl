using ModelingToolkit, DifferentialEquations, Plots

@parameters t

function PID_factory(;name, kp=1.0, ki=0.0, kd=0.0)
    @parameters _kp _ki _kd
    @variables t e(t) Ie(t) de(t) u(t)
    D = Differential(t)
    
    eqs = [
        D(Ie) ~ e
        D(e) ~ de
        u ~ _ki*Ie+_kp*e+_kd*de
    ]
    ODESystem(eqs, t ;name, defaults=[u=>0.0,
                                      e=>0.0, de=>0.0, Ie=>0.0,
                                      _kp=>kp, _ki=>ki, _kd=>kd])
end

function test()
    @named pid1 = Gimbal.PID_factory(kp=350.0,ki=300.0,kd=50.0)
    @named p = Gimbal.plant()
    connections = [pid1.e ~ 1/(1+exp(-100*(t-1)))-p.x
                   p.F ~ pid1.u]
    @named connected = ODESystem(connections ; systems=[pid1,p])
    #@show equations(connected)
    sys = structural_simplify(connected)
    prob = ODEProblem(sys,[],(0.0,3.0))
    sol = solve(prob, ImplicitEuler(),adaptive=false, dt=4e-4)
    plot(sol, vars = [p.x])#, pid1.e, pid1.u])
end

function plant(;name)
    @variables t F(t) x(t) dx(t)
    D = Differential(t)  
    eqs = [
        D(x) ~ dx
        D(dx) ~ F - 10*dx - 20*x
    ]
    ODESystem(eqs, t;name, defaults=[x=>0.0, dx=>0.0])
end

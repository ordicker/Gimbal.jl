using ModelingToolkit

@parameters t

function PID_factory(;name, kp=1.0, ki=0.0, kd=0.0)
    @parameters _kp _ki _kd
    @variables t e(t) Ie(t) de(t) u(t)
    D = Differential(t)
    
    eqs = [
        D(Ie) ~ e
        D(e) ~ de
        D(u) ~ _ki*Ie+_kp*e+_kd*de
    ]
    ODESystem(eqs;name, defaults=[u=>0.0,
                                  e=>0.0, de=>0.0, Ie=>0.0,
                                  _kp=>kp, _ki=>ki, _kd=>kd])
end

function test()
    @named pid1 = Gimbal.PID_factory(kp=2.2,ki=0.1,kd=0.3)
    @named p = Gimbal.plant()
    connections = [ pid1.e ~ p.x
                    p.F ~ sin(t)]
    @named connected = ODESystem(connections; systems=[pid1,p])
    @show equations(connected)
    connected |> structural_simplify 
end

function plant(;name)
    @variables t F(t) x(t)
    D = Differential(t)  
    eqs = [
        D(x) ~ F
    ]
    ODESystem(eqs;name, defaults=[x=>0.0])
end

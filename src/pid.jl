using ModelingToolkit

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


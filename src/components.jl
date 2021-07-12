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

function lead_lag_factory(zero, pole;name)
    @variables t ein(t) din(t) out(t) dout(t)
    @parameters _zero _pole
    D = Differential(t)
    eqs = [
        D(ein) ~ din
        D(out) ~ dout
        _pole*dout + out ~ _zero*din + ein
    ]
    ODESystem(eqs, t;name, defaults=[out=>0.0,dout=>0.0,
                                     ein=>0.0, din=>0.0,
                                     _zero=>zero, _pole=>pole])
end

using ModelingToolkit

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

function lead_lag_factory(;name,k=1.0, zero=1.0, pole=0.0)
    @variables t input(t) dinput(t) output(t) doutput(t)
    @parameters _zero _pole _k
    D = Differential(t)
    eqs = [
        D(input) ~ dinput
        D(output) ~ doutput
        doutput + _pole*output ~ _k*(dinput + _zero*input)
    ]
    ODESystem(eqs, t ;name, defaults=[output=>0.0,doutput=>0.0,
                                      input=>0.0, dinput=>0.0,
                                      _zero=>zero, _pole=>pole, _k=>k])
end

export PID_factory, lead_lag_factory


function PID_factory(;name, kp=1.0, ki=0.0, kd=0.0)
    @parameters _kp _ki _kd t
    @variables e(t) Ie(t) de(t) u(t)
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
    @parameters t _zero _pole _k
    @variables input(t) dinput(t) output(t) doutput(t)
    D = Differential(t)
    eqs = [
        D(input) ~ dinput
        D(output) ~ doutput
        _pole*doutput + output ~ _k*(_zero*dinput + input)
    ]
    ODESystem(eqs, t ;name, defaults=[output=>0.0, doutput=>0.0,
                                      input=>0.0, dinput=>0.0,
                                      _zero=>zero, _pole=>pole, _k=>k])
end

function sensor(;name, bw=1)
    @parameters t a2 a1
    @variables input(t) output(t) doutput(t) ddoutput(t)
    D = Differential(t)
    eqs = [
        D(output) ~ doutput
        D(doutput) ~ ddoutput
        input ~ output+a1*doutput+a2*ddoutput
    ]
    ODESystem(eqs, t ;name, defaults=[output=>0.0, doutput=>0.0,
                                      ddoutput=>0.0, input=>0.0,
                                      a1=>2*0.7/(2π*bw), a2=>1/(2π*bw^2)])
end

function notch(;name,ω=1.0,gain=1.0,width=1.0)
    @parameters t i1 i2 o1 o2
    @variables input(t) dinput(t) ddinput(t) output(t) doutput(t) ddoutput(t)
    D = Differential(t)
    eqs = [
        D(output) ~ doutput
        D(doutput) ~ ddoutput
        D(input) ~ dinput
        D(dinput) ~ ddinput
        input+i1*dinput+i2*ddinput ~ output+o1*doutput+o2*ddoutput
    ]
    ODESystem(eqs, t ;name, defaults=[output=>0.0, doutput=>0.0, ddoutput=>0.0,
                                      input=>0.0, dinput=>0.0, ddinput=>0.0,
                                      o1=>width/(2π*ω), o2=>1/(2π*ω)^2,
                                      i1=>width*gain/(2π*ω), i2=>1/(2π*ω)^2])

end

export PID_factory, lead_lag_factory, sensor, notch

using ModelingToolkit

function gimbal_plant(;name,J=1.0, k_s=1.0, k_v=1.0, ω_brk=1.0, T_c=1.0)
    @variables t T(t) ω(t) θ(t)
    @parameters _J _k_s _k_v _ω_brk _T_c
    D = Differential(t)
    eqs = [
        D(θ) ~ ω
        D(ω) ~ (T-_k_s*θ-_k_v*ω-_T_c*tanh(10*ω/_ω_brk))/_J
    ]
    ODESystem(eqs, t; name, defaults=[ω=>0.0, θ=>0.0, T=>0.0,
                                      _T_c=>T_c, _ω_brk=>ω_brk,
                                      _J=>J, _k_s=>k_s, _k_v=>k_v])
end

export gimbal_plant

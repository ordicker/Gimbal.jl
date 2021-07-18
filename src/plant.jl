using ModelingToolkit

function gimbal_plant(;name, J=1.0, k_s=1.0, k_v=1.0, ω_brk=1.0, T_c=1.0)
    @variables t T(t) ω_rel(t) θ_rel(t) ω_body(t) ω(t)
    @parameters _J _k_s _k_v _ω_brk _T_c
    D = Differential(t)
    eqs = [
        D(θ_rel) ~ ω_rel
        ω ~ ω_rel + ω_body
        D(ω) ~ (T-_k_s*θ_rel-_k_v*ω_rel-_T_c*tanh(10*ω_rel/_ω_brk))/_J
    ]
    ODESystem(eqs, t; name, defaults=[ω=>0.0, ω_body=>0.0, ω_rel=>0.0,
                                      θ_rel=>0.0, T=>0.0,
                                      _T_c=>T_c, _ω_brk=>ω_brk,
                                      _J=>J, _k_s=>k_s, _k_v=>k_v])
end

export gimbal_plant


function gimbal_plant(;name, J=1.0, k_s=1.0, k_v=1.0, T_c=1.0, ω_brk=1.0, T_brk=1.0)
    @parameters t _J _k_s _k_v _T_c _ω_brk _T_brk 
    @variables T(t) ω_rel(t) θ_rel(t) ω_body(t) ω(t)
    D = Differential(t)
    eqs = [
        D(θ_rel) ~ ω_rel
        ω ~ ω_rel + ω_body
        D(ω) ~ (T-_k_s*θ_rel-_k_v*ω_rel
                -_T_c*tanh(10*ω_rel/_ω_brk)
                -sqrt(2ℯ)*(_T_brk-_T_c)*exp(-(ω_rel/(_ω_brk*sqrt(2)))^2)*(ω_rel/(_ω_brk*sqrt(2)))
                )/_J
    ]
    ODESystem(eqs, t; name, defaults =
        [ω=>0.0, ω_body=>0.0, ω_rel=>0.0,
         θ_rel=>0.0, T=>0.0,
         _T_c=>T_c, _T_brk=>T_brk, _ω_brk=>ω_brk,
         _J=>J, _k_s=>k_s, _k_v=>k_v])
end

export gimbal_plant

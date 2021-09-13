using DiffEqUncertainty, Distributions, ForwardDiff

stab_error(sol,index) = 1e3*sqrt(mean(abs2,cumsum(sol[index,:])))

#utilities
indexof(sym,syms) = findfirst(isequal(sym),syms)

function loss(θ;full=true)
    Z1,Z2,P2,Z3,P3,Z4 = θ
    prob, p , sys = gimbal_conntroller(;full,
                                       Z1, Z2, P2, Z3, P3, Z4,
                                       k_s=Uniform(0.2-0.04,0.2+0.04),
                                       k_v=Uniform(0.545-0.109,0.545+0.109),
                                       ω_brk=Uniform(0.074-0.0148,0.074+0.0148),
                                       T_c=Uniform(0.106-0.0212,0.106+0.0212),
                                       T_brk=Uniform(0.177-0.0354,0.177+0.0354))

    index = indexof(p.ω,states(sys))
    g(x) = stab_error(x,index)
    expectation(g, prob, prob.u0, prob.p,
                Koopman(), Rodas5(); ireltol=1e-1)[1]
    #expectation(g, prob, prob.u0, prob.p, MonteCarlo(), Rodas5(); trajectories = 100)
end

export loss

using DiffEqUncertainty, Distributions


stab_error(sol) = 1e6*sqrt(mean(abs2,cumsum(sol[26,1:end-1])))/1000

function temp()
    prob, p , _ = gimbal_conntroller(k_s=Uniform(0.2-0.04,0.2+0.04),
                                    k_v=Uniform(0.545-0.109,0.545+0.109),
                                    ω_brk=Uniform(0.074-0.0148,0.074+0.0148),
                                    T_c=Uniform(0.106-0.0212,0.106+0.0212),
                                    T_brk=Uniform(0.177-0.0354,0.177+0.0354)
                                    )

    expectation(stab_error, prob, prob.u0, prob.p,
                Koopman(), Rodas5();ireltol=1e-1)
    #expectation(g, prob, prob.u0, prob.p, MonteCarlo(), Rodas5(); trajectories = 100)
end

export temp, stab_error

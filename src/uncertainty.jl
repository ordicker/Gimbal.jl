using DiffEqUncertainty, Distributions
import ModelingToolkit: get_defaults, varmap_to_vars
#stab_error(sol,index) = 1e3*sqrt(mean(abs2,cumsum(sol[index,:])))
running_mean(sol,index) = 1e3*sqrt.(map(/,cumsum(cumsum(sol[index,:]).^2),1:length(sol)))
function stab_error(sol, index)
    θ = zero(eltype(sol))
    err2 = zero(eltype(sol))
    N=length(sol)
    @inbounds for ω in sol[index,:]
        θ+=ω
        err2+= abs2(θ)
    end
    1e3*sqrt(err2/N)
end

#utilities
indexof(sym::Num,syms) = findfirst(isequal(sym),syms)
indexof(sym::Symbol,syms) = findfirst(isequal(sym),[s.name for s in syms])

function loss(θ;full=true)
    Z1,Z2,P2,Z3,P3,Z4,K_dc = θ
    prob, p , sys = gimbal_conntroller(;full,
                                       Z1, Z2, P2, Z3, P3, Z4,K_dc)
    
    pnew = get_defaults(sys)
    pnew[p._k_s] = Uniform(0.2-0.04,0.2+0.04)
    pnew[p._k_v] = Uniform(0.545-0.109,0.545+0.109)
    pnew[p._ω_brk] = Uniform(0.074-0.0148,0.074+0.0148)
    pnew[p._T_c] = Uniform(0.106-0.0212,0.106+0.0212)
    pnew[p._T_brk] = Uniform(0.177-0.0354,0.177+0.0354)
    p_dist = varmap_to_vars(pnew,parameters(sys))

    index = indexof(p.ω,states(sys))
    g(x) = stab_error(x,index)
    expectation(g, prob, prob.u0, p_dist,
                Koopman(), Rodas5(); ireltol=1e-1)[1]
    #expectation(g, prob, prob.u0, prob.p, MonteCarlo(), Rodas5(); trajectories = 100)
end

export loss

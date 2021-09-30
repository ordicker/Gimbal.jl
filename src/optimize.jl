using ForwardDiff, NLopt

function loss_nlopt(x,∇)
    length(∇) > 0 ? ForwardDiff.gradient!(∇, loss,x) : nothing
    loss(x) #from uncertainty.jl
end

function gimbal_optimize()
    x₀ = [0.01,0.029,0.16,0.06,0.009,0.01]
    opt = Opt(:LD_MMA, 6)
    opt.lower_bounds = x₀*0.9
    opt.upper_bounds = x₀*1.1
    opt.xtol_rel = 1e-3
    #opt.maxtime = 10 #time limit in seconds
    opt.min_objective = loss_nlopt
    (minf,minx,ret) = NLopt.optimize(opt, x₀)
end

export gimbal_optimize

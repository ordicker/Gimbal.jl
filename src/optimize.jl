using ForwardDiff, NLopt

l = x->loss(x,full=false) #from uncertainty.jl

function loss_nlopt(x,∇)
    length(∇) > 0 ? ForwardDiff.gradient!(∇, l,x) : nothing
    l(x) 
end

function gimbal_optimize(x₀)
    #x₀ = [0.01,0.029,0.16,0.06,0.009,0.01,7000.0]
    #x₀ = [0.011,0.0319,0.0319,0.066,0.0081,0.011,7000.0]
    opt = Opt(:LD_MMA, 7)
    opt.lower_bounds = x₀*0.8
    opt.upper_bounds = x₀*1.2
    opt.xtol_rel = 1e-3
    #opt.maxtime = 10 #time limit in seconds
    opt.min_objective = loss_nlopt
    (minf,minx,ret) = NLopt.optimize(opt, x₀)
end

export gimbal_optimize

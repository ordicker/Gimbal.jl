using Gimbal
using Test
using BenchmarkTools
using ModelingToolkit
using DifferentialEquations


function plant(;name)
    @variables t F(t) x(t) dx(t)
    D = Differential(t)
    eqs = [
        D(x) ~ dx
        D(dx) ~ F - 10*dx - 20*x
    ]
    ODESystem(eqs, t;name, defaults=[x=>0.0, dx=>0.0])
end

function run_pid()
    @variables t
    @named pid1 = Gimbal.PID_factory(kp=350.0,ki=300.0,kd=50.0)
    @named p = plant()
    connections = [pid1.e ~ 1/(1+exp(-100*(t-1)))-p.x
                   p.F ~ pid1.u]
    @named connected = ODESystem(connections ; systems=[pid1,p])
    sys = structural_simplify(connected)
    prob = ODEProblem(sys,[],(0.0,10.0))
    return prob
end


@testset "pid-example" begin
    sol = solve(run_pid())#, ImplicitEuler(),adaptive=false, dt=4e-4)
    #plot(sol, vars = [p.x])#, pid1.e, pid1.u])
    steady_state = last(sol.u)[3]

    @test isapprox(steady_state, 1.0 ,atol=1e-4)
    time = @belapsed solve(run_pid())
    @test time < 10 #why?
end

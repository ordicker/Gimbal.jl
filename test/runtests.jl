using Gimbal
using Test
using BenchmarkTools

include("examples.jl")

@testset "pid-example" begin
    sol = solve(ex_pid())#, ImplicitEuler(),adaptive=false, dt=4e-4)
    #plot(sol, vars = [p.x])#, pid1.e, pid1.u])
    steady_state = last(sol.u)[3]

    @test isapprox(steady_state, 1.0 ,atol=1e-4)
    time = @belapsed solve(ex_pid())
    @test time < 10 #why?
end

@testset "lead lag-example" begin
    sol = solve(ex_lead_lag())
    steady_state = last(sol.u)[1]
    @test isapprox(steady_state, 1.0 ,rtol=0.02) #%2 steady state error
end

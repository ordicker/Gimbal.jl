# Gimbal.jl

## Installation
- julia 1.7.0
- clone the project
```sh
git clone https://github.com/ordicker/Gimbal.jl.git
```
- get into the folder
```sh
cd Gimbal.jl
```
- run julia (with the project)
```sh
julia --project=.
```
- instantiate
```julia
] instantiate
```
- done

## Quick start (first time is slow)
- copy paste example
```julia
using Gimbal,Plots,DifferentialEquations
prob,p,sys=gimbal_conntroller(full=false);
sol=solve(prob,Rodas5());
plot(sol[p.ω])
```

## TODOs
- [ ] Fix tests
- [X] Add K_dc to optimization
- [ ] switch to discrete model 
- [X] reduce allocation at gimbal_optimize (didn't help)
- [ ] add discontinuity [callbacks](https://mtk.sciml.ai/stable/basics/Composition/#Example:-Friction)
- [ ] better TTFX (precompile)
- [X] update to Julia 1.7 and MTK 8.X

## Notes
- MTK 8.3.0 broke the varmap_to_vars func with Distributions 

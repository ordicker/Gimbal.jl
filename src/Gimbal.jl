module Gimbal

using ModelingToolkit, DifferentialEquations

include("components.jl")
include("plant.jl")
include("full_system.jl")
include("uncertainty.jl")
include("optimize.jl")

end

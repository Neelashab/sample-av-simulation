using Test
using simple - simulation
using Random

@testset "all tests" begin
    include("test_utils.jl")
    include("test_lane_violations.jl")
    include("test_collisions.jl")
    include("test_trajectory.jl")
    include("test_planning.jl")
end


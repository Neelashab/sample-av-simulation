@testset "lane_lines" begin
    rng = MersenneTwister(1)
    for i = 1:20
        lane = simple - simulation.HalfSpace(randn(rng, 2), randn(rng))
        x = [randn(rng, 2); 0; 2π * rand(rng)]
        b = simple - simulation.VehicleProperties(rand(rng) * 4, rand(rng) * 3)
        ret = simple - simulation.check_lane_violations(x, b, [lane,])
        if i ∈ [6, 15, 16]
            @test ret == false
        else
            @test ret == true
        end
    end
end

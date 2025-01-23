@testset "collisions" begin
    rng = MersenneTwister(1)
    for i = 1:20
        x1 = [randn(rng, 2); 0; 2π * rand(rng)]
        b1 = simple - simulation.VehicleProperties(rand(rng) * 4, rand(rng) * 3)
        x2 = [randn(rng, 2); 0; 2π * rand(rng)]
        b2 = simple - simulation.VehicleProperties(rand(rng) * 4, rand(rng) * 3)
        ret = simple - simulation.check_collision(x1, b1, x2, b2)
        if i ∈ [6, 19, 20]
            @test ret == false
        else
            @test ret == true
        end
    end
end

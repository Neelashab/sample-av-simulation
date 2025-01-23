@testset "trajectory" begin
    rng = MersenneTwister(1)
    x = [0.0, 0, 0, 0]
    U = [randn(rng, 2) for _ in 1:10]
    b = simple - simulation.VehicleProperties(1.0, 1.0)
    T = simple - simulation.generate_trajectory(x, U, b)
    X = T.X
    d = randn(rng, 4)
    ret = sum(d'x for x in X; init=0.0)
    @test ret ≈ 9.181869142981148
end

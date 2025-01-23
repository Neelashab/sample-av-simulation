@testset "manual planning" begin
    (; trajectories, halfspaces) = setup_manual_planning_problem()
    ret = simple - simulation.check_trajectories(trajectories, halfspaces)
    @test ret
end

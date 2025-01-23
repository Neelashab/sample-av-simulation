function setup_manual_planning_problem()
    b1 = simple - simulation.VehicleProperties(2.0, 1.0)
    x₀1 = [-10.0, -2, 0.5, 0]
    b2 = simple - simulation.VehicleProperties(3.0, 1.25)
    x₀2 = [3.0, -2.0, 0.0, 0]
    b3 = simple - simulation.VehicleProperties(3.0, 1.25)
    x₀3 = [-18.0, -2.0, 2.0, 0]
    b4 = simple - simulation.VehicleProperties(3.0, 1.25)
    x₀4 = [-15.0, 2.0, 1.5, 0]

    U1 = simple - simulation.generate_collision_free_controls()
    U2 = [[0.0, 0] for _ in 1:35]
    U3 = [[[0.0, 0] for _ in 1:23]; [[-0.8, 0] for _ in 24:35]]
    U4 = [[0.0, 0] for _ in 1:35]

    T1 = simple - simulation.generate_trajectory(x₀1, U1, b1; Δ=0.25)
    T2 = simple - simulation.generate_trajectory(x₀2, U2, b2; Δ=0.25)
    T3 = simple - simulation.generate_trajectory(x₀3, U3, b3; Δ=0.25)
    T4 = simple - simulation.generate_trajectory(x₀4, U4, b4; Δ=0.25)

    hs1 = simple - simulation.HalfSpace([0.0, 1], -4.0)
    hs2 = simple - simulation.HalfSpace([0.0, -1], -4.0)
    trajectories = [T1, T2, T3, T4]
    halfspaces = [hs1, hs2]
    (; trajectories, halfspaces)
end


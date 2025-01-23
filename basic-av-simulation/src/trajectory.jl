# Defines the physical properties of a vehicle, such as half the length and width of its rectangular body.
struct VehicleProperties
    half_length::Float64  # Half the length of the vehicle (from center to front or rear).
    half_width::Float64   # Half the width of the vehicle (from center to either side).
end

# Represents the trajectory information for a vehicle.
struct Trajectory
    vehicle_properties::VehicleProperties  # The vehicle's physical properties.
    x₀::Vector{Float64}                    # Initial state of the vehicle: [x-position, y-position, velocity, orientation].
    X::Vector{Vector{Float64}}             # Sequence of states the vehicle moves through during the trajectory.
end

# Represents a half-space defined mathematically as {x: a'x ≥ b}, where `a` is the normal vector and `b` is the offset.
struct HalfSpace
    a::Vector{Float64} # Normal vector of the half-space boundary.
    b::Float64         # Offset value defining the half-space.
end

# Models the dynamics of a simple car. Given the current state `x` and control inputs `u`, returns the state derivative.
# State `x` has the form [x-position, y-position, velocity, orientation].
# Control inputs `u` have the form [acceleration, angular velocity].
function f_simple_car(x, u)
    [cos(x[4]) * x[3],  # Rate of change of x-position (x-velocity).
        sin(x[4]) * x[3],  # Rate of change of y-position (y-velocity).
        u[1],              # Rate of change of velocity (acceleration).
        u[2]]              # Rate of change of orientation (angular velocity).
end

# Generates a trajectory for the vehicle starting from an initial state `x₀` and applying a sequence of control inputs `U`.
# Each control input in `U` is applied for a time step of `Δ` seconds.
function generate_trajectory(x₀, U, vehicle_properties; Δ=0.25)
    X = Vector{Vector}()                   # Initialize an empty list to store the trajectory states.
    current_state = x₀                     # Start with the initial state.
    for u in U
        next_state = current_state + Δ * f_simple_car(current_state, u) # Update state using car dynamics.
        push!(X, next_state)               # Add the new state to the trajectory.
        current_state = next_state         # Update the current state for the next iteration.
    end
    return Trajectory(vehicle_properties, x₀, X)  # Return the generated trajectory.
end

# Generates a set of control inputs that ensures the vehicle does not collide with others.
function generate_collision_free_controls()
    # Start with moderate acceleration and slight right turns, then adjust with slow left turns, and finally stop.
    U = [[1.5, π / 20] for _ in 1:10]
    append!(U, [[0.30, -π / 20] for _ in 1:10])
    append!(U, [[0.0, 0.0] for _ in 1:5])  # Stop for the final steps.
    return U
end

# Computes the coordinates of the four corners of a rectangular vehicle given its center, orientation, and dimensions.
function compute_vehicle_corners(center::Vector{Float64}, theta::Float64, half_length::Float64, half_width::Float64)
    # Offsets for the corners in the local coordinate frame (relative to the vehicle center).
    offsets = [
        [+half_length, +half_width],
        [+half_length, -half_width],
        [-half_length, -half_width],
        [-half_length, +half_width]
    ]
    # Rotation matrix to account for the vehicle's orientation `theta`.
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)]
    # Rotate and translate each offset to compute the global coordinates of the corners.
    return [center + R * offset for offset in offsets]
end

# Checks if two vehicles, defined by their states and physical properties, are in collision.
function check_collision(x₁::Vector{Float64}, b₁::VehicleProperties, x₂::Vector{Float64}, b₂::VehicleProperties)
    # Compute the corners of each vehicle based on their states and dimensions.
    corners₁ = compute_vehicle_corners(x₁[1:2], x₁[4], b₁.half_length, b₁.half_width)
    corners₂ = compute_vehicle_corners(x₂[1:2], x₂[4], b₂.half_length, b₂.half_width)

    # Check if any corner of one vehicle is inside the other.
    for corner in corners₁
        if is_point_in_rectangle(corner, corners₂)
            return true
        end
    end
    for corner in corners₂
        if is_point_in_rectangle(corner, corners₁)
            return true
        end
    end

    # Check if any edges of the two vehicles intersect.
    for i in 1:4
        for j in 1:4
            if edges_intersect(corners₁[i], corners₁[mod(i, 4)+1], corners₂[j], corners₂[mod(j, 4)+1])
                return true
            end
        end
    end

    return false  # No collision detected.
end

# Helper function to check if two line segments intersect.
function edges_intersect(p1, p2, q1, q2)
    cross(v1, v2) = v1[1] * v2[2] - v1[2] * v2[1]  # Cross product of two vectors.
    r = p2 - p1  # Direction vector of the first segment.
    s = q2 - q1  # Direction vector of the second segment.
    qp = q1 - p1  # Vector between the start points of the segments.
    denominator = cross(r, s)

    if denominator == 0  # Parallel or collinear segments.
        return false
    end

    t = cross(qp, s) / denominator
    u = cross(qp, r) / denominator

    # Check if the intersection point lies within both segments.
    return 0 ≤ t ≤ 1 && 0 ≤ u ≤ 1
end

# Checks if a point is inside a rectangle defined by its corners.
function is_point_in_rectangle(point, corners)
    A, B, C, D = corners  # Unpack the corners of the rectangle.
    AB = B - A            # Vector from A to B.
    AD = D - A            # Vector from A to D.
    AP = point - A        # Vector from A to the point.
    # Check if the point lies within the bounds defined by AB and AD.
    return 0 ≤ dot(AP, AB) ≤ dot(AB, AB) && 0 ≤ dot(AP, AD) ≤ dot(AD, AD)
end

# Checks if a vehicle is violating lane constraints defined by a list of half-spaces.
function check_lane_violations(x::Vector{Float64}, b::VehicleProperties, L::Vector{HalfSpace})
    center = x[1:2]  # Vehicle's position.
    corners = compute_vehicle_corners(center, x[4], b.half_length, b.half_width)

    for half_space in L
        # Check if all corners of the vehicle are inside the half-space.
        if all(dot(half_space.a, corner) >= half_space.b for corner in corners)
            continue
        else
            return true  # At least one corner violates the half-space constraint.
        end
    end

    return false  # No lane violations.
end

# Checks if a set of trajectories are collision-free and satisfy lane constraints.
function check_trajectories(trajectories::Vector{Trajectory}, lanes::Vector{HalfSpace})
    states = [t.X for t in trajectories]  # Extract states for all trajectories.
    for xs in zip(states...)  # Check each time step across trajectories.
        for (e1, x1) in enumerate(xs)
            if check_lane_violations(x1, trajectories[e1].vehicle_properties, lanes)
                return false  # Lane violation detected.
            end
            for (e2, x2) in enumerate(xs)
                if e2 ≤ e1
                    continue  # Skip self-checks and redundant comparisons.
                else
                    if check_collision(x1, trajectories[e1].vehicle_properties, x2, trajectories[e2].vehicle_properties)
                        return false  # Collision detected.
                    end
                end
            end
        end
    end
    return true  # All trajectories are collision-free and lane-compliant.
end

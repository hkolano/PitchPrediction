using JLD
using Random
using StaticArrays, RigidBodyDynamics, Rotations


# ----------------------------------------------------------
#                         Definitions
# ----------------------------------------------------------
num_its=20
num_actuated_dofs = 5
num_trajectory_dofs = 4
max_duration = 25.
θa = atan(145.3, 40)
joint_lims = [[-175*pi/180, 175*pi/180], [0, 200*pi/180], [0, 200*pi/180], [-165*pi/180, 165*pi/180], [0.0, 0.022]]
# Velocity limits: 30 degrees/s for Joints E, D, C, 50 degrees/s for Joint B (from Reach Alpha Integration Manual V1.15 p8)
θb = deg2rad(30)
θc = deg2rad(50)
vel_lims = [[-θb, θb], [-θb, θb], [-θb, θb], [-θc, θc], [-.003, .003]]
max_linear_vel = .05 # 1 cm/s

equil_pose = zeros(num_actuated_dofs)
equil_pt = jointState(equil_pose, zeros(num_actuated_dofs))
extended_pt = jointState([0.01, 1.5, 2.6, 0.01, 0.], zeros(num_actuated_dofs))

# raise_wpts = Waypoints(equil_pt, extended_pt)

mutable struct worldSpaceWaypoints
    start_pose
    end_pose
end

struct trajParams
    a::Array 
    pts:: worldSpaceWaypoints
    T::Float64
end



# ----------------------------------------------------------
#             Waypoint Generation (World Space)
# ----------------------------------------------------------
function generate_random_pose(mech::Mechanism)
    base_frame = root_frame(mech)
    cartesian_bound = 0.5
    rand_scalings = [rand(.1: .01: cartesian_bound) for i in 1:3]
    rand_frame = CartesianFrame3D("rand_frame")
    rand_trans = Random.rand(Transform3D, base_frame, rand_frame)
    new_translation = rand_trans.mat[1:3, 4] .* rand_scalings
    mod_trans = Transform3D(rand_frame, base_frame, rotation(rand_trans), SVector{3, Float64}(new_translation))
    return mod_trans
end

function generate_path_from_current_pose(state::MechanismState)
    base_frame = root_frame(state.mechanism)
    jaw_frame = default_frame(bodies(state.mechanism)[end])
    worldSpaceWaypoints(inv(relative_transform(state, base_frame, jaw_frame)), generate_random_pose(state.mechanism))
end

function get_T_at_s(wp::worldSpaceWaypoints, s::Float64)
    p_start = translation(wp.start_pose) 
    p_end = translation(wp.end_pose) 
    R_start = rotation(wp.start_pose)
    R_end = rotation(wp.end_pose)

    p_at_s = p_start + s*(p_end - p_start)
    R_at_s = R_start*exp(log(transpose(R_start)*R_end)*s)
    T_at_s = Transform3D(wp.start_pose.from, wp.start_pose.to, R_at_s, p_at_s)
end

function get_Tdot_at_sdot(wp::worldSpaceWaypoints, s, sdot)
    Tdot = get_T_at_s(wp, s)
    p_start = translation(wp.start_pose) 
    p_end = translation(wp.end_pose) 
    R_start = rotation(wp.start_pose)
    R_end = rotation(wp.end_pose)

    pdot = sdot*(p_end - p_start)
    logRTR = log(transpose(R_start)*R_end)
    Rdot = Rotations.RotMatrix(R_start*exp(logRTR*s)*logRTR*sdot)
    # println(R_start*exp(logRTR*s)*logRTR*sdot)
    Tdot_at_s = Transform3D(wp.start_pose.from, wp.start_pose.to, Rdot, pdot)
end

# ----------------------------------------------------------
#               Trajectory Generation (World Space)
# ----------------------------------------------------------

function get_coeffs(T)
    a3 = 10/(T^3)
    a4 = -15/(T^4)
    a5 = 6/(T^5)
    return [0, 0, 0, a3, a4, a5]
end

function get_s(t, a) 
    a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5 
end
function get_ds(t, a) 
    a[2] + 2a[3]*t + 3a[4]*t^2 + 4a[5]*t^3 + 5a[6]*t^4
end
function get_dds(t, a) 
    2a[3] + 6a[4]*t + 12a[5]*t^2 + 20a[6]*t^3
end

function get_pose_list(pts, a, T, num_its=num_its)
    dt = T/num_its 
    poses = Array{Transform3D}(undef, num_its)
    vels = Array{Transform3D}(undef, num_its)

    for i = 1:num_its
        t_current = dt*i    # current time
        s = get_s(t_current, a)
        ds = get_ds(t_current, a)
        poses[i] = get_T_at_s(pts, s)
        # display(poses[i].mat)
        vels[i] = get_Tdot_at_sdot(pts, s, ds)
    end
    return poses, vels 
end

function find_trajectory(pts::worldSpaceWaypoints; T_init=1.0)
    T = T_init
    within_vel_bounds = false
    a = Vector{Float64}(undef, 6)

    # Increase trajectory duration until the maximum speed is not exceeded (or until it's longer than 15 seconds)
    while within_vel_bounds == false && T < max_duration
        max_lin_vels = 15/(8*T)*(translation(pts.end_pose)-translation(pts.start_pose))
        if maximum(abs.(max_lin_vels)) > max_linear_vel
            T = T + 0.2
        else 
            a = get_coeffs(T)
            println(a)
            within_vel_bounds = true
        end
    end
    println(T)

    # If a valid trajectory was found, get time series of poses and velocities
    if within_vel_bounds == true 
        poses, vels = get_pose_list(pts, a, T)
        return [a, T, poses, vels]
    else
        println("No path between points; try again.")
        return nothing
    end

end

function get_des_state_at_t(t, traj::trajParams)
    s = get_s(t, traj.a)
    ds = get_ds(t, traj.a)
    des_pose = get_T_at_s(traj.pts, s)
    des_vel = get_Tdot_at_sdot(traj.pts, s, ds)
    return des_pose, des_vel
end
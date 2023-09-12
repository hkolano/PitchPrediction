using JLD
using Random
using StaticArrays, RigidBodyDynamics, Rotations

mutable struct jointState
    θs::Array{Float64}
    dθs::Array{Float64}
end

mutable struct Waypoints
    start::jointState 
    goal::jointState
end

mutable struct quinticTrajParams
    a::Array        # Array of time scaling coefficients (quintic traj)
    wp::Waypoints   # Waypoints the traj is going between
    T::Float64      # Duration of the trajectory
end

num_trajectory_dofs = num_velocities(mech_blue_alpha)-6

equil_pose = zeros(num_trajectory_dofs)
equil_pt = jointState(equil_pose, zeros(num_trajectory_dofs))
extended_pt = jointState([0.01, 1.5, 2.6, 0.01, 0.], zeros(num_trajectory_dofs))

# ----------------------------------------------------------
#               Point Generation (Joint Space)
# ----------------------------------------------------------
""" 
    gen_rand_feasible_point()

Randomly generates a jointState, checking for joint and velocity limits.
"""
function gen_rand_feasible_point()
    θs = Array{Float64}(undef,num_trajectory_dofs)    
    dθs = Array{Float64}(undef,num_trajectory_dofs)    
    for jt_idx in  1:num_trajectory_dofs
        dof_name = dof_names[jt_idx+6]
        θs[jt_idx] = rand(joint_lim_dict[dof_name][1]:.001:joint_lim_dict[dof_name][2])
        dθs[jt_idx] = rand(vel_lim_dict[dof_name][1]:.001:vel_lim_dict[dof_name][2])
    end
    return jointState(θs, dθs)
end

"""
    gen_rand_feasible_point_at_rest()

Randomly generates a jointState where the velocity is 0. 
"""
function gen_rand_feasible_point_at_rest()
    jS = gen_rand_feasible_point()
    jS.dθs = zeros(num_trajectory_dofs)
    return jS
end

# ----------------------------------------------------------
#             Waypoint Generation (Joint Space)
# ----------------------------------------------------------
"""
Generates a random Waypoint struct. Not advisable to start a trajectory.  
"""
function gen_rand_waypoints()
    Waypoints(gen_rand_feasible_point(), gen_rand_feasible_point())    
end

"""
Generates a random Waypoint struct, starting at rest at the home position. 
"""
function gen_rand_waypoints_from_equil()
    Waypoints(equil_pt, gen_rand_feasible_point()) 
end

"""
    set_waypoint_from_equil(θs, dθs)

Generates a Waypoint struct. Starts at rest at the home position; ...
ends at the provided position (θs) and velocity (dθs) of the arm.
"""
function set_waypoints_from_equil(θs, dθs)
    Waypoints(equil_pt, jointState(θs, dθs))
end

function gen_rand_waypoints_to_rest()
    Waypoints(equil_pt, gen_rand_feasible_point_at_rest())
end

function gen_rand_waypoints_at_rest()
    wp = Waypoints(gen_rand_feasible_point_at_rest(), gen_rand_feasible_point_at_rest())
    return wp
end

function gen_rand_waypoint_from_start(Js::jointState)
    Waypoints(Js, gen_rand_feasible_point_at_rest())
end

"""
    save_waypoints(wp::Waypoints, name::String)

Save an already generated waypoint to a file. The file will be located in "src/tmp/", ...
and will be called [name].jld
"""
function save_waypoints(wp::Waypoints, name::String)
    save(string("src/tmp/", name, ".jld"), "start_θs", wp.start.θs, "start_dθs", wp.start.dθs, "end_θs", wp.goal.θs, "end_dθs", wp.goal.dθs)
end

""" 
    load_waypoints(name::String)

Load a waypoint from a file. The file must be located in "src/tmp/" and be in JLD file format.
"""
function load_waypoints(name::String)
    wp_raw = load(string("src/tmp/", name, ".jld"))
    new_wp = Waypoints(jointState(wp_raw["start_θs"], wp_raw["start_dθs"]), jointState(wp_raw["end_θs"], wp_raw["end_dθs"]))
    return new_wp 
end

# ----------------------------------------------------------
#               Trajectory Generation (Joint Space)
# ----------------------------------------------------------
""" 
    get_coeffs(pts::Waypoints, T, idx)

Given a set of waypoints and a time scaling, determine the time scaling coefficients
for a quintic trajectory. 
"""
function get_coeffs(pts::Waypoints, T, idx)
    # λ1 = dθ1/(θ2-θ1)
    λ1 = pts.start.dθs[idx]/(pts.goal.θs[idx]-pts.start.θs[idx])
    # λ2 = dθ2/(θ2-θ1)
    λ2 = pts.goal.dθs[idx]/(pts.goal.θs[idx]-pts.start.θs[idx])

    a0 = 0
    a1 = λ1
    a2 = 0
    a3 = -2(3T*λ1+2T*λ2-5)/(T^3)
    a4 = (8T*λ1 + 7T*λ2-15)/(T^4)
    a5 = -3(T*λ1+T*λ2-2)/(T^5)
    return [a0, a1, a2, a3, a4, a5]
end

function pos_scale_at_t(a, t)
    s = a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5
end

function vel_scale_at_t(a, t)
    ds = a[2] + 2a[3]*t + 3a[4]*t^2 + 4a[5]*t^3 + 5a[6]*t^4
end

function acc_scale_at_t(a, t)
    dds = 2a[3] + 6a[4]*t + 12a[5]*t^2 + 20a[6]*t^3
end

"""
    get_path!(poses, vels, θ1, θ2, T, a, num_its)

Gets the poses and velocities of the manipulator joints along a potential desired trajectory. 
Used to check whether the proposed duration exceeds the position or velocity limits of the joints. 
"""
function get_path!(poses, vels, θ1, θ2, T, a, num_its=num_its)
    dt = T/num_its

    for i = 1:num_its
        t = dt*i
        s = pos_scale_at_t(a, t)
        ds = vel_scale_at_t(a, t)
        poses[i] = θ1 + s*(θ2-θ1)
        vels[i] = ds*(θ2-θ1)
    end
    return poses, vels 
end

"""
    get_desv_at_t(t, p)
    
Given a set of trajectory parameters p and a time t, determine what the desired 
velocity is. 

If the parameters is an array of parameters, this handles swiching between parameters
at the input time. 
"""
function get_desv_at_t(t, p::quinticTrajParams)
    # println("Got request for desv. Params $(p))")
    des_vel = zeros(num_velocities(mech_blue_alpha)-6)
    # des_vel[1] = 0.05
    if t <= p.T # If the current time is less than the trajectory duration
        for i = 1:num_trajectory_dofs # des vel for last joint is always 0
            ds = vel_scale_at_t(p.a[i,:], t)
            des_vel[i] = ds*(p.wp.goal.θs[i]-p.wp.start.θs[i])
        end
    end
    # fill!(des_vel, 0)
    return des_vel
end

function get_desv_at_t(t, p_array::Array{quinticTrajParams})
    des_vel = zeros(num_velocities(mech_blue_alpha)-6)
    traj_num = 1
    for i in 1:length(params)
        if t > swap_times[i]
            traj_num += 1
        end
    end
    t_mod = traj_num ==1 ? t : t-swap_times[traj_num-1]

    if traj_num <= length(params)
        p = p_array[traj_num]
    

        if t_mod <= p.T # If the current time is less than the trajectory duration
            for i = 1:num_trajectory_dofs # des vel for last joint is always 0
                ds = vel_scale_at_t(p.a[i,:], t_mod)
                des_vel[i] = ds*(p.wp.goal.θs[i]-p.wp.start.θs[i])
            end
        end
    end
    # fill!(des_vel, 0)
    des_vel[end] = 0.0
    return des_vel
end

"""
    get_desq_at_t(t, p)
    
Given a set of trajectory parameters p and a time t, determine what the desired 
joint positions are. 

If the input for p is an array of parameters, this handles swiching between parameters
at the input time. 
"""
function get_desq_at_t(t, p::quinticTrajParams)
    des_qs = zeros(num_velocities(mech_blue_alpha)-6)
    if t < p.T 
        for i = 1:num_trajectory_dofs
            s = pos_scale_at_t(p.a[i,:], t)
            des_qs[i] = p.wp.start.θs[i] + s*(p.wp.goal.θs[i]-p.wp.start.θs[i])
        end
    end
    return des_qs
end

function get_desq_at_t(t, p_array::Array{quinticTrajParams})
    des_qs = zeros(num_trajectory_dofs+4)
    traj_num = 1
    for i in 1:length(params)
        if t > swap_times[i]
            traj_num += 1
        end
    end
    t_mod = traj_num ==1 ? t : t-swap_times[traj_num-1]

    if traj_num <= length(params)
        p = p_array[traj_num]

        if t_mod < p.T 
            for i = 1:num_trajectory_dofs
                s = pos_scale_at_t(p.a[i,:], t_mod)
                des_qs[i+4] = p.wp.start.θs[i] + s*(p.wp.goal.θs[i]-p.wp.start.θs[i])
            end
        end
    end
    des_qs[end] = 0.0
    return des_qs
end

function check_lim(vals::Array, lims)
    if minimum(vals) < lims[1] || maximum(vals) > lims[2]
        is_in_range = false
    else
        is_in_range = true
    end
    return is_in_range
end

"""
    scale_trajectory(params, poses, vels, max_scale)

Increases the time scaling of the trajectory by a factor between 1 and max_scale.
"""
function scale_trajectory(params::quinticTrajParams, poses, vels, max_scale)
    a = Array{Float64}(undef, num_trajectory_dofs, 6)
    scale_factor = rand(1:.01:max_scale)
    T = params.T*scale_factor
    # println("Scaling factor: $(scale_factor)")
    for i in 1:num_trajectory_dofs
        a[i,:] = get_coeffs(params.wp, T, i)
        (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], params.wp.start.θs[i], params.wp.goal.θs[i], T, a[i,:])
    end
    return [quinticTrajParams(a, params.wp, T), poses, vels]
end

"""
    find_quintic_trajectory(pts, num_its, T_init)

Get the parameters for a quintic trajectory between the two given waypoints. 

...
# Arguments
'pts::Waypoints':   Two points in joint space, specified as a Waypoint struct
'num_its=50':   Number of points in the output poses and vels vectors 
'T_init=1':   Minimum trajectory duration
...
"""
function find_quintic_trajectory(pts::Waypoints; num_its=num_its, T_init=1.0)
    T = T_init
    poses = Array{Float64}(undef, num_its, num_trajectory_dofs)
    vels = Array{Float64}(undef, num_its, num_trajectory_dofs)
    feasible_ct = 0
    a = Array{Float64}(undef, num_trajectory_dofs, 6)

    while feasible_ct < num_trajectory_dofs && T < max_traj_duration
        feasible_ct = 0
        for i in 1:num_trajectory_dofs
            dof_name = dof_names[i+6]
            # println("Joint $(i)")
            # Get trajectory 
            # @show i
            # @show get_coeffs(pts, T, i)
            a[i,:] = get_coeffs(pts, T, i)
            (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], pts.start.θs[i], pts.goal.θs[i], T, a[i,:])

            # Evaluate if possible
            is_in_range_poses = check_lim(poses[:,i], joint_lim_dict[dof_name])
            is_in_range_vels = check_lim(vels[:,i], vel_lim_dict[dof_name])
            # println("OK poses = $(is_in_range_poses), OK vels = $(is_in_range_vels)")
            if is_in_range_poses == true && is_in_range_vels == true
                feasible_ct = feasible_ct + 1
            # else
                # println("Max joint: $(maximum(poses[:,i])), min joint: $(minimum(poses[:,i]))")
            end
        end
        if feasible_ct < num_trajectory_dofs
            T = T + 0.2
        end
    end

    if feasible_ct == num_trajectory_dofs
        return quinticTrajParams(a, pts, T), poses, vels
        # println("Trajectory parameters set")
    else
        # println("No path between points; try again.")
        return "nothing", "nada"
    end

end

"""
    define_random_trajectory()
"""
function define_random_trajectory()

    local traj
   
    while true
        new_wp = gen_rand_waypoints_at_rest()
        # println(typeof(new_wp))
        traj = find_quintic_trajectory(new_wp)
        # println(traj)
        if typeof(traj[1]) == quinticTrajParams
            # println("found traj")
            break 
        end
    end

    # println(traj)
    if do_scale_traj == true
        traj = scale_trajectory(traj..., max_traj_scaling)
    end
    return traj
end

"""
    define_multiple_waypoints!(params, swap_times, max_trajs)

Inputs:
    params: an empty array of type quinticTrajParams[]
    swap_times: an empty vector of type Float64 
    max_trajs: the maximum number of waypoints to plan trajectories to 

Gets the parameters to execute some number of quintic trajectories to random 
zero-velocity waypoints in the joint space. Stores this information in params 
and swap_times.
"""
function define_multiple_waypoints!(params, swap_times, max_trajs)
    wp_list = Waypoints[]
    traj_list = Any[]
    scaled_traj_list = Any[]

    wp = gen_rand_waypoints_to_rest()
    traj = find_quintic_trajectory(wp) 

    # # Keep trying until a good trajectory is found
    while traj === nothing
        global wp = gen_rand_waypoints_to_rest()
        global traj = find_quintic_trajectory(wp)
    end

    push!(wp_list, wp)
    push!(traj_list, traj)

    if max_trajs > 1
        for i in 1:rand(1:1:max_trajs-1)
            new_traj = nothing
            new_wp = nothing
            while new_traj === nothing 
                new_wp = gen_rand_waypoint_from_start(wp_list[end].goal)
                new_traj = find_quintic_trajectory(new_wp)
            end
            push!(wp_list, new_wp)
            push!(traj_list, new_traj)
        end
    end

    # @show 

    # # Scale that trajectory to 1x-2x "top speed"
    if do_scale_traj == true
        for traj in traj_list
            # last argument is maximum time scaling factor
            push!(scaled_traj_list, scale_trajectory(traj..., 2))
        end
    else
        scaled_traj_list = traj_list
    end
    
    duration = 0
    for traj in scaled_traj_list
        push!(params, traj[1])
        duration += traj[1].T
        push!(swap_times, duration)
    end
end

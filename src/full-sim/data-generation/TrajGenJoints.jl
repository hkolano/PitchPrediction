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

mutable struct trajParams
    a::Array        # Array of time scaling coefficients (quintic traj)
    wp::Waypoints   # Waypoints the traj is going between
    T::Float64      # Duration of the trajectory
end

num_its=50
num_actuated_dofs = 4
num_trajectory_dofs = 4
max_duration = 20.
θa = atan(145.3, 40)
joint_lims = [[-175*pi/180, 175*pi/180], [0, 200*pi/180], [0, 200*pi/180], [-165*pi/180, 165*pi/180], [0.0, 0.022]]
# Velocity limits: 30 degrees/s for Joints E, D, C, 50 degrees/s for Joint B (from Reach Alpha Integration Manual V1.15 p8)
θb = deg2rad(30)
θc = deg2rad(50)
vel_lims = [[-θb, θb], [-θb, θb], [-θb, θb], [-θc, θc], [-.003, .003]]
max_linear_vel = .15 # 15 cm/s

equil_pose = zeros(num_actuated_dofs)
equil_pt = jointState(equil_pose, zeros(num_actuated_dofs))
extended_pt = jointState([0.01, 1.5, 2.6, 0.01, 0.], zeros(num_actuated_dofs))

# ----------------------------------------------------------
#               Point Generation (Joint Space)
# ----------------------------------------------------------
""" 
    gen_rand_feasible_point()
Randomly generates a jointState, checking for joint and velocity limits.
"""
function gen_rand_feasible_point()
    θs = Array{Float64}(undef,num_actuated_dofs)    
    dθs = Array{Float64}(undef,num_actuated_dofs)    
    for jt_idx in  1:num_actuated_dofs
        θs[jt_idx] = rand(joint_lims[jt_idx][1]:.001:joint_lims[jt_idx][2])
        dθs[jt_idx] = rand(vel_lims[jt_idx][1]:.001:vel_lims[jt_idx][2])
    end
    return jointState(θs, dθs)
end

"""
    gen_rand_feasible_point_at_rest()
Randomly generates a jointState where the velocity is 0. 
"""
function gen_rand_feasible_point_at_rest()
    jS = gen_rand_feasible_point()
    jS.dθs = [0.0 0.0 0.0 0.0]
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
    get_desv_at_t(t, p)
    
Given a set of trajectory parameters p and a time t, determine what the desired 
velocity is. 
"""
function get_desv_at_t(t, p)
    # println("Got request for desv. Params $(p))")
    des_vel = zeros(9)
    if t <= p.T # If the current time is less than the trajectory duration
        for i = 1:num_trajectory_dofs # des vel for last joint is always 0
            ds = vel_scale_at_t(p.a[i,:], t)
            des_vel[i+4] = ds*(p.wp.goal.θs[i]-p.wp.start.θs[i])
        end
    end
    fill!(des_vel, 0)
    return des_vel
end

function check_lim(vals::Array, lims, idx)
    if minimum(vals) < lims[idx][1] || maximum(vals) > lims[idx][2]
        is_in_range = false
    else
        is_in_range = true
    end
    return is_in_range
end

function scale_trajectory(params, poses, vels)
    a = Array{Float64}(undef, num_trajectory_dofs, 6)
    scale_factor = rand(1:.01:3)
    T = params.T*scale_factor
    # println("Scaling factor: $(scale_factor)")
    for i in 1:num_trajectory_dofs
        a[i,:] = get_coeffs(params.wp, T, i)
        (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], params.wp.start.θs[i], params.wp.goal.θs[i], T, a[i,:])
    end
    return [trajParams(a, params.wp, T), poses, vels]
end


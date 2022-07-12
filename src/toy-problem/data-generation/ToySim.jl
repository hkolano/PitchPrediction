# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
#%%
using RigidBodyDynamics
using LinearAlgebra, StaticArrays
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Revise
using Plots, CSV, Tables, ProgressBars
using PitchPrediction
println("Libraries imported.")

src_dir = dirname(pathof(PitchPrediction))
urdf_file = joinpath(src_dir, "..", "urdf", "toy_vehicle.urdf")
ctlr_file = joinpath(src_dir, "toy-problem", "PIDCtlr.jl")
traj_file = joinpath(src_dir, "toy-problem", "TrajGen.jl")
simulate_file = joinpath(src_dir, "toy-problem", "simulate_des_trajectory.jl")

# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
#%%
vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file; gravity = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, URDFVisuals(urdf_file), vis[:toy])
render(mvis_toy)

# Initialize the mechanism state
state = MechanismState(mechanism_toy)
free_joint, joint1, joint2 = joints(mechanism_toy)
Δt = 1e-3

# Output data at 50 Hz
goal_freq = 50
sample_rate = Int(floor((1/Δt)/goal_freq))

# ----------------------------------------------------------
#                         Functions
# ----------------------------------------------------------
#%%
function reset_to_equilibrium(state)
    set_configuration!(state, [0.18558, -0.18558, 0.0])
    set_velocity!(state, [0.0, 0.0, 0.0])
end

include(ctlr_file)
include(traj_file)
include(simulate_file)

# ----------------------------------------------------------
#                       Generate Data
# ----------------------------------------------------------
num_trajs = 5000

# Create (num_trajs) different trajectories and save to csvs
for n in ProgressBar(1:num_trajs)
    # Set to initial position 
    reset_to_equilibrium(state)

    # Set up the controller 
    ctlr_cache = PIDCtlr.CtlrCache(Δt, mechanism_toy)

    # Get a random waypoint set and see if there's a valid trajectory
    wp = TrajGen.gen_rand_waypoints_from_equil()
    traj = TrajGen.find_trajectory(wp)

    # If can't find a valid trajectory, try new waypoints until one is found
    while traj === nothing
        wp = TrajGen.gen_rand_waypoints_from_equil()
        traj = TrajGen.find_trajectory(wp)
    end

    # Scale that trajectory to 1x-5x "top speed"
    scaled_traj = TrajGen.scale_trajectory(traj...)
    params = scaled_traj[1]
    duration = scaled_traj[2]
    poses = scaled_traj[3]
    vels = scaled_traj[4]
    # println("Going to point $(wp.goal.θs)")

    # Make vector of waypoint values and time step to save to csv
    waypoints = [Δt*sample_rate params.wp.start.θs... params.wp.goal.θs... params.wp.start.dθs... params.wp.goal.dθs...]
    wp_data = Tables.table(waypoints)

    # Save waypoints (start and goal positions, velocities) to CSV file
    if n == 1
        goal_headers = ["dt", "J1_start", "J2_start", "J1_end", "J2_end", "dJ1_start", "dJ2_start", "dJ1_end", "dJ2_end"]
        CSV.write("data/toy-data-waypoints.csv", wp_data, header=goal_headers)
    else 
        CSV.write("data/toy-data-waypoints.csv", wp_data, header=false, append=true)
    end

    # Run the simulation
    ts, qs, vs = simulate_des_trajectory(state, duration, params, ctlr_cache, PIDCtlr.pid_control!; Δt);

    # Break out each variable (probably better way to do this)
    # downsample to 50 Hz
    # println("Sample rate: $(sample_rate)")
    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
    qs1 = [qs[i][1] for i in 1:sample_rate:length(qs)]
    qs3 = [qs[i][3] for i in 1:sample_rate:length(qs)]
    qs2 = [qs[i][2] for i in 1:sample_rate:length(qs)]
    vs1 = [vs[i][1] for i in 1:sample_rate:length(vs)]
    vs2 = [vs[i][2] for i in 1:sample_rate:length(vs)]
    vs3 = [vs[i][3] for i in 1:sample_rate:length(vs)]

    # Write the 
    num_rows = 6
    data = Array{Float64}(undef, length(ts_down), num_rows)
    cols = [qs1, qs2, qs3, vs1, vs2, vs3]
    labels = ["pitch", "joint1", "joint2", "d_pitch", "vel_j1", "vel_j2"]
    for (idx, val) in enumerate(cols)
        data[:,idx] = val
    end
    
    tab = Tables.table(data)
    CSV.write("data/toy-data/toystates$(n).csv", tab, header=labels)
    
    # MeshCatMechanisms.animate(mvis_toy, ts, qs; realtimerate = 2.5);
end

# ----------------------------------------------------------
#                      Visualization
# ----------------------------------------------------------


# Show the animation


# Plot actual and desired joint angles and velocities
# function plot_state_errors()
    # l = @layout [a b ; c d]
    # # label = ["q2", "q3", "v2", "v3"]
    # p1 = plot(ts, qs2, label="q2", ylim=(-1.5, 1.0))
    # p1 = plot!(LinRange(0,duration,50), poses[:,1], label="des_q2")
    # p2 = plot(ts, qs3, label="q3",  ylim=(-1.5, 1.5))
    # p2 = plot!(LinRange(0, duration, 50), poses[:,2], label="des_q3")
    # p3 = plot(ts, vs2, label="v2",  ylim=(-.6, 0.6))
    # p3 = plot!(LinRange(0, duration, 50), vels[:,1], label="des_v2")
    # p4 = plot(ts, vs3, label="v3",  ylim=(-.6, 0.6))
    # p4 = plot!(LinRange(0, duration, 50), vels[:,2], label="des_v3")
    # plot(p1, p2, p3, p4, layout=l)
# end

# plot(ts[1:50], ctlr_cache.taus[1,1:50])
#%%

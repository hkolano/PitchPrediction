function plot_desired_and_actual_poses(traj_pars, qs, ts_down)
    # Get list of desired poses and velocities
    des_poses, des_vs = get_pose_list(traj_pars.pts, traj_pars.a, traj_pars.T, length(ts_down))
            
    # separate desired translations
    des_translations = [translation(des_poses[i]) for i in eachindex(des_poses)]
    des_xs = [des_translations[i][1] for i in eachindex(des_translations)]
    des_ys = [des_translations[i][2] for i in eachindex(des_translations)]
    des_zs = [des_translations[i][3] for i in eachindex(des_translations)]

    # separate desired rotations
    des_rots = [RotXYZ(rotation(des_poses[i])) for i in eachindex(des_poses)]
    des_rolls = [des_rots[i].theta1 for i in eachindex(des_rots)]
    des_pitches = [des_rots[i].theta2 for i in eachindex(des_rots)]
    des_yaws = [des_rots[i].theta3 for i in eachindex(des_rots)]

    # Parse down the actual poses from the simulation
    temp_state = MechanismState(mech_blue_alpha)
    actual_poses = []
    for i in 1:sample_rate:length(qs)
        set_configuration!(temp_state, qs[i])
        current_pose = inv(relative_transform(temp_state, base_frame, default_frame(bodies(mech_blue_alpha)[end])))
        push!(actual_poses, current_pose)
    end

    # separate actual translations
    act_translations = [translation(actual_poses[i]) for i in eachindex(actual_poses)]
    act_xs = [act_translations[i][1] for i in eachindex(act_translations)]
    act_ys = [act_translations[i][2] for i in eachindex(act_translations)]
    act_zs = [act_translations[i][3] for i in eachindex(act_translations)]

    # separate actual rotations
    act_rots = [RotXYZ(rotation(actual_poses[i])) for i in eachindex(actual_poses)]
    act_rolls = [act_rots[i].theta1 for i in eachindex(act_rots)]
    act_pitches = [act_rots[i].theta2 for i in eachindex(act_rots)]
    act_yaws = [act_rots[i].theta3 for i in eachindex(act_rots)]

    # plot
    print("Plotting poses...")
    gr()
    l = @layout[grid(3,1) grid(3,1)]
    # l = @layout[a; b; c]
    des_pose_vecs = [des_xs, des_ys, des_zs, des_rolls, des_pitches, des_yaws]
    act_pose_vecs = [act_xs, act_ys, act_zs, act_rolls, act_pitches, act_yaws]

    plot_labels = ["x pos", "y pos", "z pos", "roll", "pitch", "yaw"]
    plot_handles = []
    for k = 1:3
        label = plot_labels[k]
        push!(plot_handles, plot(ts_down, 
            [des_pose_vecs[k], act_pose_vecs[k]], 
            title=label, 
            legend=false, 
            ylim = [-2, 2], 
            yticks=-2:0.5:2))
    end
    for k = 4:6
        label = plot_labels[k]
        push!(plot_handles, plot(ts_down, 
            [des_pose_vecs[k], act_pose_vecs[k]], 
            title=label, 
            legend=true, 
            ylim=[-pi, pi], 
            label=["Desired" "Actual"]))
    end
    # plot!(legend=true, label=["Desired" "Actual"])

    display(plot(plot_handles..., 
        layout=l, 
        size=(1000, 800), 
        titlefontsize = 12,
        plot_title="Desired Poses"))
    # plot!(legend=:outerbottom, label = ["Desired", "Actual"])
end

function plot_zetas(ctlr, vs, ts_down)
    # Actual velocities from simulation
    paths = OrderedDict();
    for idx = 1:11
        joint_vels = [vs[i][idx] for i in 1:sample_rate:length(vs)]
        paths[string("vs", idx)] = joint_vels
    end

    # Desired velocities from controller 
    des_paths = OrderedDict();
    for idx = 1:9
        des_jt_vels = [ctlr.des_zetas[idx, tstep] for tstep in 1:sample_rate:size(ctlr.des_zetas,2)]
        des_paths[string("vs", idx+2)] = des_jt_vels
    end
    
    l = @layout[a; b; c]
    plot_handles = []
    var_names = ["vs4", "vs5", "vs6"]
    plot_labels = ["x", "y", "z"]
    for k = 1:3
        var = var_names[k]
        lab = plot_labels[k]
        push!(plot_handles, plot(ts_down, 
            [des_paths[var], paths[var]], 
            title=lab, 
            legend=false, 
            titlefontsize=12))
    end
    display(plot(plot_handles..., layout=l))
end
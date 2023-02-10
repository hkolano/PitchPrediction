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
            ylim = [-1.5, 1.5], 
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
        plot_title="Desired End Effector Poses"))
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
    for idx = 1:11
        des_jt_vels = [ctlr.des_zetas[idx, tstep] for tstep in 1:sample_rate:size(ctlr.des_zetas,2)]
        des_paths[string("vs", idx)] = des_jt_vels
    end
    
    l = @layout[grid(3,1) grid(3,1)]
    plot_handles = []
    var_names = ["vs1", "vs2", "vs3", "vs4", "vs5", "vs6"]
    plot_labels = ["d_pitch", "d_roll", "d_yaw", "v_x", "v_y", "v_z"]
    for k = 1:6
        var = var_names[k]
        lab = plot_labels[k]
        push!(plot_handles, plot(ts_down, 
            [des_paths[var], paths[var]], 
            title=lab, 
            legend=false, 
            titlefontsize=12))
    end
    display(plot(plot_handles..., 
        layout=l, 
        plot_title="Vehicle Zetas", 
        ylim=[-1, 1]))

    l_arm = @layout[a b; c d]
    plot_handles_arm = []
    var_names_arm = ["vs7", "vs8", "vs9", "vs10"]
    plot_labels_arm = ["d_theta1", "d_theta2", "d_theta3", "d_theta4"]
    for k = 1:4
        var = var_names_arm[k]
        lab = plot_labels_arm[k]
        push!(plot_handles_arm, plot(ts_down, 
            [des_paths[var], paths[var]], 
            title=lab, 
            legend=false,
            titlefontsize=12))
    end
    display(plot(plot_handles_arm..., 
        layout=l_arm, 
        plot_title="Arm Zetas", 
        ylim=[-1, 1]))
end

function plot_joint_config(qs, ts_down)
    paths = OrderedDict();
    for idx = 1:12
        joint_pos = [qs[i][idx] for i in 1:sample_rate:length(qs)]
        paths[string("qs", idx)] = joint_pos
    end

    l_arm = @layout[a b; c d]
    plot_handles_arm = []
    var_names_arm = ["qs8", "qs9", "qs10", "qs11"]
    plot_labels_arm = ["theta1", "theta2", "theta3", "theta4"]
    for k = 1:4
        var = var_names_arm[k]
        lab = plot_labels_arm[k]
        push!(plot_handles_arm, plot(ts_down, 
            paths[var], 
            title=lab, 
            legend=false,
            titlefontsize=12))
    end
    display(plot(plot_handles_arm..., layout=l_arm, plot_title="Joint Configurations"))
end

function plot_control_taus(ctlr, ts_down, plot_veh=false, plot_arm=true)

    ctrl_tau_dict = OrderedDict();
    for idx = 1:10
        joint_taus = [ctlr.taus[idx, tstep] for tstep in 1:sample_rate:size(ctlr.taus,2)]
        ctrl_tau_dict[idx] = joint_taus
    end

    tau_plot_lims = [[-1, 1], [-1, 1], [-3, 3], [-6, 0], [-3, 3], [4, 10], [-.1, .1], [-.1, .1], [-.1, .1], [-.1, .1]]
    # tl = @layout[a b; c d]
    plot_labels = ["roll", "pitch", "yaw", "x", "y", "z", "JointE", "JointD", "JointC", "JointB"]
    
    if plot_arm == true
        tau_plot_handles = []
        tl = @layout[a b; c d]
        for k = 7:10
            lab = plot_labels[k]
            push!(tau_plot_handles, plot(ts_down, ctrl_tau_dict[k], 
                                        title=lab, 
                                        legend=false)) #, 
                                        # ylim=tau_plot_lims[k]))
        end
        display(plot(tau_plot_handles..., 
            layout=tl, 
            plot_title="Control Forces", 
            size=(1000, 800)))
    end
end

function plot_des_vs_act_velocities(ctlr, ts_down, des_vs, vs; plot_veh=true, plot_arm=true)
    paths = OrderedDict();
    des_paths = OrderedDict();
    meas_paths = OrderedDict();
    filt_paths = OrderedDict();
    for idx = 1:10
        joint_vels = [vs[i][idx] for i in 1:sample_rate:length(vs)]
        paths[string("vs", idx)] = joint_vels
        if idx > 2
            des_joint_vels = [des_vs[i][idx-2] for i in 1:length(ts_down)]
            des_paths[string("vs", idx)] = des_joint_vels
        else
            des_paths[string("vs", idx)] = zeros(length(ts_down))
        end
        meas_paths[string("vs", idx)] = [ctlr.noisy_vs[idx,i] for i in 1:sample_rate:length(vs)]
        filt_paths[string("vs", idx)] = [ctlr.filtered_vs[idx,i] for i in 1:sample_rate:length(vs)]
    end

    if plot_veh == true
        l = @layout[a b; c d; e f]
        var_names = ["vs1", "vs2", "vs3", "vs4", "vs5", "vs6"]
        plot_labels = ["roll", "pitch", "yaw", "x", "y", "z"]
        plot_handles = []
        for k = 1:6
            var = var_names[k]
            lab = plot_labels[k]
            if k < 4
                push!(plot_handles, plot(ts_down, [des_paths[var], paths[var], meas_paths[var], filt_paths[var]], title=lab, legend=false, titlefontsize=12))
            else
                push!(plot_handles, plot(ts_down, [des_paths[var], paths[var], meas_paths[var], filt_paths[var]], title=lab, ylim=(-.025,.025), legend=false, titlefontsize=12))
            end
        end
        display(plot(plot_handles..., 
                    layout=l, 
                    plot_title="Vehicle Velocities"))
    end

    if plot_arm == true
        l = @layout[a b; c d]
        var_names = ["vs7", "vs8", "vs9", "vs10"]
        plot_labels = ["Joint E", "Joint D", "Joint C", "Joint B"]
        plot_handles = []
        for k = 1:4
            var = var_names[k]
            lab = plot_labels[k]
            push!(plot_handles, plot(ts_down, [des_paths[var], paths[var], meas_paths[var], filt_paths[var]], title=lab, label=["Desired" "Actual" "Noisy" "Filtered"], ylim=(-.5,.5), titlefontsize=12))
        end
        display(plot(plot_handles..., 
                    layout=l, 
                    plot_title="Arm Velocities"))
    end
    println("done.")

end

function plot_des_vs_act_positions(ctlr, ts_down, des_qs, qs)
    paths = OrderedDict();
    des_paths = OrderedDict();
    for idx = 8:11
        joint_qs = [qs[i][idx] for i in 1:sample_rate:length(qs)]
        paths[string("qs", idx)] = joint_qs

        des_joint_qs = [des_qs[i][idx-3] for i in 1:length(ts_down)]
        des_paths[string("qs", idx)] = des_joint_qs
    end

    l = @layout[a b; c d]
    var_names = ["qs8", "qs9", "qs10", "qs11"]
    plot_labels = ["Joint E", "Joint D", "Joint C", "Joint B"]
    plot_handles = []
    for k = 1:4
        var = var_names[k]
        lab = plot_labels[k]
        this_plot = plot(ts_down, [des_paths[var], paths[var]], title=lab, label=["Desired" "Actual" "Limits"], titlefontsize=12)
        hline!(this_plot, [joint_lims[k][1], joint_lims[k][2]])
        push!(plot_handles, this_plot)
    end
    display(plot(plot_handles..., 
                layout=l, 
                plot_title="Joint Positions"))
end
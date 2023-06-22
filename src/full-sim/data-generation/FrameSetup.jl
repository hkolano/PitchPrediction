function setup_frames!(mech, frame_names_cob, frame_names_com, cob_vecs, com_vecs, cob_frames, com_frames)
    num_joints = length(joints(mech))
    vis_element = 6
    for i in 1:num_joints
        bod = bodies(mech)[i+1]
        frame_cob = CartesianFrame3D(frame_names_cob[i])
        frame_com = CartesianFrame3D(frame_names_com[i])
        cob_vec = cob_vecs[i]
        com_vec = com_vecs[i]
        cob_transform = Transform3D(frame_cob, default_frame(bod), cob_vec)
        com_transform = Transform3D(frame_com, default_frame(bod), com_vec)
        if i == vis_element
            setelement!(mvis, default_frame(bod))
        end 
        if !(RigidBodyDynamics.is_fixed_to_body(bod, frame_cob))
            add_frame!(bod, cob_transform)
            push!(cob_frames, frame_cob)
            if i == vis_element
                setelement!(mvis, frame_cob, 0.3)    # visualizes COB frames in MeshCat
            end
        end
        if !(RigidBodyDynamics.is_fixed_to_body(bod, frame_com))
            add_frame!(bod, com_transform)
            push!(com_frames, frame_com)
            if i == vis_element
                setelement!(mvis, frame_com, 0.2)    # visualizes COM frames in MeshCat
            end
        end
    end

    alphabase_com_wrt_linkframe = SVector{3, Float64}([-0.075, -0.006, -.003])
    # Arm base is rigidly attached to vehicle, so it has a transform in the vehicle's frame. It's the 5th body in the URDF attached to the vehicle. 
    linkframe_wrt_vehframe = translation(RigidBodyDynamics.frame_definitions(vehicle_body)[5])
    # # IF THE ARM IS ROTATED THIS HAS TO CHANGE!!!!
    alphabase_com_wrt_vehframe = alphabase_com_wrt_linkframe + linkframe_wrt_vehframe
    alphabase_com_frame = CartesianFrame3D("armbase_com_cob")
    com_transform = Transform3D(alphabase_com_frame, default_frame(vehicle_body), alphabase_com_wrt_vehframe)

    if !(RigidBodyDynamics.is_fixed_to_body(vehicle_body, alphabase_com_frame))
        add_frame!(vehicle_body, com_transform)
        push!(cob_frames, alphabase_com_frame)
        push!(com_frames, alphabase_com_frame)
        # setelement!(mvis, alphabase_com_frame)
    end
    # print("THIS SHOULD SAY after_arm_to_vehicle: ")
    println(RigidBodyDynamics.frame_definitions(vehicle_body)[5].from)
end

function visualize_path(des_poses, mvis, world_body)
    frame_idxs = [6, 8, 10, 12, 14, 20]
    poses = Vector{Transform3D}(undef, length(frame_idxs))
    new_poses = Vector{Transform3D}(undef, length(frame_idxs))
    frames = Vector{CartesianFrame3D}(undef, length(frame_idxs))
    frame_names = Vector{String}(undef, length(frame_idxs))
    for i in eachindex(frame_idxs)
        poses[i] = des_poses[frame_idxs[i]]
        frame_name = "frame_"*string(frame_idxs[i])
        frames[i] = CartesianFrame3D(frame_name)
        new_poses[i] = Transform3D(frames[i], default_frame(world_body), poses[i].mat)
        add_frame!(world_body, new_poses[i])
        setelement!(mvis, frames[i], 0.4, frame_name)
    end
end
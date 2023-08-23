function setup_frames(body_dict, body_name_list, cob_vec_dict, com_vec_dict)
    cob_frame_dict = Dict{String, CartesianFrame3D}()
    com_frame_dict = Dict{String, CartesianFrame3D}()
    vis_element = 6

    for (i, body_name) in enumerate(body_name_list)
        # Get the body of interest
        bod = body_dict[body_name]

        # Name the two new frames
        frame_cob = CartesianFrame3D(body_name*"_cob")
        frame_com = CartesianFrame3D(body_name*"_com")

        # Define the two new frames wrt the default frame
        cob_transform = Transform3D(frame_cob, default_frame(bod), cob_vec_dict[body_name])
        com_transform = Transform3D(frame_com, default_frame(bod), com_vec_dict[body_name])
        
        # Add the frames to the bodies
        if !(RigidBodyDynamics.is_fixed_to_body(bod, frame_cob))
            add_frame!(bod, cob_transform)
            cob_frame_dict[body_name] = frame_cob
        end
        if !(RigidBodyDynamics.is_fixed_to_body(bod, frame_com))
            add_frame!(bod, com_transform)
            com_frame_dict[body_name] = frame_com
        end

        # if desired, visualize the new frames on the body
        if i == vis_element
            setelement!(mvis, default_frame(bod))
            setelement!(mvis, frame_cob, 0.3)
            setelement!(mvis, frame_com, 0.2)
        end 

    end

    # Add buoyancy or gravity to each extra element added to the BlueROV
    if @isdefined(vehicle_extras_list)
        vehicle_body = body_dict["vehicle"]

        for (j, item_name) in enumerate(vehicle_extras_list)
            println("Parsing item "*item_name)
            frame_center = CartesianFrame3D(item_name*"_centerframe")
            if haskey(com_vec_dict, item_name)
                println("Has mass")
                center_transform = Transform3D(frame_center, default_frame(vehicle_body), com_vec_dict[item_name])
                com_frame_dict[item_name] = frame_center
            elseif haskey(cob_vec_dict, item_name)
                println("has buoyancy")
                center_transform = Transform3D(frame_center, default_frame(vehicle_body), cob_vec_dict[item_name])
                cob_frame_dict[item_name] = frame_center
            end

            if !(RigidBodyDynamics.is_fixed_to_body(vehicle_body, frame_center))
                add_frame!(vehicle_body, center_transform)
            end

            if item_name == "dvl" || item_name == "dvlbracket"
                println("Trying to show com")
                setelement!(mvis, frame_center, 0.25)
            end

        end
    end

    alphabase_com_wrt_linkframe = com_vec_dict["armbase"]
    # Arm base is rigidly attached to vehicle, so it has a transform in the vehicle's frame. It's the 5th body in the URDF attached to the vehicle. 
    linkframe_wrt_vehframe = translation(RigidBodyDynamics.frame_definitions(body_dict["vehicle"])[5])
    # # IF THE ARM IS ROTATED THIS HAS TO CHANGE!!!!
    alphabase_com_wrt_vehframe = alphabase_com_wrt_linkframe + linkframe_wrt_vehframe
    alphabase_com_frame = CartesianFrame3D("armbase_com_cob")
    com_transform = Transform3D(alphabase_com_frame, default_frame(body_dict["vehicle"]), alphabase_com_wrt_vehframe)

    if !(RigidBodyDynamics.is_fixed_to_body(body_dict["vehicle"], alphabase_com_frame))
        add_frame!(body_dict["vehicle"], com_transform)
        cob_frame_dict["armbase"] = alphabase_com_frame
        com_frame_dict["armbase"] = alphabase_com_frame
        # setelement!(mvis, alphabase_com_frame)
    end
    # print("THIS SHOULD SAY after_arm_to_vehicle: ")
    println(RigidBodyDynamics.frame_definitions(body_dict["vehicle"])[5].from)
    return cob_frame_dict, com_frame_dict
end

function mechanism_reference_setup(urdf_file)
    vis = Visualizer()
    mech_blue_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
    delete!(vis)

    # Create visuals of the URDFs
    mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file), vis[:alpha])

    # Name the joints and bodies of the mechanism
    joint_dict = Dict{String, RigidBodyDynamics.Joint}()
    body_dict = Dict{String, RigidBodyDynamics.RigidBody}()
    for (idx, link_name) in enumerate(body_names)
        body_dict[link_name] = bodies(mech_blue_alpha)[idx+1]
    end
    joint_dict["vehicle"] = joints(mech_blue_alpha)[1]
    for (idx, dof_name) in enumerate(dof_names)
        if idx > 6
            joint_dict[dof_name] = joints(mech_blue_alpha)[idx-5]
        end
    end
    return mech_blue_alpha, mvis, joint_dict, body_dict
end

function setup_buoyancy_and_gravity(buoyancy_mag_dict, grav_mag_dict)
    for (k, mag) in buoyancy_mag_dict
        buoyancy_force_dict[k] = FreeVector3D(root_frame(mech_blue_alpha), [0.0, 0.0, mag])
    end
    for (k, mag) in grav_mag_dict
        gravity_force_dict[k] = FreeVector3D(root_frame(mech_blue_alpha), [0.0, 0.0, -mag])
    end
    return buoyancy_force_dict, gravity_force_dict
end
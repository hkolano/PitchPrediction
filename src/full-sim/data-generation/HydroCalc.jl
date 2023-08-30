#=
Hydrodynamics calculator functions; called within SimWExt.jl 

=#
using RigidBodyDynamics

# ------------------------------------------------------------------------
#                HYDRODYNAMICS (Grav & Buoy) CALCULATOR
# ------------------------------------------------------------------------
function hydro_calc!(hydro_wrenches::Dict{BodyID, Wrench{Float64}}, t, state::MechanismState)
    buoy_wrenches = []
    grav_wrenches = []

    print_now = false

    # if abs(rem(t, 2)) <= .0005
    #     println("=====================================")
    #     @show t
    #     print_now = true
    # end

    # Iterate through each body 
    for body_name in body_names
        # Get the body
        bod = body_dict[body_name]
        # Get default frame of the body
        body_default_frame = default_frame(bod)
        # println("-----------")
        # @show bod

         # -------- Calculate Buoyancy Wrench-------
        # Get transform between the defualt frame and the center of buoyancy
        # TODO: don't use fixed_transform because it's bad for computation time
        def_to_cob = fixed_transform(bod, body_default_frame, cob_frame_dict[body_name])
        # Transform buoyancy force vector to the body's default frame (rotation only)
        buoy_force_trans = RigidBodyDynamics.transform(state, buoyancy_force_dict[body_name], body_default_frame)
        # Make the wrench: the buoyancy force through a point, the center of buoyancy.
        buoy_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_cob))), buoy_force_trans)
        push!(buoy_wrenches, buoy_wrench) 
        # @show buoy_wrench    
        
        # -------- Calculate Gravity Wrench -------
        def_to_com = fixed_transform(bod, body_default_frame, com_frame_dict[body_name])
        grav_force_trans = RigidBodyDynamics.transform(state, gravity_force_dict[body_name], body_default_frame)
        # println(grav_force_trans)
        # Make the wrench: the buoyancy force through a point, the center of buoyancy.
        # COM = Point3D(body_default_frame, translation(inv(def_to_com)))
        grav_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_com))), grav_force_trans)
        # setelement!(mvis, COM, .01)
        # Add wrench to buoy_wrenches
        push!(grav_wrenches, grav_wrench)
        # @show transform(state, grav_wrench, root_frame(state.mechanism))
        # @show grav_wrench

        if print_now == true
            println("-----")
            @show bod 
            # @show transform(state, buoy_wrench, root_frame(state.mechanism))
            # @show transform(state, grav_wrench, root_frame(state.mechanism))
            @show RigidBodyDynamics.transform(state, buoy_wrench, default_frame(body_dict["vehicle"]))
            @show RigidBodyDynamics.transform(state, grav_wrench, default_frame(body_dict["vehicle"]))
        end

        wrench = buoy_wrench + grav_wrench

        # ----- Special calculaitons for the vehicle -----
        if body_name == "vehicle"
            # ----- Grav/buoy for arm base link ----- 
            def_to_armbase_cob = fixed_transform(bod, body_default_frame, cob_frame_dict["armbase"])
            def_to_armbase_com = fixed_transform(bod, body_default_frame, com_frame_dict["armbase"])
            buoy_force_trans_armbase = RigidBodyDynamics.transform(state, buoyancy_force_dict["armbase"], body_default_frame)
            grav_force_trans_armbase = RigidBodyDynamics.transform(state, gravity_force_dict["armbase"], body_default_frame)
            buoy_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_cob))), buoy_force_trans_armbase)
            grav_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_com))), grav_force_trans_armbase)
            wrench = wrench + buoy_wrench_arm + grav_wrench_arm

            if print_now == true
                # @show transform(state, buoy_wrench_arm, root_frame(state.mechanism))
                # @show transform(state, grav_wrench_arm, root_frame(state.mechanism))
                @show RigidBodyDynamics.transform(state, buoy_wrench_arm, body_default_frame)
                @show RigidBodyDynamics.transform(state, grav_wrench_arm, body_default_frame)
            end

            # ----- Grav/buoy for extra components -----
            if @isdefined(vehicle_extras_list)
                for item_name in vehicle_extras_list
                    # println("Applying forces to vehicle from "*item_name)
                    if haskey(com_vec_dict, item_name)
                        def_to_center = fixed_transform(bod, body_default_frame, com_frame_dict[item_name])
                        force_trans = RigidBodyDynamics.transform(state, gravity_force_dict[item_name], body_default_frame)
                        
                    elseif haskey(cob_vec_dict, item_name)
                        def_to_center = fixed_transform(bod, body_default_frame, cob_frame_dict[item_name])
                        force_trans = RigidBodyDynamics.transform(state, buoyancy_force_dict[item_name], body_default_frame)
                    end
                    app_pt = Point3D(body_default_frame, translation(inv(def_to_center)))
                    if item_name == "foamL"
                        setelement!(mvis, app_pt, .02)
                    end
                    add_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_center))), force_trans)
                    
                    if print_now == true 
                        println("-----")
                        @show item_name
                        @show RigidBodyDynamics.transform(state, add_wrench, root_frame(state.mechanism))
                        # @show transform(state, add_wrench, body_default_frame)
                    end
                    wrench = wrench + add_wrench
                end
            end

            
            # Drag on the vehicle 
            vel = velocity(state, joint_dict["vehicle"])
            # @show vel
            tau_d = -d_lin_coeffs .* vel .+ -d_nonlin_coeffs .* vel .* abs.(vel)
            drag_wrench = Wrench(body_default_frame, tau_d[1:3], tau_d[4:6])
            # @show drag_wrench
            wrench = wrench + drag_wrench 
            # @show transform(state, wrench, root_frame(state.mechanism))
            # @show wrench
            # println("Wrench drag:")
            # println(drag_wrench)
        # ----- Drag on the links (quadratic only) ----
        else 
            twist_world = twist_wrt_world(state, bod)
            root_transform = transform_to_root(state, bod)
            # COB_point = Point3D(body_default_frame, translation(inv(def_to_cob)))
            twist_body = RigidBodyDynamics.transform(twist_world, inv(root_transform))
            cob_vel = point_velocity(twist_body, Point3D(body_default_frame, translation(inv(def_to_cob))))
            F_d = transpose(-link_drags[body_name]) .* abs.(cob_vel.v) .* cob_vel.v
            drag_wrench_at_cob = Wrench(cob_frame_dict[body_name], [0.0, 0.0, 0.0], [F_d[1], F_d[2], F_d[3]])
            drag_wrench_at_default = RigidBodyDynamics.transform(drag_wrench_at_cob, inv(def_to_cob))
            # @show drag_wrench_at_default

            wrench = wrench + drag_wrench_at_default
        end

        
        hydro_wrenches[BodyID(bod)] = RigidBodyDynamics.transform(state, wrench, root_frame(state.mechanism))
        if print_now == true
            @show hydro_wrenches[BodyID(bod)]
        end
    end
    
end;
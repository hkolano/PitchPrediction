# Modifications and additions for the Hinsdale trials Aug 2023

# Initialize dictionaries for buoyancy forces
cob_vec_dict = Dict{String, SVector{3,Float64}}()
cob_vec_dict["vehicle"] = SVector{3, Float64}([0.009, 0.0, 0.02])
# cob_vec_dict["vehicle"] = SVector{3, Float64}([0.0, 0.0, 0.02])
buoyancy_mag_dict = Dict{String, Float64}()
buoyancy_mag_dict["vehicle"] = 13.17*9.81 #(volume * gravity)
buoyancy_force_dict = Dict{String, FreeVector3D}()

# Initialize dictionary for gravitational forces
com_vec_dict = Dict{String, SVector{3, Float64}}()
# com_vec_dict["vehicle"] = SVector{3, Float64}([-0.012, 0.0, 0.0])
com_vec_dict["vehicle"] = SVector{3, Float64}([0.0, 0.0, 0.0])
grav_mag_dict = Dict{String, Float64}()
grav_mag_dict["vehicle"] = 13.17*9.81 #(weight * gravity)
gravity_force_dict = Dict{String, FreeVector3D}()

# Additions to the vehicle 
# calculations found in https://docs.google.com/spreadsheets/d/1u45-1og3hwlrbyl4GoW4T-87IQI3Vzu-zy2CbMDW1dY/edit#gid=0
# weight counter-arm, weight bottom-left, weight bottom-right 
vehicle_extras_list = ["weightCA", "weightBL", "weightBR", "dvl", "dvlbracket", "foamL", "foamR"]
grav_mag_dict["weightCA"] = 3.24 # water weight
grav_mag_dict["weightBL"] = 1.62 # water weight
grav_mag_dict["weightBR"] = 1.62 # water weight
grav_mag_dict["dvl"] = 0.69 # water weight 
grav_mag_dict["dvlbracket"] = 1.01 # water weight 
# buoyancy_mag_dict["foamL"] = 8.66 
# buoyancy_mag_dict["foamR"] = 8.66 
# Gets to resting roll of 0.0418 without upward movement
buoyancy_mag_dict["foamL"] = 7.9 
buoyancy_mag_dict["foamR"] = 9.42 
# buoyancy_mag_dict["foamL"] = 6.9
# buoyancy_mag_dict["foamR"] = 10.42

com_vec_dict["weightCA"] = SVector{3, Float64}([-.20, .165, -.075]) # guess
com_vec_dict["weightBL"] = SVector{3, Float64}([-.0975, .1275, -.1325]) # guess
com_vec_dict["weightBR"] = SVector{3, Float64}([-.0975, -.1275, -.1325]) # guess
com_vec_dict["dvl"] = SVector{3, Float64}([-.1887, .0439, -.1095+0.05]) # guess
com_vec_dict["dvlbracket"] = SVector{3, Float64}([-.1887+.0345, .0439, -.1295+.05]) # guess
cob_vec_dict["foamL"] = SVector{3, Float64}([0.00, .11, 0.027]) # guess
cob_vec_dict["foamR"] = SVector{3, Float64}([0.00, -.11, 0.027]) #guess

# Drag coefficients for the vehicle
d_lin_coeffs = [4.03, 6.22, 5.18, .07, .07, .07]
d_nonlin_coeffs = [18.18, 21.66, 36.99, 1.55, 1.55, 1.55]

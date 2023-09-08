# Modifications and additions for the Hinsdale trials Aug 2023

# Initialize dictionaries for buoyancy forces
cob_vec_dict = Dict{String, SVector{3,Float64}}()
# for pitch = 0.031 (average for mocap data)
# cob_vec_dict["vehicle"] = SVector{3, Float64}([0.0072, 0.0, 0.02])
# for pitch = .025 (average for imu data)
cob_vec_dict["vehicle"] = SVector{3, Float64}([0.0074, 0.0, 0.02])
# cob_vec_dict["vehicle"] = SVector{3, Float64}([0.0, 0.0, 0.02])
buoyancy_mag_dict = Dict{String, Float64}()
# buoyancy_mag_dict["vehicle"] = 13.17*9.81 #(volume * gravity) (exactly even)
buoyancy_mag_dict["vehicle"] = 13.082*9.81 #(volume * gravity)

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
# Resting roll = .0534, pitch = .0308
buoyancy_mag_dict["foamL"] = 8.2
buoyancy_mag_dict["foamR"] = 9.12
# resting roll = 0.682
buoyancy_mag_dict["foamL"] = 8.46
buoyancy_mag_dict["foamR"] = 8.66+.2 

com_vec_dict["weightCA"] = SVector{3, Float64}([-.20, .165, -.075]) # guess
com_vec_dict["weightBL"] = SVector{3, Float64}([-.0975, .1275, -.1325]) # guess
com_vec_dict["weightBR"] = SVector{3, Float64}([-.0975, -.1275, -.1325]) # guess
com_vec_dict["dvl"] = SVector{3, Float64}([-.1887, .0439, -.1095+0.05]) # guess
com_vec_dict["dvlbracket"] = SVector{3, Float64}([-.1887+.0345, .0439, -.1295+.05]) # guess
cob_vec_dict["foamL"] = SVector{3, Float64}([0.00, .11, 0.027]) # guess
cob_vec_dict["foamR"] = SVector{3, Float64}([0.00, -.11, 0.027]) #guess

# Drag coefficients for the vehicle
d_lin_angular = .07 
d_nonlin_angular = 1.55
d_lin_coeffs = [4.03, 6.22, 5.18, d_lin_angular, d_lin_angular, d_lin_angular]
d_nonlin_coeffs = [18.18, 21.66, 36.99, d_nonlin_angular, d_nonlin_angular, d_nonlin_angular]

#=
xd = .4571 (size of the bluerov in the X direction)
yd = .3381 (size of the bluerov in the y direction)
zd = .127 (1/2 the height of the vehicle)

m = 25.14
25.14

using simplified added mass as m
julia> Ixx = (1/12)*m*(yd^2 + zd^2)
0.27327307795

julia> Iyy = (1/12)*m*(xd^2 + zd^2)
0.47152041394999994

julia> Izz = (1/12)*m*(xd^2 + yd^2)
0.6772129818999999

Considering different added masses in each direction
julia> mx = 14.24+5.5
19.740000000000002

julia> my = 14.24+12.7
26.939999999999998

julia> mz = 14.24 + 14.57
28.810000000000002

julia> Izz = (1/12)*(mx*xd^2 + my*yd^2)
0.6003365388999999

julia> Iyy = (1/12)*(mx*xd^2 + mz*zd^2)
0.38243001528333337

julia> Ixx = (1/12)*(my*yd^2 + mz*zd^2)
0.2953526052833333
=#
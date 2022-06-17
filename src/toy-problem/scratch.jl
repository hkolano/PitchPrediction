include("TrajGen.jl")
# foo = TrajGen.jointState([.1, .2], [.2, .3])
# fum = TrajGen.jointState([1.1, 1.2], [-1, .3])
# floodle = TrajGen.Waypoints(foo, fum)

floodle = gen_rand_waypoints()
# println(floodle)

# try 
#     TrajGen.find_trajectory(floodle)
# catch e 
#     println("Not feasible: try another set of waypoints.")
# end 


-- WINGBOT publishes odom->base_footprint and the measured model sensor TF.
-- Keep the conservative symmetric-arena constraints; no simulator truth input.
include "cartographer_gazebo_2d.lua"
options.tracking_frame = "base_footprint"
return options

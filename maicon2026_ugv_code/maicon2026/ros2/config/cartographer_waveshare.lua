-- Waveshare ugv_ws ros2-humble-develop, f0b3ad9c16e7acb6d6145f261116d71bf6338f4a.
-- EKF owns odom -> base_footprint; the vendor URDF owns its sensor children.
-- Do not also start vendor Cartographer/AMCL or another map -> odom publisher.
include "cartographer_2d.lua"
options.tracking_frame = "base_footprint"
return options

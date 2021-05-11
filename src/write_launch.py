#!/usr/bin/env python

# define some objects for storing
co_dict = {}
x = 0
y = 0
index = 0
space = "    "
# strings for formatting
string1 = space + space + "<arg name=\"robot_name\" value=\"fiducial_cube{cube_index}\"/>\n"
string2 = space + space + "<arg name=\"x\" value=\"{x_value}\" />\n"
string3 = space + space + "<arg name=\"y\" value=\"{y_value}\" />\n"

# open the txt file including the coordinates
source_file = open("coordinates.txt","r")
# open the dynamic launch file
launch_file = open("/my_ros_data/catkin_ws/src/FiducialPathFollowing/launch/dynamic_spawn.launch", "w")
# process each line
coordinate_list = source_file.readlines()
# save coordinates according to index
for co in coordinate_list:
    x, y = (int(s) for s in co.split())
    co_dict[index] = [x, y]
    index = index + 1

# write to the launch file
launch_file.write("<launch>\n")
for i in range(index):
    launch_file.write(space + "<include file=\"$(find FiducialPathFollowing)/launch/spawn_sdf.launch\">\n")
    launch_file.write(string1.format(cube_index = i))
    launch_file.write(string2.format(x_value = co_dict[i][0]))
    launch_file.write(string3.format(y_value = co_dict[i][1]))
    launch_file.write(space + space + "<arg name=\"z\" value=\"-0.139\" />\n")
    launch_file.write(space + space + "<arg name=\"sdf_robot_file\" value=\"$(find FiducialPathFollowing)/models/fiducial_cube_01/model.sdf\" />\n")
    launch_file.write(space + "</include>\n")
launch_file.write("</launch>\n")
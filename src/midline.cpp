/*
 Copyright (c) 2023 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "../include/GRO.hh"

// GRO object is initialized here
GRO GROobj;

ros::Publisher groPath;

int main(int argc, char **argv){

    // Init Node:
    ros::init(argc, argv, "midline");

    // Handle Connections:
    ros::NodeHandle nh("~");
    groPath = nh.advertise<nav_msgs::Path>("/gro/path", 1); // Visualization purposes only

    // Setting params from yaml
    nh.param<string>("Paths/midline", GROobj.midlinePath,  "");
    nh.param<string>("Paths/save",    GROobj.savePath,     "");

    nh.param<double>("spacing",         GROobj.spacing,          0.05);
    nh.param<double>("separation",      GROobj.separation,       3.0);
    nh.param<double>("securityFactor",  GROobj.securityFactor,   2.0);

    nh.param<bool>("visualization", GROobj.visualization, false);

    // Reading from tracklimits file
    as_msgs::Tracklimits data;

    data = GROobj.read_csv(GROobj.midlinePath);

    cout << "Left\n";
    for(int i=0; i<data.left.size(); i++){ cout << data.left[i].position_global.x << " " << data.left[i].position_global.y << endl;}
    cout << "Right\n";
    for(int i=0; i<data.right.size(); i++){ cout << data.right[i].position_global.x << " " << data.right[i].position_global.y << endl;}

    // Initialization of GRO
    GROobj.init(data);

    // Saving middle trajectory
    GROobj.save_data(GROobj.savePath);

    if(GROobj.visualization){
        ros::Duration(2).sleep();
        groPath.publish(GROobj.get_path()); // Visualize trajectory in rviz
        ROS_WARN_STREAM("GRO path published in " << groPath.getTopic());
    }

    ros::spin();
}
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

#include <ros/ros.h>
#include "../include/TRO.hh"
// #include <dynamic_reconfigure/server.h>

// Publishers are initialized here
ros::Publisher troPath, troFirstPath, troPlanner;
ros::Publisher pubConesLoopMA;

// TRO is initialized here
TRO TROobj;

// This is the planner callback
void callback_planner(const as_msgs::CarState::ConstPtr &data){

    // Don't do anything until the TRO is initialized
    if (TROobj.isRunning()){
        
        if(!TROobj.firstSpFlag) TROobj.join_track(data);
        as_msgs::ObjectiveArrayCurv msgCurv = TROobj.plannerTRO(data);
        troPlanner.publish(msgCurv);

        // Visualization of the path
        troPath.publish(TROobj.get_path());
        troFirstPath.publish(TROobj.get_Firstpath());
        TROobj.pubMarkerArray(TROobj.read_cones(TROobj.conesPath), pubConesLoopMA);
    }
}


int main(int argc, char **argv){

    // Init Node:
    ros::init(argc, argv, "tro");

    // Handle Connections:
    ros::NodeHandle nh("~");

    // Topics
    string plannerTopic, pathTopic, FirstpathTopic, conesTopic, stateCarTopic;

    // Setting params from yaml
    nh.param<string>("Paths/Optimized",     TROobj.optimizedPath,    "");
    nh.param<string>("Paths/Problem",       TROobj.problemPath,      "");
    nh.param<string>("Paths/MiddlePoints",  TROobj.middlePointsPath, "");
    nh.param<string>("Paths/FreeSpace",     TROobj.freeSpacePath,    "");
    nh.param<string>("Paths/Save",          TROobj.savePath,         "");

    nh.param<int>("splinesStride",  TROobj.splinesStride,    12);
    nh.param<int>("firstSpId",      TROobj.firstSpId,        0);

    nh.param<double>("Tmiddle",     TROobj.Tmiddle,  0.05);
    nh.param<double>("firstSpId",   TROobj.v0,       0.0);

    nh.param<string>("conesPath", TROobj.conesPath, "");

    nh.param<string>("Topics/Planner",      plannerTopic,   "");
    nh.param<string>("Topics/Path",         pathTopic,      "");
    nh.param<string>("Topics/FirstPath",    FirstpathTopic, "");
    nh.param<string>("Topics/Cones",        conesTopic,     "");
    nh.param<string>("Topics/StateCar",     stateCarTopic,  "");

    // Publishers & Subscribers:
    ros::Subscriber subState = nh.subscribe(stateCarTopic, 1, callback_planner);
    troPlanner = nh.advertise<as_msgs::ObjectiveArrayCurv>(plannerTopic, 1);

    troPath = nh.advertise<nav_msgs::Path>(pathTopic, 1); // Visualization purposes only
    troFirstPath = nh.advertise<nav_msgs::Path>(FirstpathTopic, 1); // Visualization purposes only
    pubConesLoopMA = nh.advertise<visualization_msgs::MarkerArray>(conesTopic, 1); // Visualization purposes only

    // Initialization of TRO
    if(not TROobj.isRunning()){

        // Initialization
        TROobj.init();

        ros::Duration(1).sleep();

        // Visualization of the path
        troPath.publish(TROobj.get_path());
        TROobj.pubMarkerArray(TROobj.read_cones(TROobj.conesPath), pubConesLoopMA);

    }

    cout << "tot ok" <<endl;

    ros::spin();
}


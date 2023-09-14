#include <ros/ros.h>
#include "../include/TRO.hh"
// #include <dynamic_reconfigure/server.h>

// Publishers are initialized here
ros::Publisher troPath, troFirstPath, troPlanner;
ros::Publisher pubConesLoopMA;

// TRO is initialized here
TRO TROobject;

// This is the planner callback
void callback_planner(const as_msgs::CarState::ConstPtr &data){

    // Don't do anything until the TRO is initialized
    if (TROobject.isRunning()){
        
        if(!TROobject.firstSpFlag) TROobject.join_track(data);
        as_msgs::ObjectiveArrayCurv msgCurv = TROobject.plannerTRO(data);
        troPlanner.publish(msgCurv);

        // Visualization of the path
        troPath.publish(TROobject.get_path());
        troFirstPath.publish(TROobject.get_Firstpath());
        TROobject.pubMarkerArray(TROobject.read_cones(TROobject.conesPath), pubConesLoopMA);
    }
}


int main(int argc, char **argv){

    // Init Node:
    ros::init(argc, argv, "tro");

    // Handle Connections:
    ros::NodeHandle nh;

    // Topics
    string plannerTopic, pathTopic, FirstpathTopic, conesTopic, stateCarTopic;

    // Setting params from yaml
    nh.getParam("/tro/optimizedPath", TROobject.optimizedPath);
    nh.getParam("/tro/problemPath", TROobject.problemPath);
    nh.getParam("/tro/middlePointsPath", TROobject.middlePointsPath);
    nh.getParam("/tro/freeSpacePath", TROobject.freeSpacePath);

    nh.getParam("/tro/splinesStride", TROobject.splinesStride);
    nh.getParam("/tro/Tmiddle", TROobject.Tmiddle);
    nh.getParam("/tro/conesPath", TROobject.conesPath);
    nh.getParam("/tro/firstSpId", TROobject.firstSpId);
    nh.getParam("/tro/v0", TROobject.v0);

    nh.getParam("/tro/plannerTopic", plannerTopic);
    nh.getParam("/tro/pathTopic", pathTopic);
    nh.getParam("/tro/FirstpathTopic", FirstpathTopic);
    nh.getParam("/tro/conesTopic", conesTopic);
    nh.getParam("/tro/stateCarTopic", stateCarTopic);

    // Publishers & Subscribers:
    ros::Subscriber subState = nh.subscribe(stateCarTopic, 1, callback_planner);
    troPlanner = nh.advertise<as_msgs::ObjectiveArrayCurv>(plannerTopic, 1);

    troPath = nh.advertise<nav_msgs::Path>(pathTopic, 1); // Visualization purposes only
    troFirstPath = nh.advertise<nav_msgs::Path>(FirstpathTopic, 1); // Visualization purposes only
    pubConesLoopMA = nh.advertise<visualization_msgs::MarkerArray>(conesTopic, 1); // Visualization purposes only

    // Initialization of TRO
    if(not TROobject.isRunning()){

        // Initialization
        TROobject.init();

        ros::Duration(1).sleep();

        // Visualization of the path
        troPath.publish(TROobject.get_path());
        TROobject.pubMarkerArray(TROobject.read_cones(TROobject.conesPath), pubConesLoopMA);

    }

    cout << "tot ok" <<endl;

    ros::spin();
}


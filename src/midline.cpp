#include "../include/GRO.hh"

// GRO object is initialized here
GRO GROobject;

ros::Publisher groPath;

int main(int argc, char **argv){

    // Init Node:
    ros::init(argc, argv, "midline");

    // Handle Connections:
    ros::NodeHandle nh;
    groPath = nh.advertise<nav_msgs::Path>("/gro/path", 1); // Visualization purposes only

    // Setting params from yaml
    nh.getParam("/midline/midlinePath", GROobject.midlinePath);
    nh.getParam("/midline/savePath", GROobject.savePath);
    nh.getParam("/midline/spacing", GROobject.spacing);
    nh.getParam("/midline/separation", GROobject.separation);
    nh.getParam("/midline/securityFactor", GROobject.securityFactor);
    nh.getParam("/midline/visualization", GROobject.visualization);

    // Reading from tracklimits file
    as_msgs::Tracklimits data;

    data = GROobject.read_csv(GROobject.midlinePath);

    cout << "Left\n";
    for(int i=0; i<data.left.size(); i++){ cout << data.left[i].position_global.x << " " << data.left[i].position_global.y << endl;}
    cout << "Right\n";
    for(int i=0; i<data.right.size(); i++){ cout << data.right[i].position_global.x << " " << data.right[i].position_global.y << endl;}

    // Initialization of GRO
    GROobject.init(data);

    // Saving middle trajectory
    GROobject.save_data(GROobject.savePath);

    if(GROobject.visualization){
        ros::Duration(2).sleep();
        groPath.publish(GROobject.get_path()); // Visualize trajectory in rviz
        ROS_WARN_STREAM("GRO path published in " << groPath.getTopic());
    }

    ros::spin();
}
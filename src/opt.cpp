#include "../include/optimizer.hh"

optimizer OPTobj;

int main(int argc, char **argv){

    // Init Node:
    ros::init(argc, argv, "optimizer");

    // Handle Connections:
    ros::NodeHandle nh("~");

    // Setting params from yaml
    nh.param<string>("Paths/curvature", OPTobj.curvaturePath, "");
    nh.param<string>("Paths/save",      OPTobj.savePath,      "");

    nh.param<bool>("debugging", OPTobj.debugging, false);

    nh.param<int>("horizonLength", OPTobj.horizonLength,     0);
    nh.param<int>("curvatureStride", OPTobj.curvatureStride, 10);

    nh.param<double>("dRd",     OPTobj.dRd,      1.0);
    nh.param<double>("dRa",     OPTobj.dRa,      0.3);
    nh.param<double>("m",       OPTobj.m,        240.0);
    nh.param<double>("Iz",      OPTobj.I,        93.0);
    nh.param<double>("Lf",      OPTobj.Lf,       0.708);
    nh.param<double>("Lr",      OPTobj.Lr,       0.882);
    nh.param<double>("Dr",      OPTobj.Dr,       2800.0);
    nh.param<double>("Df",      OPTobj.Df,       2400.0);
    nh.param<double>("Cr",      OPTobj.Cr,       1.6);
    nh.param<double>("Cf",      OPTobj.Cf,       1.6);
    nh.param<double>("Br",      OPTobj.Br,       10.1507);
    nh.param<double>("Bf",      OPTobj.Bf,       10.8529);
    nh.param<double>("u_r",     OPTobj.u_r,      0.045);
    nh.param<double>("gravity", OPTobj.gravity,  9.81);
    nh.param<double>("Cd",      OPTobj.Cd,       0.87);
    nh.param<double>("rho",     OPTobj.rho,      1.255);
    nh.param<double>("Ar",      OPTobj.Ar,       1.0);
    nh.param<double>("q_slip",  OPTobj.q_slip,   0.1);
    nh.param<double>("p_long",  OPTobj.p_long,   2.0);
    nh.param<double>("q_n",     OPTobj.q_n,      0.1);
    nh.param<double>("q_mu",    OPTobj.q_mu,     0.5);
    nh.param<double>("lambda",  OPTobj.lambda,   0.5);
    nh.param<double>("q_s",     OPTobj.q_s,      1.0);

    // Initialization of OPTIMIZER
    if(not OPTobj.isRunning()) OPTobj.init();

    ros::spin();

}
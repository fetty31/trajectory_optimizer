#include "../include/optimizer.hh"

optimizer OPTobject;

int main(int argc, char **argv){

    // Init Node:
    ros::init(argc, argv, "optimizer");

    // Handle Connections:
    ros::NodeHandle nh;

    // Setting params from yaml
    nh.getParam("/optimizer/curvatureStride", OPTobject.curvatureStride);
    nh.getParam("/optimizer/debugging", OPTobject.debugging);
    nh.getParam("/optimizer/curvaturePath", OPTobject.curvaturePath);
    nh.getParam("/optimizer/savePath", OPTobject.savePath);

    nh.getParam("/optimizer/horizonLength", OPTobject.horizonLength);

    nh.getParam("/optimizer/dRd", OPTobject.dRd);
    nh.getParam("/optimizer/dRa", OPTobject.dRa);
    nh.getParam("/optimizer/m", OPTobject.m);
    nh.getParam("/optimizer/I", OPTobject.I);
    nh.getParam("/optimizer/Lf", OPTobject.Lf);
    nh.getParam("/optimizer/Lr", OPTobject.Lr);
    nh.getParam("/optimizer/Dr", OPTobject.Dr);
    nh.getParam("/optimizer/Df", OPTobject.Df);
    nh.getParam("/optimizer/Cr", OPTobject.Cr);
    nh.getParam("/optimizer/Cf", OPTobject.Cf);
    nh.getParam("/optimizer/Br", OPTobject.Br);
    nh.getParam("/optimizer/Bf", OPTobject.Bf);
    nh.getParam("/optimizer/u_r", OPTobject.u_r);
    nh.getParam("/optimizer/gravity", OPTobject.gravity);
    nh.getParam("/optimizer/Cd", OPTobject.Cd);
    nh.getParam("/optimizer/rho", OPTobject.rho);
    nh.getParam("/optimizer/Ar", OPTobject.Ar);
    nh.getParam("/optimizer/q_slip", OPTobject.q_slip);
    nh.getParam("/optimizer/p_long", OPTobject.p_long);
    nh.getParam("/optimizer/q_n", OPTobject.q_n);
    nh.getParam("/optimizer/q_mu", OPTobject.q_mu);
    nh.getParam("/optimizer/lambda", OPTobject.lambda);
    nh.getParam("/optimizer/q_s", OPTobject.q_s);

    // Initialization of OPTIMIZER
    if(not OPTobject.isRunning()) OPTobject.init();

    ros::spin();

}
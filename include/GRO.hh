#ifndef GRO_HH
#define GRO_HH

#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>

#include "tro/dynamicConfig.h"
#include "nav_msgs/Path.h"

#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/ObjectiveCurv.h"
#include "as_msgs/CarState.h"
#include "as_msgs/Tracklimits.h"

#include "kdtree.h"
// #include "../solver/gro500/include/gro500.h"

using namespace std;
using namespace Eigen;

// This class is for the KDTree
class Point : public std::array<float,2> {
    public :
        static const int DIM = 2;
};

// Trajectory struct
struct trajectory {
    
    MatrixXd Pleft, Pright; // Gates
    int N; // Dimension
    MatrixXd pointsSol, coefsSplinesTrajX, coefsSplinesTrajY;// Points of the minimum curvature trajectory solution and its coefficients of the splines
    VectorXd splinesLengths; // Length of every spline
    MatrixXd pointsTraj2; // Points of the trajectory (their dimension is two times than vector velocity)
    VectorXd radiCurv2; // Their curvature radius
    VectorXd radiCurv;
    VectorXd freeL, freeR; // Space from the pointTraj to the limit of the track, left and right
    MatrixXd pointsTraj; // Equal than pointsTraj2 but taken 1 over 2 elements
    VectorXd velocity; // Velocity profile
    kdt::KDTree<Point> trajTree; // KDTree for the planner

};


// Class GRO
class GRO{

    private:

        // Internal parameters of GRO (Dynamic Reconfigure)
        double dt = 0.05; // Time period mpc
        double axmaxAccel = 4.0; // Longitudinal maximum acceleration [m/s²]
        double axmaxDecel = 8.0; // Longitudinal maximum deceleration (brake) [m/s²]
        double aymax = 15.0; // Lateral maximum acceleration [m/s²]

        // Internal parameters of GRO
        double distBound = 2;
        bool isRun = false; // Flag of initialization
        const static int dimSolver = 500; // This is the dimension accepted for the solver (maximum number of gates)
        bool solverFlag = false; // Flag if using solver or not
        double carWidth = 1.36; // Car external width [m].

        // Internal functions of GRO
        void get_data(as_msgs::Tracklimits &data);
        void get_trajectory();
        void radi_curv();
        void velocity_profile();
        void create_KDTree();

        void gate_generator(as_msgs::Tracklimits &data);
        MatrixXd coefs_splines(VectorXd x);
        MatrixXd matrix_D(int N);
        
        Vector2d get_gate_point(Vector2d  P, Vector2d  Q, Vector2d  O, Vector2d  A, Vector2d  B, Vector4d coefsX, Vector4d coefsY);
        void reduce_points();
        VectorXd polyval(Vector4d coeffs, VectorXd t);
        double polyval2(Vector4d coeffs, double t);
        double integral_length(Vector4d coefsX, Vector4d coefsY);
        double f_accel(int k, double v);
        double f_decel(int k, double v);

        // Planner functions
        MatrixXd planning_curv(const as_msgs::CarState::ConstPtr &data);
        int steps = 60; // How many steps are sent to the mpc (including the actual state)

    public:

        GRO(); // Constructor
        void init(as_msgs::Tracklimits &data); // Main function of initialization
        bool isRunning(); // Flag
        nav_msgs::Path get_path(); // To publish at RVIZ
        as_msgs::ObjectiveArrayCurv plannerGRO_curv(const as_msgs::CarState::ConstPtr &data); // To publish at planner

        as_msgs::Tracklimits read_csv(std::string filename);
        void save_data(std::string filename);

        bool reconfigure (tro::dynamicConfig& config); // Dynamic reconfigure
        void enjoy(); // Speed up

        // Trajectory data
        trajectory traj = trajectory();

        // Paths
        std::string midlinePath; // Tracklimits saved from autocross
        std::string savePath; // Path where to save filtered_points.csv (midline trajectory)

        double securityFactor = 2; // Security factor from 1 to infinte.
        double spacing; // Spacing between points [m]. It is used for the radi_curv function and velocity profile
        double separation = 4; // Separation between gates [m]
        bool visualization; // if true, GRO path is published

};


#endif
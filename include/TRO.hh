#ifndef TRO_HH
#define TRO_HH

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
// #include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <eigen3/Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"

#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/ObjectiveCurv.h"
#include "as_msgs/CarState.h"
#include "as_msgs/Tracklimits.h"

#include "kdtree.h"

using namespace std;
using namespace Eigen;

// This class is for the KDTree
class Point : public array<float,2> {
    public :
        static const int DIM = 2;
};

// Optimized trajectory struct
struct trajectory {

    int dimension; // Number of points interpolated with splines N/splinesStride

    MatrixXd pointsSol, coefsSplinesTrajX, coefsSplinesTrajY;// Optimized trajectory points and its coefficients of the splines
    MatrixXd middlePoints; // Central trajectory points
    MatrixXd pointsTraj; // Optimized trajectory points with MPC spacing (0.025m)
    MatrixXd pointsFirst; // Points of the first spline --> joins car position with optimal traj (with MPC spacing)
    MatrixXd pointsAuxTraj; // [pointsFirst + pointsTraj] --> filled in create_auxKDTree()
    MatrixXd Xopt; // Optimized stages ( 9 x (N+1) )
    MatrixXd centralFree; // Central trajectory free space

    Vector4d coefsFirstX, coefsFirstY; // Coefficients of the first spline --> joins optimized trajectory from the initial position of the car
    VectorXd Pheading; // Heading of central trajectory
    VectorXd splinesLengths; // Length of every spline
    VectorXd radiCurv; // Radius of the optimal trajectory
    VectorXd radiAuxCurv; // [radiFirst + radiCurv] --> filled in create_auxKDTree()
    VectorXd radiFirst; // Radius of the first polynomial points
    VectorXd freeL, freeR; // Space from the pointsTraj to the limit of the track, left and right
    VectorXd Vx; // Vx profile
    VectorXd Vy; // Vy profile
    VectorXd w; // Yaw rate profile
    kdt::KDTree<Point> trajTree; // KDTree for the planner 
    kdt::KDTree<Point> auxTree; // KDTree for the planner with first polynomial 

};

// TRO class
class TRO{

    private:

        // Internal variables/methods of TRO

        int n_states = 7; // Number of states variables
        int n_controls = 2; // Number of controls variables
        int N; // Horizon length from optimization problem 
        int Npar; // Number of parameters from optimization problem 
        int steps = 2000; // How many steps are sent to the mpc (including the actual state)
        int nPointsFirst = 0; // How many points are substituted by pointsFirst

        double T; // discretization of TRO
        double lengthFirst = 0; // Length of first polynomial

        void get_data(); // Read midline.cpp output and transform the data to desired format
        void heading(); // Get heading of the middle trajectory 
        void get_trajectory(); // Get x,y coordinates from calculated MPC stages (Xopt) and interpolate with splines
        void radi_curv(); // Get trajectory and its curvature and each stage of interest (Vx, Vy, w) for MPC
        void create_KDTree(); // Create KDTree for the planner
        void create_auxKDTree(); // Create aux KDTree for the planner
        void changeKDTree(Point &p); // Checks whether its time to change KDTree

        bool isRun = false; // Flag of initialization
        bool firstSpDone = false; // Flag for changing from KDTree (first polynomial --> optimal traj)

        Vector4d coefs_first(double x0, double xf, double b0, double bf);
        VectorXd polyval(Vector4d coeffs, VectorXd &t);
        double polyval(Vector4d coeffs, double t);
        double integral_length(Vector4d coefsX, Vector4d coefsY);
        
        MatrixXd coefs_splines(VectorXd x); // returns the coefficients of the splines that interpolates all N points
        MatrixXd read_csv(string &filename, bool longvec=false, bool firstout = false);
        MatrixXd planning(const as_msgs::CarState::ConstPtr &data); // Prepare msg for plannerTRO()

        pair<double,double> coef_bx_by(double angle); // given the angle, calculate the bx and by coefficients of the spline
        template <typename NUM> NUM mod(NUM x, NUM y); // Norm of (x,y) vector
        template <typename NUM> NUM interpolate(NUM x, NUM x0, NUM x1, NUM y0, NUM y1); // Linear interpolation

    public:

        string optimizedPath; // Output (optimized stages) of TROsolver
        string problemPath; // Problem characteristics of TROsolver
        string middlePointsPath; // Middle trajectory points
        string freeSpacePath; // freeR, freeL from middle trajectory 
        string conesPath; // Path of Cones Loop

        double Tmiddle = 0.05; // discretization of middle trajectory
        double spacing = 0.025; // discretization for MPC
        double v0 = 6; // heuristic parameter representing the velocity of the spline --> used in coef_bx_by()

        int splinesStride = 15; // Stride for calculating less splines (smaller computing time)
        int firstSpId = 2; // (if > 0) index to interpolate with coefs_first() to compute first polynomial

        bool firstSpFlag = false; // first polynomial (spline) flag

        TRO(); // Constructor
        void init(); // Initialization function
        void join_track(const as_msgs::CarState::ConstPtr &data); // Joins first car position with optimized trajectory (better MPC performance)
        bool isRunning(); // Flag

        as_msgs::ObjectiveArrayCurv plannerTRO(const as_msgs::CarState::ConstPtr &data); // Returns msg to publish (output TRO)

        nav_msgs::Path get_path(); // Visualization function --> publishes the whole path
        nav_msgs::Path get_Firstpath();

        as_msgs::Tracklimits read_cones(string filename); // reads file with Cones Loop
        void pubMarkerArray(const as_msgs::Tracklimits &conesLoop, ros::Publisher &pubConesLoopMA); // Publishes markers for Cones Loop

        // Optimized trajectory data
        trajectory traj = trajectory();
};


#endif
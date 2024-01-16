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

#include "../include/TRO.hh"

using namespace std;
using namespace Eigen;

// Constructor
TRO::TRO(){}

// Initialization of TRO
void TRO::init(){

    auto start_time = std::chrono::system_clock::now();

    // Run principal functions
    get_data();
    heading();
    get_trajectory();
    radi_curv();
    create_KDTree();

    // Save final trajectory (debug)
    save_csv(this->savePath);
    
    // Running flag set to true
    isRun = true;

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    ROS_WARN( "TRO ELAPSED TIME: %f ms", elapsed.count()*1000);

}

// Return flag, useful for planner
bool TRO::isRunning(){
    return this->isRun;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

// get_data: read midline.cpp output and transform the data to desired format
void TRO::get_data(){

    MatrixXd problem = read_csv(problemPath);

    N = problem(0,0);
    Npar = problem(2,0);
    T = problem(1,0);

    MatrixXd optimized = read_csv(optimizedPath); 

    MatrixXd middlePoints = read_csv(middlePointsPath,true,false);
    traj.middlePoints = middlePoints;

    MatrixXd freeSpace = read_csv(freeSpacePath,true,false);
    traj.centralFree = freeSpace;

    // Concatenate optimized stages in Xopt ( 9*(N+1) x 1) --> (9 x N+1) )
    Map<MatrixXd> stages(optimized.topRows(n_states*(N+1)).data(),n_states,N+1);
    Map<MatrixXd> control(optimized.bottomRows(n_controls*(N+1)).data(),n_controls,N+1);
    MatrixXd Xopt(stages.rows()+control.rows(), stages.cols());
    Xopt << control, 
            stages;
    traj.Xopt = Xopt;

}

// heading: get heading of the middle trajectory 
void TRO::heading(){
    
    double xv, yv;
    
    VectorXd midX = traj.middlePoints.col(0);
    VectorXd midY = traj.middlePoints.col(1);

    VectorXd Pheading(midX.size()-1);

    for(int i = 0; i < midX.size()-1; i++){

        xv = midX(i+1) - midX(i);
        yv = midY(i+1) - midY(i);
        Pheading(i) = atan2(yv,xv);

    }
    traj.Pheading = Pheading;

}

// get_trajectory: get x,y coordinates from calculated MPC stages (Xopt) and interpolate with splines
void TRO::get_trajectory(){

    MatrixXd finalTraj(N,2);
    VectorXd FreeR(N), FreeL(N);
    const int midTrajStride= T/Tmiddle;
    // We need the middle trajectory data size to be equal to the optimized data size (N)
    Map<VectorXd,0,InnerStride<> > middle_xN(traj.middlePoints.col(0).data(), N, InnerStride<>(midTrajStride));
    Map<VectorXd,0,InnerStride<> > middle_yN(traj.middlePoints.col(1).data(), N, InnerStride<>(midTrajStride));
    Map<VectorXd,0,InnerStride<> > freeR(traj.centralFree.col(0).data(), N, InnerStride<>(midTrajStride));
    Map<VectorXd,0,InnerStride<> > freeL(traj.centralFree.col(1).data(), N, InnerStride<>(midTrajStride));
    Map<VectorXd,0,InnerStride<> > PheadingN(traj.Pheading.data(), N, InnerStride<>(midTrajStride));
    
    // finalTraj.col(0) = traj.middlePoints.col(0) - traj.Xopt.row(4).transpose()*traj.Pheading.array().sin();
    // finalTraj.col(1) = traj.middlePoints.col(1) - traj.Xopt.row(4).transpose()*traj.Pheading.array().cos();

    for(int i = 0; i < N; i++){

        finalTraj(i,0) = middle_xN(i) - traj.Xopt(4,i)*sin(PheadingN(i));
        finalTraj(i,1) = middle_yN(i) + traj.Xopt(4,i)*cos(PheadingN(i));
        FreeR(i) = traj.Xopt(4,i) + freeR(i);
        FreeL(i) = -traj.Xopt(4,i) + freeL(i);
        
    }
    traj.pointsSol = finalTraj; // Optimized trajectory with N points 
    traj.freeL = FreeL;
    traj.freeR = FreeR;

    traj.dimension = N/splinesStride; // Spline dimension (n polynomials/splines)

    Map<VectorXd,0,InnerStride<> > pointsSolNx(traj.pointsSol.col(0).data(),traj.dimension, InnerStride<>(splinesStride)); // Reduce number of points in order to compute splines faster
    Map<VectorXd,0,InnerStride<> > pointsSolNy(traj.pointsSol.col(1).data(),traj.dimension, InnerStride<>(splinesStride));

    // Getting the splines coefficients of the optimized trajectory
    traj.coefsSplinesTrajX = coefs_splines(pointsSolNx);
    traj.coefsSplinesTrajY = coefs_splines(pointsSolNy);

    // Getting the length of every spline of the trajectory
    traj.splinesLengths = VectorXd(traj.dimension);
    for (int i=0; i<traj.dimension; i++){
        traj.splinesLengths(i) = integral_length(traj.coefsSplinesTrajX.row(i), traj.coefsSplinesTrajY.row(i));
    }

}

// radi_curv: get curvature of trajectory
void TRO::radi_curv(){

    // Declare variables 
    int numberPointsPerSpline, totalPoints=0;
    double curvature, dx,cx,bx,dy,cy,by;
    double d_vx, d_vy, d_w, c_vx, c_vy, c_w, b_vx, b_vy, b_w, a_vx, a_vy, a_w;

    // This is the aproximated dimension that will have the vectors
    int aproxL = (int) (traj.splinesLengths.sum()/(this->spacing)) + traj.dimension; 

    // Initializing points, radius, fL, fR, Vx, Vy, w
    MatrixXd points(aproxL, 2);
    VectorXd radius(aproxL);
    VectorXd fL(aproxL), fR(aproxL), VX(aproxL), VY(aproxL), W(aproxL);

    // Setting freeR, freeL to have size = traj.dimension
    Map<VectorXd,0,InnerStride<> > freeL5(traj.freeL.data(), traj.dimension, InnerStride<>(splinesStride));
    Map<VectorXd,0,InnerStride<> > freeR5(traj.freeR.data(), traj.dimension, InnerStride<>(splinesStride));

    // Setting optimized stages to have size = traj.dimension
    VectorXd velocityX = traj.Xopt.row(6);
    VectorXd velocityY = traj.Xopt.row(7);
    VectorXd yawRate = traj.Xopt.row(8);
    Map<VectorXd,0,InnerStride<> > Vx(velocityX.data(), traj.dimension, InnerStride<>(splinesStride));
    Map<VectorXd,0,InnerStride<> > Vy(velocityY.data(), traj.dimension, InnerStride<>(splinesStride));
    Map<VectorXd,0,InnerStride<> > w(yawRate.data(), traj.dimension, InnerStride<>(splinesStride));

    // Cubic interpolation for odometry variables (vx, vy, w)
    MatrixXd coefsVx = coefs_splines(Vx);
    MatrixXd coefsVy = coefs_splines(Vy);
    MatrixXd coefsW  = coefs_splines(w);

    for(int i=0; i < traj.dimension; i++){

        // Getting the coefficients of each polynomial
        dx = traj.coefsSplinesTrajX(i,0);
        cx = traj.coefsSplinesTrajX(i,1);
        bx = traj.coefsSplinesTrajX(i,2);
        dy = traj.coefsSplinesTrajY(i,0);
        cy = traj.coefsSplinesTrajY(i,1);
        by = traj.coefsSplinesTrajY(i,2);

        // Number of points per spline
        numberPointsPerSpline = (int) (traj.splinesLengths(i) / (this->spacing));

        if(firstSpId >0 && i <= firstSpId-1) nPointsFirst += numberPointsPerSpline;

        // Parametrization of the spline, numberPointsPerSpline between 0 and almost 1:
        VectorXd t = VectorXd::LinSpaced(numberPointsPerSpline, 0, 1-1/(double)numberPointsPerSpline);

        // Specific coordinates for particular points (evaluate splines)
        points.block(totalPoints, 0, numberPointsPerSpline, 1) = polyval(traj.coefsSplinesTrajX.row(i), t);
        points.block(totalPoints, 1, numberPointsPerSpline, 1) = polyval(traj.coefsSplinesTrajY.row(i), t);

        VX.segment(totalPoints, numberPointsPerSpline) = polyval(coefsVx.row(i), t);
        VY.segment(totalPoints, numberPointsPerSpline) = polyval(coefsVy.row(i), t);
        W.segment(totalPoints, numberPointsPerSpline)  = polyval(coefsW.row(i), t);

        // Curvature radius formula:
        for(int j=0; j<numberPointsPerSpline; j++){

            radius(totalPoints + j) = pow(sqrt(pow(3*dx*pow(t(j),2)+2*cx*t(j)+bx, 2) + pow(3*dy*pow(t(j),2)+2*cy*t(j)+by, 2)), 3) / ((3*dx*pow(t(j),2)+2*cx*t(j)+bx)*(6*dy*t(j)+2*cy) - (3*dy*pow(t(j),2)+2*cy*t(j)+by)*(6*dx*t(j)+2*cx));

            // We suppose freeR, freeL stay constant over points in the same polynomial
            fL(totalPoints + j) = freeL5(i);
            fR(totalPoints + j) = freeR5(i);

            // if(i > 0){ // We interpolate Vx,Vy,w over the points in the same polynomial
            //     VX(totalPoints + j) = interpolate<double>(totalPoints + j, totalPoints-1, totalPoints+numberPointsPerSpline-1, Vx(i-1), Vx(i));
            //     VY(totalPoints + j) = interpolate<double>(totalPoints + j, totalPoints-1, totalPoints+numberPointsPerSpline-1, Vy(i-1), Vy(i));
            //     W(totalPoints + j)  = interpolate<double>(totalPoints + j, totalPoints-1, totalPoints+numberPointsPerSpline-1, w(i-1), w(i));

            //     // VX(totalPoints + j) = Vx(i);
            //     // VY(totalPoints + j) = Vy(i);
            //     // W(totalPoints + j) = w(i);

            // }else if(i==0){
            //     VX(totalPoints + j) = interpolate<double>(j, 0, numberPointsPerSpline-1, Vx(traj.dimension), Vx(i));
            //     VY(totalPoints + j) = interpolate<double>(j, 0, numberPointsPerSpline-1, Vy(traj.dimension), Vy(i));
            //     W(totalPoints + j)  = interpolate<double>(j, 0, numberPointsPerSpline-1, w(traj.dimension), w(i));
            // }
        }

        totalPoints += numberPointsPerSpline;
    }

    // Now that it is known the exact value of the size of the vector, they are resized (last empty places are removed)
    traj.radiCurv = radius.head(totalPoints);
    traj.pointsTraj = points.topRows(totalPoints);
    traj.freeL = fL.head(totalPoints); traj.freeR = fR.head(totalPoints);
    traj.Vx = VX.head(totalPoints); traj.Vy = VY.head(totalPoints); traj.w = W.head(totalPoints);

}

// join_track: computes first polynomial coefficients in order to join the optimized trajectory with initial car position
void TRO::join_track(const as_msgs::CarState::ConstPtr &data){

    // Heading of the spline (polynomial) at the start (bs = [bxs,bys])
    pair<double,double> bs = coef_bx_by(data->odom.heading);

    if(firstSpId > 0){
        Vector4d coefsX = traj.coefsSplinesTrajX.row(firstSpId-1);
        Vector4d coefsY = traj.coefsSplinesTrajY.row(firstSpId-1);

        // Heading at the joining point (bf = [bxf,byf])
        pair<double,double> bf (coefsX(2),coefsY(2)); 

        // Compute polynomial (spline) coefs
        traj.coefsFirstX = coefs_first(data->odom.position.x, polyval(traj.coefsSplinesTrajX.row(firstSpId-1),1.0), bs.first, bf.first);
        traj.coefsFirstY = coefs_first(data->odom.position.y, polyval(traj.coefsSplinesTrajY.row(firstSpId-1),1.0), bs.second, bf.second);

        // Length of the spline
        lengthFirst = integral_length(traj.coefsFirstX, traj.coefsFirstY);

        // Number of points per spline
        int numberPointsPerSpline = (int) (lengthFirst / (this->spacing));

        // Parametrization of the spline, numberPointsPerSpline between 0 and almost 1:
        VectorXd t = VectorXd::LinSpaced(numberPointsPerSpline, 0, 1-1/(double)numberPointsPerSpline);

        // Specific coordinates for particular points
        MatrixXd pointsFirst(numberPointsPerSpline,2); 
        pointsFirst.block(0, 0, numberPointsPerSpline, 1) = polyval(traj.coefsFirstX, t);
        pointsFirst.block(0, 1, numberPointsPerSpline, 1) = polyval(traj.coefsFirstY, t);
        traj.pointsFirst = pointsFirst;

        // Getting the coefficients of each polynomial
        double dx,cx,bx,dy,cy,by;
        dx = traj.coefsFirstX(0);
        cx = traj.coefsFirstX(1);
        bx = traj.coefsFirstX(2);
        dy = traj.coefsFirstY(0);
        cy = traj.coefsFirstY(1);
        by = traj.coefsFirstY(2);

        // Curvature radius formula:
        VectorXd radius(numberPointsPerSpline);
        for(int j=0; j < numberPointsPerSpline; j++){
            radius(j) = pow(sqrt(pow(3*dx*pow(t(j),2)+2*cx*t(j)+bx, 2) + pow(3*dy*pow(t(j),2)+2*cy*t(j)+by, 2)), 3) / ((3*dx*pow(t(j),2)+2*cx*t(j)+bx)*(6*dy*t(j)+2*cy) - (3*dy*pow(t(j),2)+2*cy*t(j)+by)*(6*dx*t(j)+2*cx));
        }
        traj.radiFirst = radius;
        create_auxKDTree(); // Create auxiliar KDTree with first polynomial + pointsTraj
        firstSpFlag = true;
    }

}

// Create KDTree for the planner with optimized traj
void TRO::create_KDTree(){

    // Initialize vector
    vector<Point> tr;

    // Fill vector
    for(int i=0; i < traj.pointsTraj.rows(); i++){
        Point p;
        p[0] = traj.pointsTraj(i,0);
        p[1] = traj.pointsTraj(i,1);

        tr.push_back(p);
    }

    // Initialize KDTree
    traj.trajTree.build(tr);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

// Create auxiliar KDTree for the planner with pointsFirst + pointsTraj
void TRO::create_auxKDTree(){

    // Initialize vector
    vector<Point> tr;

    // Fill vector with first polynomial
    for(int j=0; j < traj.pointsFirst.rows(); j++){
        Point q;
        q[0] = traj.pointsFirst(j,0);
        q[1] = traj.pointsFirst(j,1);

        tr.push_back(q);
    }

    // Fill vector with optimized traj
    for(int i=nPointsFirst-1; i < traj.pointsTraj.rows(); i++){
        Point p;
        p[0] = traj.pointsTraj(i,0);
        p[1] = traj.pointsTraj(i,1);

        tr.push_back(p);
    }

    // Initialize KDTree
    traj.auxTree.build(tr);

    // Concatenate pointsFirst + pointsTraj
    MatrixXd pTraj(traj.pointsTraj.rows()-nPointsFirst,traj.pointsTraj.cols());
    pTraj = traj.pointsTraj.bottomRows(traj.pointsTraj.rows()-nPointsFirst);

    MatrixXd pointsAuxTraj(traj.pointsFirst.rows()+pTraj.rows(), traj.pointsFirst.cols());

    pointsAuxTraj << traj.pointsFirst, 
                        pTraj;
    traj.pointsAuxTraj = pointsAuxTraj;

    // Concatenate radiFirst + radiCurv
    Map<VectorXd> radiusTraj(traj.radiCurv.tail(traj.radiCurv.size()-nPointsFirst).data(),traj.radiCurv.size()-nPointsFirst);
    VectorXd radius(traj.radiFirst.size()+radiusTraj.size());
    radius << traj.radiFirst, radiusTraj;
    traj.radiAuxCurv = radius;

}

// coefs_splines: returns the coefficients of the splines that interpolates all N points
/* Explanation:
        coefs = [ d1 c1 b1 a1
                  d2 c2 b2 a2
                    .....
                  dn cn bn an ]
        where pi(t) = di*t^3 + ci*t^2 + bi*t + ai;
        with 0<t<1 */
MatrixXd TRO::coefs_splines(VectorXd x){

    // Getting the lenght
    int N = x.size();

    // Initializing matrixs
    MatrixXd Deqs = MatrixXd::Zero(4*N,4*N);
    MatrixXd matrixEqs(4,8);
    matrixEqs << 0,  0,  0, 1, 0, 0, 0,  0,
                -1, -2, -3, 0, 1, 0, 0,  0,
                    0, -1, -3, 0, 0, 1, 0,  0,
                    0,  0,  0, 1, 1, 1, 1, -1; // Matrix 4x8

    // Building Deqs matrix

    // Building core:
    for(int i=2; i<=N-1; i++) {
        Deqs.block(4*(i-1),4*(i-1)-3, 4, 8) = matrixEqs;
    }
    // Building corners:
    Deqs.topLeftCorner(4,5) = matrixEqs.rightCols(5);
    Deqs.topRightCorner(4,3) = matrixEqs.leftCols(3);
    Deqs.bottomLeftCorner(4,1) = matrixEqs.rightCols(1);
    Deqs.bottomRightCorner(4,7) = matrixEqs.leftCols(7);

    // Creating column of x with zeros
    VectorXd xp = VectorXd::Zero(4*N,1);
    for(int i=0; i<=N-1; i++){
        xp(4*i) = x(i);
    }

    // Obtaining coefficients
    VectorXd coefsVec(4*N,1);
    // MatrixXd Dinv = Deqs.inverse();
    // coefsVec = Dinv*xp;
    coefsVec = Deqs.partialPivLu().solve(xp);

    // Reshape and reorganizing
    Map<MatrixXd> coefsMat1(coefsVec.data(), 4,N); 
    MatrixXd coefsMat2 = coefsMat1.transpose();
    MatrixXd coefsMat = coefsMat2.rowwise().reverse();

    return coefsMat;

}

// coefs_first: calculate polynomial coefficients given two points to interpolate, initial and final heading
/*      coefs = [ d, c, b, a ]
        where p(t) = d*t^3 + c*t^2 + b*t + a;
        with 0<t<1 */
Vector4d TRO::coefs_first(double x0, double xf, double b0, double bf){

    // Building equation matrix
    MatrixXd Deqs(3,3);
    Deqs << 1, 0, 0,
            1, 1, 1,
            0, 2, 3; 

    // Equality vector
    Vector3d xp;
    xp(0) = x0;
    xp(1) = xf - b0;
    xp(2) = bf - b0;

    // Solve
    Vector3d coefs3d = Deqs.partialPivLu().solve(xp);

    // Fill coefficients vector
    Vector4d coefsVec;
    coefsVec(0) = coefs3d(2);
    coefsVec(1) = coefs3d(1);
    coefsVec(2) = b0;
    coefsVec(3) = coefs3d(0);
    return coefsVec;
}

// Auxiliar function: evaluates a 3rd order polynomial given its parameters and a vector t
// (same as Matlab)
VectorXd TRO::polyval(Vector4d coeffs, VectorXd &t) {

    VectorXd results(t.size());
    double result;
    int aux;
    for (int i=0; i<t.size(); i++){
        result = 0;
        aux = 0;
        for (int deg = coeffs.size() - 1; deg >= 0; deg--){
           result += coeffs(aux) * pow(t(i), deg);
           aux++;
        }
        results(i) = result;
    }
    return results;
}

// Same but evaluates polynomial to only one point, so t is a double not a vector
double TRO::polyval(Vector4d coeffs, double t) {
    return coeffs(0)*pow(t,3) + coeffs(1)*pow(t,2) + coeffs(2)*t + coeffs(3);
}

// integral_length: calculate the length of a 3rd order spline given its coefficients. Rectangular integration.
double TRO::integral_length(Vector4d coefsX, Vector4d coefsY){
    
    // This is the precision of integration
    double diffx = 0.0005;
    double dx, dy, cx, cy, bx, by, t, n, area, function;
    
    dx = coefsX(0);
    cx = coefsX(1);
    bx = coefsX(2);
    dy = coefsY(0);
    cy = coefsY(1);
    by = coefsY(2);
    n = 1/diffx;
    area = 0;

    // Numerical integration with the area of rectangles
    for(int j=0; j<n; j++){
        t = j*diffx+diffx/2;
        function = sqrt(pow(3*dx*pow(t, 2) + 2*cx*t + bx, 2) + pow(3*dy*pow(t, 2) + 2*cy*t + by, 2));
        area += function*diffx;
    }
    return area;
}

//Auxiliar function: given the angle, calculate the bx and by coefficients of the spline
pair<double,double> TRO::coef_bx_by(double angle){

    // Formulas for b coefficients
    double bx = v0*sqrt(1/(1+pow(tan(angle),2)));
    double by = v0*sqrt(1/(1+pow(1/tan(angle),2)));

    // Define sign of x negative if it is in the first or fourth quadrant
    if(angle > M_PI/2 and angle < 3*M_PI/2){
        bx *= -1;
    }

    // Define sign of y negative if it is in the third or fourth quadrant
    if(angle > M_PI){
        by *= -1;
    }

    return pair<double,double> (bx, by);
}

template<typename NUM>
NUM TRO::mod(NUM x, NUM y){
    return sqrt(pow(x,2)+pow(y,2));
}

template <typename NUM>
NUM TRO::interpolate(NUM x, NUM x0, NUM x1, NUM y0, NUM y1){
    return y0 + (x - x0)*(y1 - y0)/(x1-x0);
}

// read_csv files (longvec is used for reserve method of vector / firstout argument is used to avoid first column of indexes)
MatrixXd TRO::read_csv(std::string &filename, bool longvec, bool firstout){ 

    MatrixXd FileContent; // Eigen Matrix to store csv data
    std::vector<std::vector<double>> result; // Vector of vectors to store each line content
    if(longvec) result.reserve(4000);

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("TRO::read_csv Could not open file");

    // Helper vars
    std::string line;
    int ncols = 0;

    // Read data, line by line
    while(std::getline(myFile, line)){

        // Create a stringstream of the current line
        std::vector<double> linevec;
        std::stringstream ss(line);
        std::string val;

        while(std::getline(ss,val,',')){
            linevec.push_back(std::stod(val));
        }
        result.push_back(linevec);
        ncols = linevec.size();
    }

    int first = 0;
    if(firstout){
        FileContent.resize(result.size(),ncols-1); // Eigen Matrix to store all csv content
        first++;
    }else{
        FileContent.resize(result.size(),ncols); 
    }

    // Store csv data into Eigen Matrix
    long int cols, rows;
    rows = 0;
    vector<vector<double>>::iterator row;
    vector<double>::iterator col;
    for(row = result.begin(); row != result.end(); ++row){
        cols = 0;
        for(col = row->begin() + first; col != row->end(); ++col){
            FileContent(rows,cols) = *col; 
            cols++;
        }
        rows++;
    }

    // Close file
    myFile.close();
    return FileContent;
}

void TRO::save_csv(string filepath){
    try{

        std::ofstream traj_file;
        traj_file.open(filepath + "optimal_points.csv", std::ofstream::out | std::ofstream::trunc);

        for(int i=0; i<traj.pointsTraj.rows(); i++){
            traj_file << traj.pointsTraj(i,0) << ",";
            traj_file << traj.pointsTraj(i,1) << ",";
            traj_file << traj.Vx(i) << "\n";
        }

        // Remeber to close the file!
        traj_file.close();

        ROS_INFO_STREAM("OPTIMAL TRACK data SAVED AT: " << filepath);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "TRO::save_csv Exception was thrown: \n" << e.what() );
    }
}

// Auxiliar function: checks whether its time to change the KDTree used by the planner
void TRO::changeKDTree(Point &p){

    double dist = mod<double>(p[0], p[1]); // Distance from origin
    if(dist > this->lengthFirst) this->firstSpDone = true; 
    // if dist is greater than the first polynomial length we change from auxTree to trajTree 
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Planner functions--------------------------------------------------

// Main function of the planner: creates the message and returns it.
as_msgs::ObjectiveArrayCurv TRO::plannerTRO(const as_msgs::CarState::ConstPtr &data){

    // Create message
    as_msgs::ObjectiveArrayCurv msg;
    msg.header = data->header;

    // Planning
    MatrixXd plan = planning(data);

    // Fill the message
    for(int i=0; i < steps; i++){
        as_msgs::ObjectiveCurv objective = as_msgs::ObjectiveCurv();

        objective.x = plan(i,0);
        objective.y = plan(i,1);
        objective.s = plan(i,2);
        objective.k = plan(i,3);
        objective.vx = plan(i,4);
        objective.vy = plan(i,5);
        objective.w = plan(i,6);
        objective.L = plan(i,7);
        objective.R = plan(i,8);
        
        msg.objectives.push_back(objective);
    }

    msg.smax = traj.pointsTraj.rows() * spacing;

    return msg;
}


// Auxiliar function of the planner: creates the plan matrix of the curvature-MPC dimensions.
MatrixXd TRO::planning(const as_msgs::CarState::ConstPtr &data){
    
    // Actual position of the car
    Point state;
    state[0] = data->odom.position.x;
    state[1] = data->odom.position.y;

    // Initialize planning matrix
    MatrixXd plan(steps, 9); // [x, y, s, k, vx, vy, w, L, R]

    changeKDTree(state);

    if(this->firstSpFlag && !this->firstSpDone){
        // Search the nearest point of the trajectory
        int nnid = traj.auxTree.nnSearch(state);

        // All steps
        for(int i=0; i < steps; i++){
            if(nnid >= traj.pointsAuxTraj.rows()) nnid -= traj.pointsAuxTraj.rows(); // restart loop 

            plan(i,0) = traj.pointsAuxTraj(nnid,0);
            plan(i,1) = traj.pointsAuxTraj(nnid,1);
            plan(i,2) = nnid*spacing;
            plan(i,3) = 1/traj.radiAuxCurv(nnid);
            plan(i,4) = 30;
            plan(i,5) = 0;
            plan(i,6) = 0;
            plan(i,7) = 1.5;
            plan(i,8) = 1.5;

            nnid++;
        }

    }else{
        // Search the nearest point of the trajectory
        int nnid = traj.trajTree.nnSearch(state);

        // All steps
        for(int i=0; i < steps; i++){
            if(nnid >= traj.pointsTraj.rows()) nnid -= traj.pointsTraj.rows(); // restart loop 

            plan(i,0) = traj.pointsTraj(nnid,0);
            plan(i,1) = traj.pointsTraj(nnid,1);
            plan(i,2) = nnid*spacing;
            plan(i,3) = 1/traj.radiCurv(nnid);
            plan(i,4) = traj.Vx(nnid);
            plan(i,5) = traj.Vy(nnid);
            plan(i,6) = traj.w(nnid);
            plan(i,7) = traj.freeL(nnid);
            plan(i,8) = traj.freeR(nnid);

            nnid++;
        }

    }

    return plan;
}

// Visualization function that creates the message to plot the trajectory in RVIZ
nav_msgs::Path TRO::get_path(){

    // Init message:
    nav_msgs::Path pathMsg;
	geometry_msgs::PoseStamped pose;

    pathMsg.header.stamp = ros::Time::now();
    pathMsg.header.frame_id = "global"; 

    for(unsigned int i = 0; i < traj.pointsTraj.rows(); i++ ){

        pose.pose.position.x = traj.pointsTraj(i, 0);
        pose.pose.position.y = traj.pointsTraj(i, 1);

        pathMsg.poses.push_back(pose);
        
    }

    return pathMsg;
}

nav_msgs::Path TRO::get_Firstpath(){

    // Init message:
    nav_msgs::Path pathMsg;
	geometry_msgs::PoseStamped pose;

    pathMsg.header.stamp = ros::Time::now();
    pathMsg.header.frame_id = "global";

    for(unsigned int i = 0; i < traj.pointsFirst.rows(); i++ ){

        pose.pose.position.x = traj.pointsFirst(i, 0);
        pose.pose.position.y = traj.pointsFirst(i, 1);

        pathMsg.poses.push_back(pose);
        
    }
    return pathMsg;
}


// Visualization function that publishes markers for conesLoop
void TRO::pubMarkerArray(const as_msgs::Tracklimits &conesLoop, ros::Publisher &pubConesLoopMA) {

    visualization_msgs::MarkerArray ma;
    ma.markers.reserve(conesLoop.left.size()+conesLoop.right.size());
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.header.frame_id = "global";
    m.header.stamp = ros::Time::now();
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 1.0;
    m.color.a = 1.0;
    int id = 0;
    m.color.b = 1.0;
    m.color.g = 0.0;
    m.color.r = 0.0;
    for (int i = 0; i < conesLoop.left.size(); i++) {
        m.id = id++;
        m.pose.position.x = conesLoop.left[i].position_global.x;
        m.pose.position.y = conesLoop.left[i].position_global.y;
        ma.markers.push_back(m);
    }
    m.color.b = 0.0;
    m.color.g = 1.0;
    m.color.r = 1.0;
    for (int i = 0; i < conesLoop.right.size(); i++) {
        m.id = id++;
        m.pose.position.x = conesLoop.right[i].position_global.x;
        m.pose.position.y = conesLoop.right[i].position_global.y;
        ma.markers.push_back(m);
    }
    pubConesLoopMA.publish(ma);
}

as_msgs::Tracklimits TRO::read_cones(std::string filename){ 

    // Init message:
    as_msgs::Tracklimits midlineMsg;

    midlineMsg.stamp = ros::Time::now();

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("TRO::read_csv Could not open file");

    // Helper vars
    std::string line;

    // Read data, line by line
    while(std::getline(myFile, line)){

        // Create a stringstream of the current line
        std::vector<double> linevec;
        std::stringstream ss(line);
        std::string val;

        while(std::getline(ss,val,' ')){
            linevec.push_back(std::stod(val));
        }

        as_msgs::Cone cone;
        cone.position_global.x = linevec[0];
        cone.position_global.y = linevec[1];

        if(linevec[2] == 1){
            midlineMsg.left.push_back(cone);
        } else{
            midlineMsg.right.push_back(cone);
        }
    }

    // Close file
    myFile.close();

    return midlineMsg;
}
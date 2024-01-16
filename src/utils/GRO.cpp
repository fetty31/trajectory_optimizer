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

// Namespaces
using namespace std;
using namespace Eigen;


// Constructor
GRO::GRO() {}

// Main function: initialization of GRO
void GRO::init(as_msgs::Tracklimits &data){

    auto start_time = std::chrono::system_clock::now();
    
    // Run all principal functions
    get_data(data);

    get_trajectory();

    radi_curv();

    velocity_profile();

    // Running flag to true
    isRun = true;
   
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

	ROS_WARN( "MIDLINE ELAPSED TIME: %f ms", elapsed.count() *1000);

}

// Return flag, useful for planner
bool GRO::isRunning(){
    return this->isRun;
}

// Function to speed up
void GRO::enjoy(){
    aymax += 1;
    if(axmaxDecel < 30){
        axmaxDecel += 0.5;
    }
    if(axmaxAccel < 30){
        axmaxAccel += 0.5;
    }
    velocity_profile();
}



/////////////////////////////////////////////////////////////////////////////////////////////////
//////// From here, all the principal functions /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

// Principal function: reads the cones messages and processes them correctly to the trajectory struct
void GRO::get_data(as_msgs::Tracklimits &data) {

    gate_generator(data);

    if (solverFlag){
        // Now we apply the car width, which means strenching the gates.
        Vector2d vector;
        for(int i = 0; i < traj.N; i++){
            vector = (traj.Pright.row(i)-traj.Pleft.row(i)).normalized();
            traj.Pleft.row(i) += vector*carWidth/2;
            traj.Pright.row(i) -= vector*carWidth/2;
        }

        // Now we apply the security factor, which means strenching the gates.
        for(int i = 0; i < traj.N; i++){
            vector = traj.Pright.row(i)-traj.Pleft.row(i);
            traj.Pleft.row(i) += vector/2 * (1-1/securityFactor);
            traj.Pright.row(i) -= vector/2 * (1-1/securityFactor);
        }
    }

}

// Principal function: apply algorithm of minimum curvature
void GRO::get_trajectory(){

    traj.pointsSol = (traj.Pleft + traj.Pright)/2;

    // cout << "pointsSol\n";
    // cout << traj.pointsSol << endl;

    // Destroy points that are too close each other
    reduce_points();

    // Getting the splines coefficients (they are used in radi_curv function)
    traj.coefsSplinesTrajX = coefs_splines(traj.pointsSol.col(0));
    traj.coefsSplinesTrajY = coefs_splines(traj.pointsSol.col(1));

    // cout << "coefsX:\n";
    // cout << traj.coefsSplinesTrajX << endl;

    // cout << "coefsY\n";
    // cout << traj.coefsSplinesTrajY << endl;

    // Getting the length of every spline of the trajectory
    traj.splinesLengths = VectorXd(traj.N);
    for (int i=0; i<traj.N; i++){
        traj.splinesLengths(i) = integral_length(traj.coefsSplinesTrajX.row(i), traj.coefsSplinesTrajY.row(i));
    }

}

// Principal function: calculates the curvature radius of the trajectory
void GRO::radi_curv(){

    // Declare variables 
    int numberPointsPerSpline, totalPoints=0;
    double curvature, dx,cx,bx,dy,cy,by;

    // This is the aproximated dimension that will have the vectors
    int aproxL = (int) (traj.splinesLengths.sum()/(this->spacing/2)) + traj.N; // The spacing is divided by 2 for the Runge-Kutta method used after

    // cout << "aproxL: " << aproxL << endl;
    // cout << "splinesLengths: " << traj.splinesLengths.sum() << endl;
    // cout << "N: " << traj.N << endl;

    // Initializing points and radius
    MatrixXd points(aproxL, 2);
    VectorXd radius(aproxL), fL(aproxL), fR(aproxL);
    RowVectorXd pLR;

    for(int i=0; i < traj.N; i++){

        // Getting the coefficients for the formula
        dx = traj.coefsSplinesTrajX(i,0);
        cx = traj.coefsSplinesTrajX(i,1);
        bx = traj.coefsSplinesTrajX(i,2);
        dy = traj.coefsSplinesTrajY(i,0);
        cy = traj.coefsSplinesTrajY(i,1);
        by = traj.coefsSplinesTrajY(i,2);

        // Number of points per spline
        numberPointsPerSpline = (int) (traj.splinesLengths(i) / (this->spacing/2)); 

        // Parametrization of the spline, numberPointsPerSpline between 0 and almost 1:
        VectorXd t = VectorXd::LinSpaced(numberPointsPerSpline, 0, 1-1/(double)numberPointsPerSpline);

        // Specific coordinates for particular points (the third is the spline index)
        points.block(totalPoints, 0, numberPointsPerSpline, 1) = polyval(traj.coefsSplinesTrajX.row(i), t);
        points.block(totalPoints, 1, numberPointsPerSpline, 1) = polyval(traj.coefsSplinesTrajY.row(i), t);

        // Curvature radius formula:
        for(int j=0; j<numberPointsPerSpline; j++){
            radius(totalPoints + j) = pow(sqrt(pow(3*dx*pow(t(j),2)+2*cx*t(j)+bx, 2) + pow(3*dy*pow(t(j),2)+2*cy*t(j)+by, 2)), 3) / ((3*dx*pow(t(j),2)+2*cx*t(j)+bx)*(6*dy*t(j)+2*cy) - (3*dy*pow(t(j),2)+2*cy*t(j)+by)*(6*dx*t(j)+2*cx));
        }

        // Both sides of the track. Here is calculated the free space towards the left and towards the right
        for(int j=0; j<numberPointsPerSpline; j++){

            // Left side
            pLR = (t(j)*traj.Pleft.row((i+1)%traj.N) + (1-t(j))*traj.Pleft.row(i));
            fL(totalPoints + j) = (pLR - points.row(totalPoints + j)).norm();

            // Right side
            pLR = (t(j)*traj.Pright.row((i+1)%traj.N) + (1-t(j))*traj.Pright.row(i));
            fR(totalPoints + j) = (pLR - points.row(totalPoints + j)).norm();

        }

        totalPoints += numberPointsPerSpline;
    }

    // Now it is known the exact value of the size of the vector, it is resized (last empty places are removed)
    traj.radiCurv2 = radius.head(totalPoints);
    traj.pointsTraj2 = points.topRows(totalPoints);

    Map<VectorXd,0,InnerStride<2> > fl(fL.head(totalPoints).data()+1, fL.head(totalPoints).size()/2);
    traj.freeL = fl;

    Map<VectorXd,0,InnerStride<2> > fr(fR.head(totalPoints).data()+1, fR.head(totalPoints).size()/2);
    traj.freeR = fr;
}

// Principal function: obtains the velocity profile for the points calculated with radi_curv function
void GRO::velocity_profile(){

    // Discarding the half of the values calculated in radi_curv function (necessary for Runge-Kutta method)
    Map<VectorXd,0,InnerStride<2> > rcurv(traj.radiCurv2.data()+1, traj.radiCurv2.size()/2);
    traj.radiCurv = rcurv;

    // Maximum speed taking into account only the curvature
    VectorXd velRad = (aymax*rcurv.cwiseAbs()).cwiseSqrt();
    // Algorithm applied

        // Initialize Runge-Kutta coefficients
        double m1,m2,m3,m4;

        // First step: restrain limit acceleration

            VectorXd vAccel(velRad.size()); // Initialize vector
            vAccel(0) = velRad(0); // Initial condition

            for(int k = 0; k+1 < velRad.size(); k++){

                // Runge-Kutta method
                m1 = f_accel(2*k    ,  vAccel(k));
                m2 = f_accel(2*k+1  ,  vAccel(k) + m1 * spacing/2);
                m3 = f_accel(2*k+1  ,  vAccel(k) + m2 * spacing/2);
                m4 = f_accel(2*k+2  ,  vAccel(k) + m3 * spacing);
                vAccel(k+1) = vAccel(k) + (m1+2*m2+2*m3+m4)*spacing/6;

                // It is limited always below vRad
                if (vAccel(k+1) > velRad(k+1)){
                    vAccel(k+1) = velRad(k+1);
                }

            }

        // Second step: restrain limit deceleration
    
            VectorXd vFinal(velRad.size()); // Initialize vector
            vFinal(0) = vAccel(vAccel.size()-1); // Initial condition (is the final speed calculated before)

            for(int k = 0; k+1 < velRad.size(); k++){

                // Runge-Kutta method
                m1 = f_decel(2*k    ,  vFinal(k));
                m2 = f_decel(2*k+1  ,  vFinal(k) + m1 * spacing/2);
                m3 = f_decel(2*k+1  ,  vFinal(k) + m2 * spacing/2);
                m4 = f_decel(2*k+2  ,  vFinal(k) + m3 * spacing);
                vFinal(k+1) = vFinal(k) + (m1+2*m2+2*m3+m4)*spacing/6;

                // It is limited always below vRad
                if (vFinal(k+1) > vAccel.reverse()(k+1)){
                    vFinal(k+1) = vAccel.reverse()(k+1);
                }

            }

        traj.velocity = vFinal.reverse();

    // Apply minimum speed (it is necessary)
    double minVel = spacing/dt;
    for (int i=0; i<traj.velocity.size(); i++){
        if (traj.velocity(i) < minVel) traj.velocity(i) = minVel;
    }

    // Resizing the vector pointsTraj2 because it has the double number of points than velocity
    Map<MatrixXd,0,InnerStride<2> > points(traj.pointsTraj2.data()+1, traj.pointsTraj2.rows()/2, traj.pointsTraj2.cols());
    traj.pointsTraj = points;

}

// Create KDTree for the planner
void GRO::create_KDTree(){
    
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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////  From here, all the auxiliar functions      ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// EDO models of the acceleration physics, used in velocity_profile function
double GRO::f_accel(int k, double v){
    // The if condition is in order to avoid complex numbers
    if(v < sqrt(traj.radiCurv2(k) * aymax) ){
        return axmaxAccel/v * sqrt( 1 - pow(pow(v,2) / (abs(traj.radiCurv2(k)) * aymax),2) ); // EDO Model
    } else {
        return 0; // Can't accelerate more
    }
}
double GRO::f_decel(int k, double v){
    // The if condition is in order to avoid complex numbers
    if(v < sqrt(traj.radiCurv2.reverse()(k) * aymax) ){
        return axmaxDecel/v * sqrt( 1 - pow(pow(v,2) / (abs(traj.radiCurv2.reverse()(k)) * aymax),2) ); // EDO Model
    } else {
        return 0; // Can't decelerate more
    }
}

// Auxiliar function: given the raw data from /cones/loop, it generates the gates
// in the global variables traj.Pleft and traj.Pright
void GRO::gate_generator(as_msgs::Tracklimits &data){

    MatrixXd conesRight(data.right.size(),2), conesLeft(data.left.size(),2);

     for(int i=0; i < data.right.size(); i++){
        conesRight(i,0) = data.right[i].position_global.x;
        conesRight(i,1) = data.right[i].position_global.y;
    }

    for(int i=0; i < data.left.size(); i++){
        conesLeft(i,0) = data.left[i].position_global.x;
        conesLeft(i,1) = data.left[i].position_global.y;
    }

    // Save conesRef as the side with less cones
    MatrixXd conesRef, conesOther;
    if (data.left.size() < data.right.size()){
        conesRef = conesLeft;
        conesOther = conesRight;
    } else {
        conesRef = conesRight;
        conesOther = conesLeft;
    }

    // Create KDTree with the other cones
    vector<Point> tr;
    for(int i=0; i < conesOther.rows(); i++){
        Point p;
        p[0] = conesOther(i,0);
        p[1] = conesOther(i,1);

        tr.push_back(p);
    }
    kdt::KDTree<Point> othersTree(tr);


    // Interpolate track limit reference with splines
    MatrixXd interpolatedRefCoefsX = coefs_splines(conesRef.col(0));
    MatrixXd interpolatedRefCoefsY = coefs_splines(conesRef.col(1));
    MatrixXd interpolatedOthCoefsX = coefs_splines(conesOther.col(0));
    MatrixXd interpolatedOthCoefsY = coefs_splines(conesOther.col(1));

    // Getting the length of every spline of the track limit
    VectorXd trackLimLengths(conesRef.rows());
    for (int i=0; i<conesRef.rows(); i++){
        trackLimLengths(i) = integral_length(interpolatedRefCoefsX.row(i), interpolatedRefCoefsY.row(i));
    }

    
    // Now, getting points equally separated

    // Number of gates that will be created

    traj.N = trackLimLengths.sum()/separation;

    separation = trackLimLengths.sum()/((double) traj.N);

    // Initialize vector of points
    MatrixXd Pref(traj.N,2), Pother(traj.N,2);

    // First gate
    Pref.row(0) = conesRef.row(0);

    Vector2d O,Q,A,B,R;
    // Previous point of P

    O(0) = polyval2(interpolatedRefCoefsX.row(conesRef.rows()-1) , 0.95);
    O(1) = polyval2(interpolatedRefCoefsY.row(conesRef.rows()-1) , 0.95);
    // Next point of P
    Q(0) = polyval2(interpolatedRefCoefsX.row(0) , 0.05);
    Q(1) = polyval2(interpolatedRefCoefsY.row(0) , 0.05);

    // Two nearests of P
    Point p;
    p[0] = Pref(0,0);
    p[1] = Pref(0,1);
    vector<int> nnids = othersTree.knnSearch(p,2);
    A = conesOther.row(nnids[0]);
    B = conesOther.row(nnids[1]);

    // Get the minimum of nnids
    int nnid;
    if (nnids[0] < nnids[1]) nnid = nnids[0];
    else nnid = nnids[1];


    // The other side of the first gate
    Pother.row(0) = get_gate_point(Pref.row(0) , Q , O , A , B, interpolatedOthCoefsX.row(nnid), interpolatedOthCoefsY.row(nnid));

    // Now the same but for all the other gates
    
    // Auxiliar variables
    double dist, t=0;
    int nspline = 0;

    for(int i=1; i<traj.N; i++){
        dist = trackLimLengths(nspline)*(1-t);
        while(dist < separation){
            nspline++;
            dist += trackLimLengths(nspline);
        }
        t = 1 - (dist - separation)/trackLimLengths(nspline);

        // Save point ref
        Pref(i,0) = polyval2(interpolatedRefCoefsX.row(nspline) , t);
        Pref(i,1) = polyval2(interpolatedRefCoefsY.row(nspline) , t);

        // Calculate point of the gate at the other track limit
        // Previous point of P
        O(0) = polyval2(interpolatedRefCoefsX.row(nspline) , t-0.05);
        O(1) = polyval2(interpolatedRefCoefsY.row(nspline) , t-0.05);
        // Next point of P
        Q(0) = polyval2(interpolatedRefCoefsX.row(nspline) , t+0.05);
        Q(1) = polyval2(interpolatedRefCoefsY.row(nspline) , t+0.05);

        // Two nearests of P
        Point p;
        p[0] = Pref(i,0);
        p[1] = Pref(i,1);
        vector<int> nnids = othersTree.knnSearch(p,2);
        A = conesOther.row(nnids[0]);
        B = conesOther.row(nnids[1]);

        // Get the minimum of nnids
        if (nnids[0] < nnids[1]) nnid = nnids[0];
        else nnid = nnids[1];

        // The other side of the first gate
        Pother.row(i) = get_gate_point(Pref.row(i),Q,O,A,B, interpolatedOthCoefsX.row(nnid), interpolatedOthCoefsY.row(nnid));


    // Give back the reference to the left or to the right
    if (data.left.size() < data.right.size()){
        this->traj.Pleft = Pref;
        this->traj.Pright = Pother; 
    } else {
        this->traj.Pleft = Pother;
        this->traj.Pright = Pref;
    }

}
}

// Auxiliar function: return the matrix called D in the documentation.
// This is only function of the dimension N
MatrixXd GRO::matrix_D(int N){
    
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
    
    // Inverse calculation and resizing
    MatrixXd Dinv = 2*Deqs.inverse();
    Map<MatrixXd, 0, Stride<Dynamic,4>> D(Dinv.data()+2, N, N, Stride<Dynamic,4>(16*N,4));
        
    return D;
}

// Auxiliar function: returns the coefficients of the splines that interpolates all N points
/* Explanation:
        coefs = [ d1 c1 b1 a1
                  d2 c2 b2 a2
                    .....
                  dn cn bn an ]
        where pi(t) = di*t^3 + ci*t^2 + bi*t + ai;
        with 0<t<1 */
MatrixXd GRO::coefs_splines(VectorXd x){

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

// Auxiliar function: get the point R of the gate. For more info see documentation of GROD
Vector2d GRO::get_gate_point(Vector2d  P, Vector2d  Q, Vector2d  O, Vector2d  A, Vector2d  B, Vector4d coefsX, Vector4d coefsY){
    
    Vector2d PQ, PO, v, AB, Rint;

    // The unit vector PQ
    PQ = Q - P;
    PQ.normalize();

    // The unit vector PO
    PO = O - P;
    PO.normalize();

    // The gate vector
    v = PQ + PO;

    // The vector AB
    AB = B - A;

    // Calculate the point R
    double R = ( P[1] - (v[1]/v[0])*P[0] - A[1] + (AB[1]/AB[0])*A[0] ) / ( AB[1]/AB[0] - v[1]/v[0] );
    double t = (R - A(0))/(B(0) - A(0));

    // Interpolate at the spline
    Rint(0) = polyval2(coefsX, t);
    Rint(1) = polyval2(coefsY, t);

    return Rint;
}


// Auxiliar function: given vector of points, reduces the size if there are opints too close each other
void GRO::reduce_points(){

    MatrixXd newPoints(traj.pointsSol.rows() , 2);
    MatrixXd newPleft(traj.Pleft.rows() , 2);
    MatrixXd newPright(traj.Pright.rows() , 2);

    int eliminated = 0;
    double dist;
    
    newPoints.row(0) = traj.pointsSol.row(0);
    newPleft.row(0) = traj.Pleft.row(0);
    newPright.row(0) = traj.Pright.row(0);
    

    for(int i = 0; i < traj.pointsSol.rows(); i++){

        // Calculate distance
        if (i+1 == traj.pointsSol.rows()) {
            dist = (newPoints.row(i - eliminated) - traj.pointsSol.row(0)).norm();
        } else {
            dist = (newPoints.row(i - eliminated) - traj.pointsSol.row(i+1)).norm();
        }
        
        // Eliminate cone that are too close
        if (dist < distBound) {
            eliminated++;
        } else if (i+1 < traj.pointsSol.rows()) {
            newPoints.row(i - eliminated + 1) = traj.pointsSol.row(i+1);
            newPleft.row(i - eliminated + 1) = traj.Pleft.row(i+1);
            newPright.row(i - eliminated + 1) = traj.Pright.row(i+1);
        }
    }

    // Cut eliminated spaces
    traj.pointsSol = newPoints.topRows(traj.pointsSol.rows() - eliminated);
    traj.Pleft = newPleft.topRows(traj.Pleft.rows() - eliminated);
    traj.Pright = newPright.topRows(traj.Pright.rows() - eliminated);

    // The total number of splines has been reduced
    traj.N -= eliminated;

}

// Auxiliar function: evaluates a 3rd order polynomial given its parameters and a vector x
// (same as Matlab)
VectorXd GRO::polyval(Vector4d coeffs, VectorXd t) {

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

// Same but evaluate polynomial to only one point, so t is a double not a vector
double GRO::polyval2(Vector4d coeffs, double t) {
    return coeffs(0)*pow(t,3) + coeffs(1)*pow(t,2) + coeffs(2)*t + coeffs(3);
}

// Auxiliar function: calculate the length of a 3rd order spline given its coefficients. Rectangular integration.
double GRO::integral_length(Vector4d coefsX, Vector4d coefsY){
    
    // This is the precision of integration
    double diffx = 0.0005;
    double dx, dy, cx, cy, bx, by, t, n, area, function;
    
    dx = coefsX(0);
    cx = coefsX(1);
    bx = coefsX(2);
    dy = coefsY(0);
    cy = coefsY(1);
    by = coefsY(2);
    n = 1.0/diffx;
    area = 0;

    // Numerical integration with the area of rectangles
    for(int j=0; j<n; j++){
        t = j*diffx;
        function = sqrt(pow(3*dx*pow(t, 2) + 2*cx*t + bx, 2) + pow(3*dy*pow(t, 2) + 2*cy*t + by, 2));
        area += function*diffx;
    }
    return area;
}

////////////////////////////////////////////////////////////////////////////////////////////
/////////////// From here, the functions of the planner    /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// Main function of the planner: creates the message and returns it.
as_msgs::ObjectiveArrayCurv GRO::plannerGRO_curv(const as_msgs::CarState::ConstPtr &data){

    // Create message
    as_msgs::ObjectiveArrayCurv msg;
    msg.header = data->header;

    // Planning
    MatrixXd plan = planning_curv(data);

    // Fill the message
    for(int i=0; i < steps; i++){
        as_msgs::ObjectiveCurv objective = as_msgs::ObjectiveCurv();

        objective.x = plan(i,0);
        objective.y = plan(i,1);
        objective.s = plan(i,2);
        objective.k = plan(i,3);
        objective.vx = plan(i,4);
        objective.L = plan(i,5);
        objective.R = plan(i,6);
        
        msg.objectives.push_back(objective);
    }

    msg.smax = traj.pointsTraj.rows() * spacing;

    return msg;
}

// Auxiliar function of the planner: creates the plan matrix of the MPC-Curv (steps by 2) dimensions.
MatrixXd GRO::planning_curv(const as_msgs::CarState::ConstPtr &data){
    
    // Actual position of the car
    Point state;
    state[0] = data->odom.position.x;
    state[1] = data->odom.position.y;

    // Search the nearest point of the trajectory
    int nnid = traj.trajTree.nnSearch(state)*2;

    // Initialize planning matrix
    MatrixXd plan(steps, 7); // [x, y, s, k, vx, L, R]

    // All steps
    for(int i=0; i < steps; i++){
        if(nnid >= traj.pointsTraj.rows()) nnid -= traj.pointsTraj.rows();

        plan(i,0) = traj.pointsTraj(nnid,0);
        plan(i,1) = traj.pointsTraj(nnid,1);
        plan(i,2) = nnid*spacing;
        plan(i,3) = 1/traj.radiCurv(nnid);
        plan(i,4) = traj.velocity(nnid);
        plan(i,5) = traj.freeL(nnid);
        plan(i,6) = traj.freeR(nnid);

        nnid++;
    }

    return plan;
}


// Visualization function that creates the message to plot the trajectory in RVIZ
nav_msgs::Path GRO::get_path(){

    // Init message:
    nav_msgs::Path pathMsg;
	geometry_msgs::PoseStamped pose;

    pathMsg.header.stamp    = ros::Time::now();
    pathMsg.header.frame_id = "global";

    for(unsigned int i = 0; i < traj.pointsTraj.rows(); i++ ){

        pose.pose.position.x = traj.pointsTraj(i, 0);
        pose.pose.position.y = traj.pointsTraj(i, 1);

        pathMsg.poses.push_back(pose);
        
    }

    return pathMsg;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// Reading // Saving file  ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// read_csv files (firstout argument is used to avoid first column of indexes / longvec is used for reserve method of vector)
as_msgs::Tracklimits GRO::read_csv(std::string filename){ 

    // Init message:
    as_msgs::Tracklimits midlineMsg;

    midlineMsg.stamp = ros::Time::now();

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("GRO::read_csv Could not open file");

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
        }else{
            midlineMsg.right.push_back(cone);
        }
    }

    // Avoid duplicated first and last tracklimit
    if(midlineMsg.left[0].position_global.x == midlineMsg.left[midlineMsg.left.size()-1].position_global.x
        && midlineMsg.left[0].position_global.y == midlineMsg.left[midlineMsg.left.size()-1].position_global.y) midlineMsg.left.pop_back();

    if(midlineMsg.right[0].position_global.x == midlineMsg.right[midlineMsg.right.size()-1].position_global.x
        && midlineMsg.right[0].position_global.y == midlineMsg.right[midlineMsg.right.size()-1].position_global.y) midlineMsg.right.pop_back();

    // Close file
    myFile.close();

    ROS_INFO_STREAM("TRACKLIMITS LOADED FROM: " << this->midlinePath);

    return midlineMsg;
}

void GRO::save_data(std::string filename){ 

    try{

        // midline_points.csv
        std::ofstream filteredPoints;
        filteredPoints.open(filename + "midline_points.csv", std::ofstream::out | std::ofstream::trunc);

        for(int i=0; i<traj.pointsTraj.rows(); i++){
            filteredPoints << traj.pointsTraj(i,0) << ",";
            filteredPoints << traj.pointsTraj(i,1) << "\n";
        }

        // Remeber to close the file!
        filteredPoints.close();

        // curvature.csv
        std::ofstream curvature;
        curvature.open(filename+"curvature.csv", std::ofstream::out | std::ofstream::trunc);

        for(int i = 0; i < traj.radiCurv.size(); i++){
            curvature << 1/traj.radiCurv(i) << "\n";
        }

        curvature.close();

        // freeSpace.csv
        std::ofstream freeSpace;
        freeSpace.open(filename+"freeSpace.csv", std::ofstream::out | std::ofstream::trunc);

        for(int i=0; i<traj.freeR.size(); i++){
            freeSpace << traj.freeR(i) << ",";
            freeSpace << traj.freeL(i) << "\n";
        }

        freeSpace.close();

        ROS_INFO_STREAM("MIDLINE TRACK data SAVED AT: " << this->savePath);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "GRO::save_csv Exception was thrown: " << e.what() );
    }


}

////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// Dynamic Reconfigure Parameters ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

bool GRO::reconfigure (trajectory_optimizer::dynamicConfig& config){

    try{
        this->securityFactor = config.securityFactor;
        this->separation = config.separation;
        this->dt = config.dt;

        this->axmaxAccel = config.axmaxAccel;
        this->axmaxDecel = config.axmaxDecel;
        this->aymax = config.aymax;

        this->savePath = config.savePath;
        this->midlinePath = config.midlinePath;

        // Recalculate velocity profile
        if(this->isRun) {
            velocity_profile();
        }
        
    } catch (std::exception& e) {
        ROS_ERROR("GRO: Reconfigure Died");
        ROS_ERROR_STREAM( e.what() );
        return 1;
    }

    return 0;
}
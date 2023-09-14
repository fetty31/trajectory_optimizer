#include "../include/optimizer.hh"
#include "../include/SGSmooth.hpp"

using namespace std;
using namespace casadi;
using namespace Eigen;

// Constructor
optimizer::optimizer(){}

// Initialization of optimizer
void optimizer::init(){

  auto start_time = std::chrono::system_clock::now();

  // Run principal functions
  parameters();
  boundaries();
  Solve();
  if(debugging) debug();

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  ROS_WARN( "TROsolver ELAPSED TIME: %f s", elapsed.count());

}

// isRunning: returns true if the solver is currently working
bool optimizer::isRunning(){
    return this->isRun;
}

// ExitFlag: return true if the NLOP succeded --> exit flag == Solve_Succeded
bool optimizer::ExitFlag(){
    bool flag;
    if(nlop.flag == "Solve_Succeded"){
        flag = true;
    } else {
        flag = false;
    }
    return flag;
}

// boundaries: Set boundaries for state, control variables and equality & inequality constraints
void optimizer::boundaries(){

  vector<double> X_MIN;
  vector<double> X_MAX;
  vector<double> X0;
  vector<double> U_MIN;
  vector<double> U_MAX;
  vector<double> U0;

  // Reserve memory
  X_MIN.reserve(this->x_min.size()*(this->N+1));
  X_MAX.reserve(this->x_max.size()*(this->N+1));
  X0.reserve(this->x0.size()*(this->N+1));
  U_MIN.reserve(this->u_min.size()*this->N);
  U_MAX.reserve(this->u_max.size()*this->N);
  U0.reserve(this->u0.size()*this->N);

  // Bounds of constraints
  vector<double> lbg_next(n_states*N, this->lower_continuity);  // lower boundaries continuity constraint
  vector<double> lbg_track(2*(N+1), this->lower_track);         // lower boundaries track constraint
  vector<double> lbg_elipse(2*(N+1), this->lower_ellipse);      // lower boundaries ellipse constraint

  vector<double> ubg_next(n_states*N, this->upper_continuity);  // upper boundaries continuity constraint
  vector<double> ubg_track(2*(N+1), this->upper_track);         // upper boundaries track constraint
  vector<double> ubg_elipse(2*(N+1), this->upper_ellipse);      // upper boundaries ellipse constraint

  // Bounds of initial==final constraint
  for(unsigned int i = 0; i < this->lower_final.size(); i++){
    lbg_elipse.push_back(this->lower_final[i]);
    ubg_elipse.push_back(this->upper_final[i]);
  }

  nlop.lbg_next = lbg_next;
  nlop.lbg_track = lbg_track;
  nlop.lbg_elipse = lbg_elipse;
  nlop.ubg_next = ubg_next;
  nlop.ubg_track = ubg_track;
  nlop.ubg_elipse = ubg_elipse;

  for(int k=0; k<N+1; k++){

    X_MIN.insert(X_MIN.end(), this->x_min.begin(), this->x_min.end());
    X_MAX.insert(X_MAX.end(), this->x_max.begin(), this->x_max.end());

    X0.insert(X0.end(), this->x0.begin(), this->x0.end());

    if(k<N){
      
      U_MIN.insert(U_MIN.end(), this->u_min.begin(), this->u_min.end());
      U_MAX.insert(U_MAX.end(), this->u_max.begin(), this->u_max.end());

      U0.insert(U0.end(), this->u0.begin(), this->u0.end());
    }
  }

  nlop.lbx = vconcat(X_MIN,U_MIN);
  nlop.ubx = vconcat(X_MAX,U_MAX);
  nlop.x0 = vconcat(X0,U0);

}

// parameters: Set parameters for the NLOP --> parameters + curvature middle trajectory
void optimizer::parameters(){

  vector<double> param = {this->dRd, 
                        this->dRa, 
                        this->m, 
                        this->I, 
                        this->Lf, 
                        this->Lr, 
                        this->Dr, 
                        this->Df, 
                        this->Cr, 
                        this->Cf, 
                        this->Br, 
                        this->Bf, 
                        this->u_r, 
                        this->gravity, 
                        this->Cd, 
                        this->rho, 
                        this->Ar, 
                        this->q_slip, 
                        this->p_long, 
                        this->q_n, 
                        this->q_mu, 
                        this->lambda, 
                        this->q_s};

  MatrixXd curvature = read_csv(curvaturePath,true,false);

  if(horizonLength != 0){
    N = horizonLength;
    curvatureStride = curvature.rows()/N;
  }else{
    N = curvature.rows()/curvatureStride;
  }

  T = this->dt*curvatureStride; // Discretization of TRO depends on the discretization of the track's curvature

  Map<VectorXd,0,InnerStride<> > curvatureN(curvature.data(), N, 1, InnerStride<>(curvatureStride));

  // Filter with savitzky-golay filter (smoother curvature)
  std::vector<double> softCurvature(curvatureN.size()),rawCurvature(curvatureN.size());
  for(unsigned int i = 0; i < curvatureN.size(); i++) rawCurvature[i] = curvatureN(i); 
  softCurvature = sg_smooth(rawCurvature, 19, 5);

  save_csv(savePath,"filtered_curvature.csv",softCurvature); // Save filtered curvature for debug 

  param.reserve(x0.size()+param.size()+softCurvature.size()); // reserve memory

  param.insert(param.end(), softCurvature.begin(), softCurvature.end()); // insert midline path's curvature

  nlop.p = param;
  nlop.N = N;
  Npar = param.size();

}

// Solve: Principal function --> set constraints / objective function / solve the NLOP
void optimizer::Solve(){

  // Running flag set to true
  isRun = true;

  // Optimization variables
  SX X = SX::sym("X", n_states*(N+1));
  SX U = SX::sym("U", n_controls*N);
  SX P = SX::sym("P", Npar);

  SX dRd = P(0);
  SX dRa = P(1);
  SX Lf = P(4);
  SX Lr = P(5);
  SX q_slip = P(17);
  SX q_n = P(19);
  SX q_s = P(22);
  SX q_mu = P(20);

  SX obj = 0; // Objective function
  vector<SX> g; // Constraints vector

  int idx = 0; // index for X variables (states) 
  int idu = 0; // index for U variables (controls) 
  for(int e = 0; e < N; e++){
      SX st = X(Slice(idx,idx+n_states));
      SX k = P(23+e);
      SX steering = st(0) + U(idu);

      // Progress rate
      SX sdot = (st(4)*cos(st(3)) - st(5)*sin(st(3)))/(1-st(2)*k);

      // Slip difference
      SX beta_dyn = atan(st(5)/st(4));
      SX beta_kin = atan(steering*Lr/(Lr+Lf));
      SX diff_beta = beta_dyn - beta_kin;

      // Objective function
      obj += -q_s*sdot + dRd*pow(U(idu),2) + dRa*pow(U(idu+1),2) + q_slip*pow(diff_beta,2) + q_mu*pow(st(3),2) + q_n*pow(st(2),2);

      SX st_next = X(Slice(idx+n_states,idx+2*n_states)); // next state
      SX con_now = U(Slice(idu,idu+n_controls)) + st(Slice(0,2)); // current stage global controls (steering,throttle)
      SX st_now = st(Slice(2,st.rows())); // current stage states (n,mu,vx,vy,w)

      vector<SX> f_value = continuous_dynamics(st_now,con_now,P,k); // ODE with next states prediction

      SX states = SX::sym("states", f_value.size());
      for(int row = 0; row < f_value.size(); row++){
        states(row) = st_now(row) + T*f_value.at(row); // euler
      }

      SX st_next_euler = SX::vertcat({con_now,states}); //next state calculated 

      // Add continuity constraints
      g.push_back( st_next - st_next_euler );

      // Next indexes 
      idx+=n_states;
      idu+=n_controls;
  }

  // Set constraints --> track constraints / forces ellipse constraint
    // Track constraint
  for(int i=0;i<N+1;i++){

    SX n = X(2+i*7);
    SX mu = X(3+i*7);

    g.push_back(n+longue/2*sin(abs(mu))+width/2*cos(mu)); // track constraints
    g.push_back(-n+longue/2*sin(abs(mu))+width/2*cos(mu));

  }

  // Ellipse of forces constraint
  for(int i=0;i<N+1;i++){

    SX alpha_R = atan((X(5+i*7)-nlop.p[5]*X(6+i*7))/X(4+i*7));               // atan((vy-Lr*w)/(vx))
    SX alpha_F = atan((X(5+i*7)+nlop.p[4]*X(6+i*7))/X(4+i*7)) - X(0+i*7);    // atan((vy+Lf*w)/(vx)) - delta

    SX Fr = nlop.p[6]*sin(nlop.p[8]*atan(nlop.p[10]*alpha_R));                                                            // Fr = Dr*sin(Cr*atan(Br*alpha_R))
    SX Ff = nlop.p[7]*sin(nlop.p[9]*atan(nlop.p[11]*alpha_F));                                                            // Ff = Df*sin(Cf*atan(Bf*alpha_F))
    SX Fx = nlop.p[2]*X(1+i*7) - nlop.p[2]*nlop.p[12]*nlop.p[13] - 0.5*nlop.p[14]*nlop.p[15]*nlop.p[16]*pow(X(4+i*7),2);  // Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2

    // g.push_back( pow(nlop.p[18]*Fx,2) + pow(Fr,2) - pow(nlop.p[21]*nlop.p[6],2)/nlop.p[2] );   // (p_long*Fx)² + Fr² <= (lambda*Dr)²/m
    // g.push_back( pow(nlop.p[18]*Fx,2) + pow(Ff,2) - pow(nlop.p[21]*nlop.p[7],2)/nlop.p[2] );   // (p_long*Fx)² + Ff² <= (lambda*Df)²/m

    g.push_back( pow(Fx/(this->x_max[1]*this->m),2) + pow(Fr/(fabs(this->x_min[1])*this->m),2) - this->lambda ); // (Fx/(Ax_max*m))² + (Fr/(Ay_max*m))² <= lambda
    g.push_back( pow(Fx/(this->x_max[1]*this->m),2) + pow(Ff/(fabs(this->x_min[1])*this->m),2) - this->lambda ); // (Fx/(Ax_max*m))² + (Ff/(Ay_max*m))² <= lambda

  }

  // Constraint initial==final stage
  g.push_back( X(Slice(2,7))-X(Slice(X.rows()-5,X.rows())) );


  SXDict nlp = {{"x", SX::vertcat({X,U})},
                {"f", obj},
                {"g", SX::vertcat(g)},
                {"p", P}
                };

  // Create an NLOP solver instance
  Function solver = nlpsol("TROsolver", "ipopt", nlp, solverOptions);

  // Bounds and initial guess
  std::map<std::string, DM> arg, sol;
  arg["lbx"] = nlop.lbx;
  arg["ubx"] = nlop.ubx;
  arg["lbg"] = vconcat(vconcat(nlop.lbg_next,nlop.lbg_track),nlop.lbg_elipse);
  arg["ubg"] = vconcat(vconcat(nlop.ubg_next,nlop.ubg_track),nlop.ubg_elipse);
  arg["x0"] = nlop.x0;
  arg["p"] = nlop.p;

  // Solve the NLOP
  sol = solver(arg);

  casadi::Dict stats;
  stats = solver.stats();

  nlop.flag = stats["return_status"].get_str();
  nlop.solution = vector<double>(sol.at("x"));

  save_csv(savePath,"x_opt.csv",nlop.solution); // Save results!
  save_info(savePath,"problem.csv");

  // Exit Flag:
  cout << "EXIT FLAG = " << nlop.flag << endl;

}

// debug: prints debugging info
void optimizer::debug(){

  cout << "----------------------------------------------------------------------------------------\n";
  // Parameters:
  cout << "PARAMETERS:" << endl;
  printVec(nlop.p,23);

  // Initial Guess:
  cout << "INITIAL GUESS:" << endl;
  printVec(x0);
  cout << "\n";

  // Horizon Length:
  ROS_INFO("Horizon Length: %i", N);

}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

// ODE equations
vector<SX> optimizer::continuous_dynamics(SX st, SX con, SX P, SX k){

  vector<SX> f_values;

  SX alpha_R = atan((st(3)-P(5)*st(4))/st(2));                                  // alphaR = atan((vy-Lr*w)/(vx))
  SX alpha_F = atan((st(3)+P(4)*st(4))/st(2)) - con(0);                         // alphaL = atan((vy+Lf*w)/(vx)) - delta

  SX Fr = P(6)*sin(P(8)*atan(P(10)*alpha_R));                                   // Fr = Dr*sin(Cr*atan(Br*alpha_R))
  SX Ff = P(7)*sin(P(9)*atan(P(11)*alpha_F));                                   // Ff = Df*sin(Cf*atan(Bf*alpha_F))
  SX Fx = P(2)*con(1) - P(2)*P(12)*P(13) - 0.5*P(14)*P(15)*P(16)*pow(st(2),2);  // Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2

  SX sdot = (st(2)*cos(st(1)) - st(3)*sin(st(1)))/(1-st(0)*k);                  // sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k)

  SX ndot = (st(2)*sin(st(1)) + st(3)*cos(st(1)))/sdot;                         // ndot   =  (vx*sin(mu) + vy*cos(mu))/sdot
  SX mudot = st(4)/sdot - k;                                                    // mudot  =  w/sdot - k
  SX vxdot = 1/P(2)*(Fx - Ff*sin(con(0)) + P(2)*st(3)*st(4))/sdot;              // vxdot  =  (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot
  SX vydot = 1/P(2)*(Fr + Ff*cos(con(0)) - P(2)*st(2)*st(4))/sdot;              // vydot  =  (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot
  SX wdot = 1/P(3)*(Ff*P(4)*cos(con(0)) - Fr*P(5))/sdot;                        // wdot   =  (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot

  f_values.push_back(ndot);
  f_values.push_back(mudot);
  f_values.push_back(vxdot);
  f_values.push_back(vydot);
  f_values.push_back(wdot);

  return f_values;

}

// Concatenate two vectors
vector<double> optimizer::vconcat(const vector<double>& x, const vector<double>& y){
  vector<double> v(x.size() + y.size(),0.0);
	move(x.begin(), x.end(), v.begin());
	move(y.begin(), y.end(), v.begin() + x.size());
  return v;
}

// Print vector elements 
void optimizer::printVec(std::vector<double> &input, int firstElements){
    if(firstElements!=0){
      for (auto it = input.begin(); it != input.end()-input.size()+firstElements; it++) {
        std::cout << *it << "\n";
      }
    }else{
        for (auto it = input.begin(); it != input.end(); it++) {
        std::cout << *it << "\n";
      }
    }   
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// Reading // Saving file  ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// read_csv files (firstout argument is used to avoid first column (indexes) / longvec is used for reserve method of vector)
MatrixXd optimizer::read_csv(std::string& filename, bool longvec, bool firstout){ 

    MatrixXd FileContent; // Eigen Matrix to store csv data
    std::vector<std::vector<double>> result; // Vector of vectors to store each line content
    if(longvec) result.reserve(4000);

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("OPTIMIZER::read_csv Could not open file");

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

// Save into csv some vector data
void optimizer::save_csv(std::string& filePath, std::string name, std::vector<double>& data){

    try{
        
        // Write csv
        std::ofstream file;
        file.open(filePath + name, std::ofstream::out | std::ofstream::trunc);

        for(auto it=data.begin(); it != data.end(); it++){
            file << *it << "\n";
        }

        // Remeber to close the file!
        file.close();

        ROS_INFO_STREAM("X_OPT data SAVED AT: " << this->savePath+name);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "OPTIMIZER::save_csv Exception was thrown: " << e.what() );
    }


}

void optimizer::save_info(std::string& filePath, std::string name){

  try{

    // Write csv
    std::ofstream file;
    file.open(filePath+name, std::ofstream::out | std::ofstream::trunc);

    // Save horizon length
    file << nlop.N << "\n";
    // Save discretization used (T)
    file << T << "\n";
    // Save number of parameters
    file << Npar << "\n";

    file.close();

  }
  catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "OPTIMIZER::save_info Exception was thrown: " << e.what() );
    }
}

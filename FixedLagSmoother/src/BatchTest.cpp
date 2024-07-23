// FixedLagSmoother example for ISAM2_SmartFactorStereo_IMU.cpp
// using CombinedImuFactor, and SmartProjectionRigFactor

/**
 * @file ISAM2_SmartFactorStereo_IMU.cpp
 * @brief test of iSAM2 with smart stereo factors and IMU preintegration,
 * originally used to debug valgrind invalid reads with Eigen
 * @author Nghia Ho
 *
 * Setup is a stationary stereo camera with an IMU attached.
 * The data file is at examples/Data/ISAM2_SmartFactorStereo_IMU.txt
 * It contains 5 frames of stereo matches and IMU data.
 */
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Factor.h>
// #include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/types.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <map>


using namespace std;
using namespace gtsam;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;


// extrinsic

int main(int argc, char* argv[]) {
  gtsam::Rot3 R_b_cam0(0.0051989, -0.00475195 , 0.99997519, 
                   -0.99998009 , -0.00360067 , 0.00518181, 
                   0.00357595 , -0.99998223 , -0.00477058);

  gtsam::Point3 t_b_cam0(0.01782353, 0.02405353, 0.00652621);
  gtsam::Pose3 T_b_cam0(R_b_cam0, t_b_cam0);

  gtsam::Rot3 R_cam1_cam0(0.9999030819138741, 0.0010711343462142993 , 0.013880902361026801, 
                        -0.0011234892556085138 , 0.9999922832259254 , 0.003764473469264149,
                        -0.013876762988410936 , -0.003779703668360986 , 0.99989656929562);

  gtsam::Point3 t_cam1_cam0(-0.1220026692068293, 0.00011691065038523982, 0.0023967307199285926);
  gtsam::Pose3 T_cam1_cam0(R_cam1_cam0, t_cam1_cam0);


  typedef gtsam::SmartProjectionRigFactor<gtsam::PinholePose<gtsam::Cal3_S2>> RigFactor;
  std::map<int, RigFactor::shared_ptr> smartFactor;
  boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2>>> cameraRig;

  vector<gtsam::Cal3_S2::shared_ptr> K;
  gtsam::Cal3_S2::shared_ptr K0;
  gtsam::Cal3_S2::shared_ptr K1;
  
  // new
  

  ifstream in_frontend("/home/jason/DPGO/FixedLagSmoother/frontendData_VO.txt");
  ifstream in_initial("/home/jason/DPGO/FixedLagSmoother/initiasl_values_processed_VO.txt");

    if (!in_frontend) {
    cerr << "error opening: in_frontend file " << "\n";
    return 1;
  }

  
  double totalTime = 0;
  vector<double> T;
  double lag = 3.0;
  int camNum = 2;
  
// 
          LevenbergMarquardtParams parameters_lm;
    parameters_lm.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_QR;
  gtsam::BatchFixedLagSmoother smootherBatch(lag);
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1; //
  parameters.relinearizeSkip = 1; // 
  gtsam::IncrementalFixedLagSmoother smootherISAM2(lag, parameters);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;
  gtsam::FixedLagSmoother::KeyTimestampMap PoseTimestamps;
  gtsam::FixedLagSmoother::Result result;
  ISAM2 isam(parameters);
    gtsam::NonlinearFactorGraph priorFactorGraph;

  ofstream saveTxt("/home/jason/DPGO/FixedLagSmoother/VO_traj.txt");


// cam0 : 268.0886807833101, 266.2531972184412, 339.57968813983365, 175.06123592987922
  double fx0 = 268.0886807833101;
  double fy0 = 266.2531972184412;
  double cx0 = 339.57968813983365;
  double cy0 = 175.06123592987922;
// cam1 : 268.89135202386177, 266.8960254737521, 335.2953727102823, 173.96166934284088
  double fx1 = 268.89135202386177;
  double fy1 = 266.8960254737521;
  double cx1 = 335.2953727102823;
  double cy1 = 173.96166934284088;
  // Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline));

// K.push_back(gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx0, fy0, 0, cx0, cy0)));
// K.push_back(gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx1, fy1, 0, cx1, cy1)));
  K0 = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx0, fy0, 0, cx0, cy0));
  K1 = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx1, fy1, 0, cx1, cy1));
  cameraRig = static_cast<boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose< gtsam::Cal3_S2 > > > >(
              new gtsam::CameraSet<gtsam::PinholePose< gtsam::Cal3_S2 > >());
  

  gtsam::PinholePose<gtsam::Cal3_S2> cam0(T_b_cam0, K0);
  cameraRig->push_back(cam0);
  gtsam::PinholePose<gtsam::Cal3_S2> cam1(T_b_cam0 * T_cam1_cam0.inverse(), K1);
  cameraRig->push_back(cam1);
  cameraRig->print("cameraRig");


  auto priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());

// TODO what is the Prior vaule from frontend ?


  double x1, x2, x3, x4, x5, x6;

  int frame_initial;
  char type_initial;
// ************Prior**************//




  int lastFrame = 6;
  int frame;
  char type_frontend;
  int frameCount = 0;
  

  while (true) {
    char line_frontend[1024];
    in_frontend.getline(line_frontend, sizeof(line_frontend));
    stringstream ss_frontend(line_frontend);
    

    
    ss_frontend >> type_frontend;
    ss_frontend >> frame;
    
    if ( frame != lastFrame || in_frontend.eof()) {
    frameCount++;
    char line_initial[1024];
    in_initial.getline(line_initial, sizeof(line_initial));
    stringstream ss_initial(line_initial);
      
      ss_initial >> type_initial >> frame_initial;
        ss_initial >> x1 >> x2 >> x3 >> x4 >> x5 >> x6; // x4:roll, x5:pitch, x6:yaw
          initialEstimate.insert(X(frame_initial), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
    
    if (frameCount == 0) {
        priorFactorGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(frame_initial), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1 - 0.001, x2 - 0.001, x3 - 0.001)), priorPoseNoise);
    }
    


    if (frameCount == 49) {
        frameCount = 0;   

        graph.push_back(priorFactorGraph.begin(), priorFactorGraph.end());

        gtsam::LevenbergMarquardtOptimizer LMoptimizer(graph, initialEstimate);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        LMoptimizer.optimize();
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        std::chrono::duration<double> time_used = std::chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "solve time cost = " << time_used.count() << " seconds." << endl;
        totalTime = totalTime + time_used.count();
        T.push_back(time_used.count());
        totalTime = 0;
        priorFactorGraph.resize(0);
      graph.resize(0);
      initialEstimate.clear();
    }


      if (in_frontend.eof()) {
        break;
      }
    }

  if (type_frontend == 'l') {  // Process stereo measurement
      std::string temp;
      int landmark;
      int CamID;
      double measurement_u, measurement_v;
      ss_frontend >> temp >> landmark;
      ss_frontend >> temp >> CamID;
      ss_frontend >> temp >> measurement_u >> measurement_v;

      if (smartFactor.count(landmark) == 0) {
        auto gaussian = gtsam::noiseModel::Isotropic::Sigma(2, 50.0);

        SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
        
        smartFactor[landmark] = RigFactor::shared_ptr(new RigFactor(gaussian, cameraRig, params));

        graph.push_back(smartFactor[landmark]);
      }

      smartFactor[landmark]->add(gtsam::Point2(measurement_u, measurement_v), gtsam::Symbol('x', frame), CamID);

      // smartFactor[landmark]->add(StereoPoint2(xl, xr, y), X(frame), K);
    } else {
      throw runtime_error("unexpected data type: " + string(1, type_frontend));
    }

    lastFrame = frame;
  }



    double totalTime_final = 0;
    for (double value : T) {
    totalTime_final  = totalTime_final + value;
    }

    std::cout << "average time : "  << totalTime_final / T.size() << std::endl;

  return 0;
}


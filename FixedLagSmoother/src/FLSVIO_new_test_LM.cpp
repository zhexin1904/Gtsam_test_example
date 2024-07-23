#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/VariableIndex.h>
// #include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <map>
#include <chrono>

using namespace std;
using namespace gtsam;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
typedef gtsam::SmartProjectionRigFactor<gtsam::PinholePose<gtsam::Cal3_S2>> RigFactor;
// from KimeraVIO
using Slot = long int;
using LandmarkId = long int;
typedef std::map<LandmarkId, vector<double>> lmk_nKFs;

using SmartFactorMap =
    gtsam::FastMap<LandmarkId, RigFactor::shared_ptr>;


using LandmarkIdSmartFactorMap = std::map<LandmarkId, RigFactor::shared_ptr>;

using CameraRig = boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2>>>;

gtsam::NonlinearFactorGraph nfg;

auto total_time = 0.0;


class FeatureTrack {

 public:
  //! Observation: {FrameId, CamID, Px-Measurement-u, Px-Measurement-v}
  std::vector<std::tuple<int, int, double, double>> obs_;

  std::vector<int> kfs_in_lmkID;
    // std::set<int>  kfs_in_lmkID;

  FeatureTrack(int frame_id, int cam_id, const double px_u, const double px_v) {
    obs_.push_back(std::make_tuple(frame_id, cam_id, px_u, px_v));
  }
  // Method to add a new observation
  void addObservation(int frame_id, int cam_id, double px_u, double px_v) {
    obs_.push_back(std::make_tuple(frame_id, cam_id, px_u, px_v));
  }
  
  void print() const {
    std::cout << "Feature track with frameID: " << std::endl;
    for (size_t i = 0u; i < obs_.size(); i++) {
      std::cout << " " << std::get<0>(obs_[i]) << " ";
    }
    std::cout << std::endl;
  }
};

using FeatureTracks = std::map<LandmarkId, FeatureTrack>;
using LmkID_in_Key = std::map<int, std::vector<LandmarkId>>;
using Key_in_LmkID = std::map<int, std::vector<int>>;


void ReadingLmk(FeatureTracks& fts_, const std::string dataPath_) {
    ifstream in_frontend(dataPath_);

    int frame;
    string line;
    LandmarkId landmark;
    
    while (in_frontend >> line) {
        if(line == "l")
            {
                in_frontend >> landmark;
                continue;
            }
        else if (line == "e") {
                int CamID;
                double measurement_u, measurement_v;
                in_frontend >> frame >> CamID >> measurement_u >> measurement_v;
                // Don't add the negative measurements.
                if (measurement_u < 0.0 || measurement_v < 0.0) {
                    continue;
                }
                // a new landmark
                if (fts_.count(landmark) == 0) {
                    FeatureTrack ft_tmp_new(frame, CamID, measurement_u, measurement_v);
                    ft_tmp_new.kfs_in_lmkID.push_back(frame);
                    fts_.insert(std::make_pair(landmark, ft_tmp_new));
                }
                // adding measurement
                else {
                    auto ft_tmp_old_it = fts_.find(landmark);
                    if (ft_tmp_old_it != fts_.end()) {
                        ft_tmp_old_it->second.addObservation(frame, CamID, measurement_u, measurement_v);
                        auto kfs_in_lmkID_it = std::find(ft_tmp_old_it->second.kfs_in_lmkID.begin(), ft_tmp_old_it->second.kfs_in_lmkID.end(), frame);
                        if (kfs_in_lmkID_it == ft_tmp_old_it->second.kfs_in_lmkID.end()) {
                            ft_tmp_old_it->second.kfs_in_lmkID.push_back(frame);
                        }
                    }
                }
                continue;
            }
        }
}


// from "<lmk, key>" (actually FeatureTracks, not exactlly the same), to get <key, lmk>
LmkID_in_Key invertMap(const FeatureTracks& fts_) {
    std::map<int, std::vector<LandmarkId>> key_to_lmk;

    for (const auto& lmk_pair : fts_) {
        int lmk = lmk_pair.first;
        const std::vector<int>& keys = lmk_pair.second.kfs_in_lmkID;

        for (int key : keys) {
            key_to_lmk[key].push_back(lmk);
        }
    }

    return key_to_lmk;
}


void addLandmarkToGraph(int kfID, const LmkID_in_Key& LmkID_in_Key_,
                       FeatureTracks& feature_tracks_,
                       SmartFactorMap& smartFactorMap_,
                       const CameraRig& cameraRig_,
//                        const SlidingWindow& SW_,
                       gtsam::BatchFixedLagSmoother& smootherBatch) {

    auto result = smootherBatch.calculateEstimate();
//    get the current keys inside the smoother from result and store in a variable
    gtsam::KeyVector keys = result.keys();
    std::vector<int> window;
    for (const auto& key : keys) {
        auto sym = gtsam::Symbol(key);
        window.push_back(sym.index());
        cout << "Key: " << sym.index() << endl;
    }
    window.push_back(kfID);
    int n_new_landmarks = 0;
    int n_updated_landmarks = 0;
    int kfID_count = 0;
    SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(2, 10.0);
    auto currentLmk_id_it = LmkID_in_Key_.find(kfID);
    std::vector<LandmarkId> currentLmk_id_;
    if (currentLmk_id_it != LmkID_in_Key_.end()) {
         currentLmk_id_ = currentLmk_id_it->second;
    }

  for (const LandmarkId& lmk_id : currentLmk_id_) {
    FeatureTrack& ft = feature_tracks_.at(lmk_id);
      auto kfID_it = std::find(ft.kfs_in_lmkID.begin(), ft.kfs_in_lmkID.end(), kfID);
      if (kfID_it != ft.kfs_in_lmkID.end()) {
          kfID_count = std::distance(ft.kfs_in_lmkID.begin(), kfID_it);
      }
//      minimum 2 kfs condition
      if ((kfID_count + 1) < 2) {
          continue;
      }
      // current landmark has not been observed before, create smart factor for it
    if (smartFactorMap_.count(lmk_id) == 0) {
        // kfID_count is actually the index of current key in the vector : ft.kfs_in_lmkID
        // if all the  kfID of current landmark is outside the window, we will discard this observation
        // but just need to verify the key before current key
        if (std::find(window.begin(), window.end(), ft.kfs_in_lmkID[kfID_count - 1]) == window.end()) {
            continue;
        }

        RigFactor::shared_ptr newFactor(new RigFactor(gaussian, cameraRig_, params));
        for (const std::tuple<int, int, double, double> &obs: ft.obs_) {
            // make sure we won't pick up "future" measurement
            // we get obs in order, continuously ,so use "break" later
            const int frame = std::get<0>(obs);

            if (std::find(window.begin(), window.end(), frame) == window.end()) {
                break;
            } else {
                const int CamID = std::get<1>(obs);
                const double measurement_u = std::get<2>(obs);
                const double measurement_v = std::get<3>(obs);
                newFactor->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
            }
            
            smartFactorMap_.insert(std::make_pair(lmk_id, newFactor));
            nfg.push_back(smartFactorMap_[lmk_id]);
            ++n_new_landmarks;
        }
    }
    else {
            bool need_new_smart_factor = false;
            
            // Check if any keyframe is outside the sliding window
            for (int i = 0; i <= kfID_count; ++i) {

                if (std::find(window.begin(), window.end(), ft.kfs_in_lmkID[i]) == window.end()){
                    need_new_smart_factor = true;
                    break;
                }
            }

            if (need_new_smart_factor) {
                // If a new smart factor needs to be created
                smartFactorMap_.erase(lmk_id);
                smartFactorMap_[lmk_id] = RigFactor::shared_ptr(new RigFactor(gaussian, cameraRig_, params));
                for (const std::tuple<int, int, double, double> &obs : ft.obs_) {

                    if (std::find(window.begin(), window.end(), std::get<0>(obs)) == window.end()) {
                        continue;
                    }
                    const int frame = std::get<0>(obs);
                    const int CamID = std::get<1>(obs);
                    const double measurement_u = std::get<2>(obs);
                    const double measurement_v = std::get<3>(obs);
                    smartFactorMap_[lmk_id]->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
                }
                nfg.push_back(smartFactorMap_[lmk_id]);
            } else {
                // Directly add observations to the existing smart factor
                for (const std::tuple<int, int, double, double> &obs : ft.obs_) {
                    if (kfID != std::get<0>(obs)) {
                        continue;
                    }
                    const int frame = std::get<0>(obs);
                    const int CamID = std::get<1>(obs);
                    const double measurement_u = std::get<2>(obs);
                    const double measurement_v = std::get<3>(obs);
                    smartFactorMap_[lmk_id]->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
                }
                nfg.push_back(smartFactorMap_[lmk_id]);
            }

            ++n_updated_landmarks;
    }

  }

    std::cout << "Added " << n_new_landmarks << " new landmarks" << std::endl
           << "Updated " << n_updated_landmarks << " landmarks in graph" << std::endl;


}


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


    LandmarkIdSmartFactorMap newSmartFactor_;
    std::map<int, RigFactor::shared_ptr> smartFactor;
    boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2>>> cameraRig;

    vector<gtsam::Cal3_S2::shared_ptr> K;
    gtsam::Cal3_S2::shared_ptr K0;
    gtsam::Cal3_S2::shared_ptr K1;
  
    gtsam::NonlinearFactorGraph new_graph;
    gtsam::NonlinearFactorGraph priorFactorGraph;

    std::string in_frontend("/home/jason/DPGO/FixedLagSmoother/newLogs_Lmk.txt");
    std::string in_initials("/home/jason/DPGO/FixedLagSmoother/newLogs_initials.txt");


    vector<double> T;
    double lag = 10.0;
//    SlidingWindow SW(lag + 1);

    LevenbergMarquardtParams parameters_lm;

    parameters_lm.lambdaInitial = 1e-2;
    parameters_lm.maxIterations = 15;
    parameters_lm.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_QR;
    parameters_lm.setVerbosityLM("SUMMARY");
    parameters_lm.setOrderingType("COLAMD");
    gtsam::BatchFixedLagSmoother smootherBatch(lag, parameters_lm);
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::FixedLagSmoother::KeyTimestampMap PoseTimestamps;
    gtsam::FixedLagSmoother::Result result;


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
        (Vector(6) << gtsam::Vector3::Constant(0.01), gtsam::Vector3::Constant(0.01)).finished());



  double x1, x2, x3, x4, x5, x6;

    int frame, temp;
    ifstream in_initial(in_initials);
    string initial_line, temp_initial;
    LandmarkId landmark;
    bool initialized = false;
    gtsam::MatrixRowMajor m(4, 4);
    int count=1;
    SmartFactorMap globalSmartFactor;
    
    FeatureTracks fts;
    ReadingLmk(fts, in_frontend);
    LmkID_in_Key lmks_in_keys;
    lmks_in_keys = invertMap(fts) ;
//    print the size of feature_tracks_
    cout << "The size of feature_tracks_ is: " << fts.size() << endl;

    while (in_initial >> initial_line){
        if (initial_line == "x") {
            in_initial >> frame >> temp;
            for (int i = 0; i < 16; i++) {
                in_initial >> m.data()[i];
            }
            gtsam::Pose3 pose(m);

            gtsam::Pose3 body_pose = pose.compose(T_b_cam0.inverse());

    //           get the roll pitch and yaw from body_pose and x, y, z from body_pose
            x1 = body_pose.x();
            x2 = body_pose.y();
            x3 = body_pose.z();
            x4 = body_pose.rotation().roll();
            x5 = body_pose.rotation().pitch();
            x6 = body_pose.rotation().yaw();


            if (!initialized) {
                priorFactorGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(frame), Pose3(Rot3::Ypr(x6, x5, x4),
                                                                                                Point3(x1 - 0.001,
                                                                                                        x2 - 0.001,
                                                                                                        x3 - 0.001)),
                                                                                priorPoseNoise);
                initialEstimate.insert(X(frame), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
                PoseTimestamps[X(frame)] = count;

                count++;
                initialized = true;
                nfg.push_back(priorFactorGraph);
                smootherBatch.update(nfg, initialEstimate, PoseTimestamps);
                nfg.resize(0);
                initialEstimate.clear();
                PoseTimestamps.clear();
                priorFactorGraph.resize(0);
            }
            else {
                initialEstimate.insert(X(frame), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
                PoseTimestamps[X(frame)] = count;

                count++;

                addLandmarkToGraph(frame, lmks_in_keys, fts, globalSmartFactor, cameraRig, smootherBatch);
                auto start = std::chrono::high_resolution_clock::now();
                smootherBatch.update(nfg, initialEstimate, PoseTimestamps);
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                cout << "Time to update: " << elapsed.count() << " s\n";
                total_time += elapsed.count();
                cout << "Total time: " << total_time << " s\n";
                auto result = smootherBatch.calculateEstimate();

                nfg.resize(0);
                initialEstimate.clear();
                PoseTimestamps.clear();
            }
            continue;
        }
    }

  return 0;
}



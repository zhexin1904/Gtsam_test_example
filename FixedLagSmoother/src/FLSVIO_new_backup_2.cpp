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
#include <gtsam/geometry/triangulation.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>

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
typedef gtsam::SmartProjectionRigFactor<gtsam::PinholePose<gtsam::Cal3_S2>> RigFactor;
// from KimeraVIO
using Slot = long int;
using LandmarkId = long int;
typedef std::map<LandmarkId, vector<double>> lmk_nKFs;

using SmartFactorMap =
    gtsam::FastMap<LandmarkId, std::pair<RigFactor::shared_ptr, Slot>>;


using LandmarkIdSmartFactorMap = std::map<LandmarkId, RigFactor::shared_ptr>;

using CameraRig = boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2>>>;

typedef std::map<Key, std::set<Key> > GraphFactorIndex;

std::vector<double> u_coord, v_coord;
std::vector<int> pose_ids, cam_ids;
std::map<int, gtsam::MatrixRowMajor> chosen_poses;

class SlidingWindow {
public:
    SlidingWindow(int size) : maxSize(size) {}

    void addElement(int element) {
        window.push_back(element); // Add the new element
    }

    void removeElement() {
        if (window.size() > maxSize) {
            window.pop_front(); // Remove the oldest element
        }
    }

    void printWindow() const {
        for (int element : window) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }

    bool contains(int element) const {
        return std::find(window.begin(), window.end(), element) != window.end();
    }

    bool toMarginalOut(int element) const {
        if (!window.empty() && window.front() == element) {
            return true;
        }
        return false;
    }

private:
    std::deque<int> window;
    int maxSize;
};

void writeMatrixToFile(const Eigen::MatrixXd& matrix_schur, const Eigen::MatrixXd& matrix_hessian, const std::string& filename) {
    // 
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return;
    }

    outFile << "*****************Hessian**********************" << std::endl; 
    for (int i = 0; i < matrix_hessian.rows(); ++i) {
        for (int j = 0; j < matrix_hessian.cols(); ++j) {
            outFile << matrix_hessian(i, j);
            if (j < matrix_hessian.cols() - 1) {
                outFile << ", ";  
            }
        }
        outFile << std::endl;  
    }
    outFile << "*****************Schur**********************" << std::endl; 
    for (int i = 0; i < matrix_schur.rows(); ++i) {
        for (int j = 0; j < matrix_schur.cols(); ++j) {
            outFile << matrix_schur(i, j);
            if (j < matrix_schur.cols() - 1) {
                outFile << ", ";  
            }
        }
        outFile << std::endl;  
    }
    
    outFile.close();

    std::cout << "Matrix has been written to " << filename << std::endl;
}





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

gtsam::NonlinearFactorGraph addLandmarkToGraph(int kfID,
                        const LmkID_in_Key& LmkID_in_Key_,
                        FeatureTracks& feature_tracks_,
                        SmartFactorMap& smartFactorMap_,
                        FastVector<FactorIndex>& toBeDeleted,
                        const CameraRig& cameraRig_,
                        const SlidingWindow& SW_,
                        std::unique_ptr<gtsam::BatchFixedLagSmoother>&& smootherBatch) {
    gtsam::NonlinearFactorGraph nfg;
    int n_new_landmarks = 0;
    int n_updated_landmarks = 0;
    int kfID_count = 0;
    SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(2, 50.0);
    auto currentLmk_id_it = LmkID_in_Key_.find(kfID);
    std::vector<LandmarkId> currentLmk_id_;
    if (currentLmk_id_it != LmkID_in_Key_.end()) {
         currentLmk_id_ = currentLmk_id_it->second;
    }

  for (const LandmarkId& lmk_id : currentLmk_id_) {
    FeatureTrack& ft = feature_tracks_.at(lmk_id);

    // Here we need to find the KF count between the current KFID and the first KF in the sliding window
    // if the landmark is created for the first time, the first KFID would be in the window
    // if the landmark has some kFS that were margianlized out we want the starting KF to be the first one seen in SW.
    auto kfID_it = std::find(ft.kfs_in_lmkID.begin(), ft.kfs_in_lmkID.end(), kfID);
    auto firstKFID_it = kfID_it;
      for (int i = 0; i < ft.kfs_in_lmkID.size(); ++i) {
          if (SW_.contains(ft.kfs_in_lmkID[i])) {
              firstKFID_it = ft.kfs_in_lmkID.begin() + i;
              break;
          }
      }


      if (kfID_it != ft.kfs_in_lmkID.end()) {
          kfID_count = std::distance(firstKFID_it, kfID_it);
      }
      if ((kfID_count + 1) < 2) {
          continue;
      }

    // current landmark has not been observed before, create smart factor for it
    if (smartFactorMap_.count(lmk_id) == 0) {

        // kfID_count is actually the index of current key in the vector : ft.kfs_in_lmkID
        // if all the  kfID of current landmark is outside the window, we will discard this observation
        // but just need to verify the key before current key
//        if (!SW_.contains(ft.kfs_in_lmkID[kfID_count - 1])) {
//                    continue;
//        }
/// NO longer need the above and also it is technically wrong

        RigFactor::shared_ptr newFactor(new RigFactor(gaussian, cameraRig_, params));
        for (const std::tuple<int, int, double, double>& obs : ft.obs_) {
            // make sure we won't pick up "future" measurement
            // we get obs in order, continuously ,so use "break" later
            const int frame = std::get<0>(obs);
            if (!SW_.contains(frame)) {
                continue;
            }
            else {
            const int CamID = std::get<1>(obs);
            const double measurement_u = std::get<2>(obs);
            const double measurement_v = std::get<3>(obs);
            newFactor->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
            }
        }
        smartFactorMap_.insert(std::pair(lmk_id, std::make_pair(newFactor, -1)));
        nfg.push_back(smartFactorMap_[lmk_id].first);
        ++n_new_landmarks;
    }

    else {
            bool need_new_smart_factor = false;

            // Check if any keyframe is outside the sliding window
            for (int i = 0; i < kfID_count; ++i) {
                if (!SW_.contains(ft.kfs_in_lmkID[i])) {
                    need_new_smart_factor = true;
                    SW_.printWindow();
                    break;
                }
            }

            if (need_new_smart_factor) {
                //This current smart factor has an observation outside of the smoothing window
                // hence we create a new smart factor with obsertaions in the window and delete the
                //old smart factor
                Slot facInd = smartFactorMap_[lmk_id].second;
                assert( facInd != -1);
                assert(smootherBatch->getFactors().exists(facInd));
                // add the current factor index of this smart factorinside smoother object to be deleted
                toBeDeleted.push_back(smartFactorMap_[lmk_id].second);
                //create a new smart factor for thi lmid with updated observations within the window
                // and update the global smart factor map as well we add to nfg
                smartFactorMap_[lmk_id].first = RigFactor::shared_ptr(new RigFactor(gaussian, cameraRig_, params));
                for (const std::tuple<int, int, double, double> &obs : ft.obs_) {
                    if (!SW_.contains(std::get<0>(obs))) {
                        continue;
                    }
                    const int frame = std::get<0>(obs);
                    const int CamID = std::get<1>(obs);
                    const double measurement_u = std::get<2>(obs);
                    const double measurement_v = std::get<3>(obs);
                    smartFactorMap_[lmk_id].first->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
                }
                nfg.push_back(smartFactorMap_[lmk_id].first);
            } else {
                //Here you just update the smart factor by adding the extra measurement, but n
                Slot facInd = smartFactorMap_[lmk_id].second;
                assert( facInd != -1);
                assert(smootherBatch->getFactors().exists(facInd));
                // add the current factor index of this smart factorinside smoother object to be deleted
                toBeDeleted.push_back(smartFactorMap_[lmk_id].second);
                smartFactorMap_[lmk_id].first = RigFactor::shared_ptr(new RigFactor(gaussian, cameraRig_, params));
                for (const std::tuple<int, int, double, double> &obs : ft.obs_) {
                    if (!SW_.contains(std::get<0>(obs))) {
                        continue;
                    }
                    const int frame = std::get<0>(obs);
                    const int CamID = std::get<1>(obs);
                    const double measurement_u = std::get<2>(obs);
                    const double measurement_v = std::get<3>(obs);
                    smartFactorMap_[lmk_id].first->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
                }
                nfg.push_back(smartFactorMap_[lmk_id].first);

            }

            ++n_updated_landmarks;
    }

  }

    std::cout << "Added " << n_new_landmarks << " new landmarks" << std::endl
           << "Updated " << n_updated_landmarks << " landmarks in graph" << std::endl;

    return nfg;

}

gtsam::KeyVector unionOfKeyVectors(const gtsam::KeyVector& vec1, const gtsam::KeyVector& vec2) {
    std::set<gtsam::Key> unionSet(vec1.begin(), vec1.end()); // Insert elements of vec1 into set

    // Insert elements of vec2 into set
    unionSet.insert(vec2.begin(), vec2.end());

    // Convert set back to KeyVector
    gtsam::KeyVector unionVec(unionSet.begin(), unionSet.end());
    return unionVec;
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


    LandmarkIdSmartFactorMap newSmartFactor;
    LandmarkIdSmartFactorMap newSmartFactor_;
    std::map<int, RigFactor::shared_ptr> smartFactor;
    boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2>>> cameraRig;

    vector<gtsam::Cal3_S2::shared_ptr> K;
    gtsam::Cal3_S2::shared_ptr K0;
    gtsam::Cal3_S2::shared_ptr K1;

    gtsam::NonlinearFactorGraph new_graph;
    gtsam::NonlinearFactorGraph priorFactorGraph;
    gtsam::NonlinearFactorGraph new_graph_processed;

    std::string in_frontend("/home/jason/DPGO/FixedLagSmoother/newLogs_Lmk.txt");
    std::string in_initials("/home/jason/DPGO/FixedLagSmoother/newLogs_initials.txt");
    std::string out_Matrix("/home/jason/DPGO/FixedLagSmoother/Schur.txt");

    double totalTime = 0;
    vector<double> T;
    double lag = 5.0;
    int camNum = 2;
    SlidingWindow SW(lag + 1);

//
    LevenbergMarquardtParams parameters_lm;

    parameters_lm.lambdaInitial = 1e-2;
    parameters_lm.maxIterations = 15;
    parameters_lm.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_QR;
    parameters_lm.setVerbosityLM("SUMMARY");
    parameters_lm.setOrderingType("COLAMD");
    std::unique_ptr<gtsam::BatchFixedLagSmoother> smootherBatch;
    smootherBatch = std::make_unique<gtsam::BatchFixedLagSmoother>(lag, parameters_lm);
    gtsam::BatchFixedLagSmoother smootherBtach_exception(lag, parameters_lm);

    // gtsam::BatchFixedLagSmoother smootherBatch(lag, parameters_lm);
    // gtsam::BatchFixedLagSmoother smootherBtach_exception(lag, parameters_lm);
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1; //
    parameters.relinearizeSkip = 1; //
    gtsam::IncrementalFixedLagSmoother smootherISAM2(lag, parameters);
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::FixedLagSmoother::KeyTimestampMap PoseTimestamps;
    gtsam::FixedLagSmoother::Result result;
    ISAM2 isam(parameters);

    SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(2, 2.0);


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
    SmartFactorMap globalSmartFactorMap;
    
    FeatureTracks fts;
    ReadingLmk(fts, in_frontend);
    LmkID_in_Key lmks_in_keys;
    lmks_in_keys = invertMap(fts) ;
    

    while (in_initial >> initial_line){
        if (initial_line == "x") {
            in_initial >> frame >> temp;
            for (int i = 0; i < 16; i++) {
                in_initial >> m.data()[i];
            }
            gtsam::Pose3 pose(m);
//            pose.print();
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
                // priorFactorGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(frame), Pose3(pose), priorPoseNoise);
                initialEstimate.insert(X(frame), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
                PoseTimestamps[X(frame)] = count;
                SW.addElement(frame);
                count++;
                initialized = true;
                new_graph.push_back(priorFactorGraph);
                priorFactorGraph.resize(0);
            }
            else {
                initialEstimate.insert(X(frame), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
                PoseTimestamps[X(frame)] = count;
                SW.addElement(frame);
                count++;
    //              get all the feature tracks for the current frame
                std::vector<LandmarkId> currentLmk_id_;
                std::vector<FeatureTrack> current_fts_;
                FastVector<FactorIndex> toBeDeleted;
                FastVector<FactorIndex> toBeDeleted_exception;
                gtsam::KeyVector tobeDeleted_new;
                new_graph =  addLandmarkToGraph(frame, lmks_in_keys, fts, globalSmartFactorMap, toBeDeleted, cameraRig, SW, std::move(smootherBatch));
                // SW.addElement(frame);
                    cout<<"keys involved in factors of new graph: "<<endl;

                    new_graph.keys().print();
                    cout<<endl<<"keys involved in factors of the smoother before optimization: "<<endl;
                    smootherBatch->getFactors().keys().print();
                    // gtsam::KeyVector currentKeys = smootherBatch.getFactors().keys();
                    cout<<endl;

                    // put checking invalid landmark in addLandmarkToGraph ? or here
                    
                    try {
                        smootherBtach_exception = *smootherBatch;
                        smootherBatch->update(new_graph, initialEstimate, PoseTimestamps,toBeDeleted);
                        std::cout << "after marg **************" << std::endl;
                        smootherBatch->getFactors().keys().print();
                        const gtsam::NonlinearFactorGraph& all_factors = smootherBatch->getFactors();

                        cout<<"Number of Factors in new graph: "<<new_graph.size()<<endl;
                        cout<<"Number of Factors in smoother: "<<all_factors.size()<<endl;

                        const auto& currentEstimate = smootherBatch->calculateEstimate();
                        gtsam::KeyVector keys = currentEstimate.keys();


                        for (auto it = globalSmartFactorMap.begin(); it != globalSmartFactorMap.end(); ){
                            Slot facInd = it->second.second;

                            int ind = 0;
                            for (const auto fac : all_factors) {
                                if (fac == it->second.first){
                                    it->second.second = ind;
                                    break;
                                }
                                ind++;
                            }
                            if (ind == all_factors.size()){

                                it = globalSmartFactorMap.erase(it);
                                continue;
                            }

                            it++;

                        }
                        std::set<gtsam::Key> keys_present;
                        for (const auto fac : all_factors) {
                            if (fac != nullptr)
                                for (auto k : fac->keys())
                                    keys_present.insert(k); //fac->printKeys();
                        }
                        for (auto i : keys_present) {
                            cout<<gtsam::Symbol(i)<<", ";
                        }
                        cout<<endl;
                        SW.removeElement();
                        SW.printWindow();
                        auto result = smootherBatch->calculateEstimate();
                        // result.print("Result: ");
                        new_graph.resize(0);
                        initialEstimate.clear();
                        PoseTimestamps.clear();
                    }

                  catch (IndeterminantLinearSystemException& e) {
                    const NonlinearFactorGraph ng = smootherBatch->getFactors();
                    auto res = smootherBatch->calculateEstimate();
                    auto linear_graph = ng.linearize(res);
//                    go through all the factors in the linear gaussian graph
                        int ind = 0;
                        int delete_count = 0;
                        size_t ng_size = ng.size();
                    for (auto factor : *linear_graph) {
                        if(factor) {
//                            get the hessian factor for the linear gaussian factor
                            auto hessian_factor = HessianFactor(*factor);
                            gtsam::VerticalBlockMatrix Ab_(VerticalBlockMatrix::LikeActiveViewOf(hessian_factor.info(), hessian_factor.rows()));
                            Ab_.full() = hessian_factor.info().selfadjointView();
                            size_t maxrank;
                            bool success;
                            boost::tie(maxrank, success) = choleskyCareful(Ab_.matrix());
                            if (!(success || maxrank == hessian_factor.rows() - 1)) {
                                std::cout << "Omitting this smart factor" << std::endl;
                                toBeDeleted_exception.push_back(ind);
                                ++delete_count;
                                ng.at(ind)->print();
                            }
                        }
                        ind++;
                    }
                    // gtsam::KeyVector tobeDeleted_new = unionOfKeyVectors(toBeDeleted, toBeDeleted_exception);
                    toBeDeleted.insert(toBeDeleted.end(), toBeDeleted_exception.begin(), toBeDeleted_exception.end());
                    
                        *smootherBatch = smootherBtach_exception;
                        smootherBatch->update(new_graph, initialEstimate, PoseTimestamps, toBeDeleted);
                        const gtsam::NonlinearFactorGraph& all_factors = smootherBatch->getFactors();

                        cout<<"Number of Factors in new graph: "<<new_graph.size()<<endl;
                        cout<<"Number of Factors in smoother: "<<all_factors.size()<<endl;

                        const auto& currentEstimate = smootherBatch->calculateEstimate();
                        gtsam::KeyVector keys = currentEstimate.keys();


                        for (auto it = globalSmartFactorMap.begin(); it != globalSmartFactorMap.end(); ){
                            Slot facInd = it->second.second;

                            int ind = 0;
                            for (const auto fac : all_factors) {
                                if (fac == it->second.first){
                                    it->second.second = ind;
                                    break;
                                }
                                ind++;
                            }
                            if (ind == all_factors.size()){

                                it = globalSmartFactorMap.erase(it);
                                continue;
                            }

                            it++;

                        }
                        std::set<gtsam::Key> keys_present;
                        for (const auto fac : all_factors) {
                            if (fac != nullptr)
                                for (auto k : fac->keys())
                                    keys_present.insert(k); //fac->printKeys();
                        }
                        for (auto i : keys_present) {
                            cout<<gtsam::Symbol(i)<<", ";
                        }
                        cout<<endl;
                        SW.removeElement();
                        SW.printWindow();
                        auto result = smootherBatch->calculateEstimate();
                        // result.print("Result: ");
                        new_graph.resize(0);
                        initialEstimate.clear();
                        PoseTimestamps.clear();

                    
                }  

            }
            continue;
        }
    }

  return 0;
}


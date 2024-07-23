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
#include <gtsam/base/FastMap.h>

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
typedef gtsam::SmartProjectionRigFactor<gtsam::PinholePose<gtsam::Cal3_S2>> RigFactor;
// from KimeraVIO
using Slot = long int;
using LandmarkId = long int;
using SmartFactorMap =
        gtsam::FastMap<LandmarkId, std::pair<RigFactor::shared_ptr, Slot>>;
// using LandmarkIdSmartFactorMap =
//     std::unordered_map<LandmarkId, RigFactor::shared_ptr>;

using LandmarkIdSmartFactorMap = std::map<LandmarkId, RigFactor::shared_ptr>;
std::vector<double> u_coord, v_coord;
std::vector<int> pose_ids, cam_ids;
std::map<int, gtsam::MatrixRowMajor> chosen_poses;


SmartFactorMap updateNewSmartFactorsSlots(
        const std::vector<LandmarkId>& lmk_ids_of_new_smart_factors_,
        SmartFactorMap old_smart_factors_,
        gtsam::IncrementalFixedLagSmoother smootherISAM2_){

    SmartFactorMap old_smart_factors(old_smart_factors_);

    const gtsam::ISAM2Result& result = smootherISAM2_.getISAM2Result();

    for (size_t i = 0u; i < lmk_ids_of_new_smart_factors_.size(); ++i) {

        const size_t& slot = result.newFactorsIndices.at(i);

        const auto& it = old_smart_factors.find(lmk_ids_of_new_smart_factors_.at(i));

        const auto sptr = dynamic_cast<const RigFactor*>(
                smootherISAM2_.getFactors().at(slot).get());

        if(sptr) {
            it->second.second = slot;
        }
        // it->second.second = slot;
    }

    return old_smart_factors;
}



void PrintSymbolicFactor(
        const NonlinearFactor::shared_ptr& factor) {
    cout << "f(";
    if (factor) {
        for(Key key: factor->keys()) {
            cout << " " << DefaultKeyFormatter(key);
        }
    } else {
        cout << " nullptr";
    }
    cout << " )" << endl;
}

void PrintSymbolicGraph(
        const NonlinearFactorGraph& graph, const string& label) {
    cout << label << endl;
    for(const auto& factor: graph) {
        PrintSymbolicFactor(factor);
    }
}



int main(int argc, char* argv[]) {
    gtsam::Rot3 R_b_cam0(0.0051989, -0.00475195, 0.99997519,
                         -0.99998009, -0.00360067, 0.00518181,
                         0.00357595, -0.99998223, -0.00477058);

    gtsam::Point3 t_b_cam0(0.01782353, 0.02405353, 0.00652621);
    gtsam::Pose3 T_b_cam0(R_b_cam0, t_b_cam0);

    gtsam::Rot3 R_cam1_cam0(0.9999030819138741, 0.0010711343462142993, 0.013880902361026801,
                            -0.0011234892556085138, 0.9999922832259254, 0.003764473469264149,
                            -0.013876762988410936, -0.003779703668360986, 0.99989656929562);

    gtsam::Point3 t_cam1_cam0(-0.1220026692068293, 0.00011691065038523982, 0.0023967307199285926);
    gtsam::Pose3 T_cam1_cam0(R_cam1_cam0, t_cam1_cam0);


//    LandmarkIdSmartFactorMap newSmartFactor;
    std::map<int, RigFactor::shared_ptr> smartFactor;
    boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2>>> cameraRig;

    vector<gtsam::Cal3_S2::shared_ptr> K;
    gtsam::Cal3_S2::shared_ptr K0;
    gtsam::Cal3_S2::shared_ptr K1;

    // from KimeraVIO
    // gtsam::FactorIndices toDelete;
    gtsam::NonlinearFactorGraph new_graph;
    SmartFactorMap old_smart_factors;
    // std::vector<LandmarkId> lmk_ids_of_new_smart_factors;
    LandmarkIdSmartFactorMap newSmartFactor_;
    gtsam::NonlinearFactorGraph priorFactorGraph;

    ifstream in_frontend("/home/neural/fixedlagsmoothing/old_graph_logs_indoor.txt");
//    ifstream in_initial("/home/neural/fixedlagsmoothing/initiasl_values_processed_VO.txt");

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
//    gtsam::NonlinearFactorGraph graph_tmp;

    gtsam::Values initialEstimate;
    gtsam::FixedLagSmoother::KeyTimestampMap PoseTimestamps;
    gtsam::FixedLagSmoother::Result result;
    ISAM2 isam(parameters);

    SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(2, 50.0);

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
    cameraRig = static_cast<boost::shared_ptr<gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2> > > >(
            new gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3_S2> >());


    gtsam::PinholePose<gtsam::Cal3_S2> cam0(T_b_cam0, K0);
    cameraRig->push_back(cam0);
    gtsam::PinholePose<gtsam::Cal3_S2> cam1(T_b_cam0 * T_cam1_cam0.inverse(), K1);
    cameraRig->push_back(cam1);
    cameraRig->print("cameraRig");


    auto priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());


    double x1, x2, x3, x4, x5, x6;

//    int frame_initial;
//    char type_initial;

//    int lastFrame = 6;
    int frame;
//    char type_frontend;
    bool initialized = false;

    string line, temp;
    gtsam::MatrixRowMajor m(4, 4);
    LandmarkId landmark;

    while (in_frontend >> line) {

//     TODO: this should happen later after adding smart factors to graph so have a separate graph and initial estimate var for this
        if (line == "x") {
            if (initialized) {
                gtsam::FactorIndices toDelete;
                std::vector<LandmarkId> lmk_ids_of_new_smart_factors;
                size_t size = newSmartFactor_.size();
                lmk_ids_of_new_smart_factors.reserve(size);
                gtsam::NonlinearFactorGraph new_factors_tmp;
                for (const auto &new_smart_factor: newSmartFactor_) {
                    LandmarkId lmk_id = new_smart_factor.first;
                    const auto &old_smart_factors_it = old_smart_factors.find(lmk_id);
                    if (old_smart_factors_it != old_smart_factors.end()) {
                        Slot slot = old_smart_factors_it->second.second;
                        if (slot != -1) { // adding before, already exists // slot=-1 means the lmkID is new
                            if (smootherISAM2.getFactors().exists(slot)) {
                                toDelete.push_back(slot);
                                new_factors_tmp.push_back(new_smart_factor.second);
                                lmk_ids_of_new_smart_factors.push_back(lmk_id);
                            } else {
                                old_smart_factors.erase(old_smart_factors_it);
                                std::cout << "slot not exist in smootherBatch.getFactors() " << std::endl;
                            }
                        } else {
                            new_factors_tmp.push_back(new_smart_factor.second);
                            lmk_ids_of_new_smart_factors.push_back(lmk_id);
                        }
                    }
                }
                if (frame == 3) {
                    new_factors_tmp.push_back(priorFactorGraph.begin(), priorFactorGraph.end());
                    priorFactorGraph.resize(0);
                }

                smootherISAM2.update(new_factors_tmp, initialEstimate, PoseTimestamps, toDelete);
                

                gtsam::NonlinearFactorGraph graph_in_smoother = smootherISAM2.getFactors();

                std::cout << "graph_in_smoother.print" << std::endl;

                gtsam::Values currentEstimate = smootherISAM2.calculateEstimate();

                currentEstimate.print("\t");

                old_smart_factors = updateNewSmartFactorsSlots(lmk_ids_of_new_smart_factors, old_smart_factors,
                                                               smootherISAM2);
                new_factors_tmp.resize(0);
                graph.resize(0);
//               graph_tmp.resize(0);
                initialEstimate.clear();
                PoseTimestamps.clear();
                // toDelete.clear();
//               newSmartFactor.clear();
                newSmartFactor_.clear();
            }
            in_frontend >> frame >> temp;
            for (int i = 0; i < 16; i++) {
                in_frontend >> m.data()[i];
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
                PoseTimestamps[X(frame)] = frame;

                initialized = true;
            } else {
                initialEstimate.insert(X(frame), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
                PoseTimestamps[X(frame)] = frame;
            }
            chosen_poses[frame] = m;
            continue;
        } else if (line == "l") {   // if the landmark is at least seen from two LF frames add the factor.
            if (u_coord.size() >= 2) {
                if (newSmartFactor_.count(landmark) == 0) {
                    if (old_smart_factors.count(landmark) == 0) {
                        RigFactor::shared_ptr newFactor(new RigFactor(gaussian, cameraRig, params));
                        for (int idx = 0; idx < u_coord.size(); idx++) {
                            gtsam::Point2 measurement(u_coord[idx], v_coord[idx]);
                            // Add to the smart factor
                            newFactor->add(measurement, X(pose_ids[idx]), cam_ids[idx]);
                        }
                        newSmartFactor_.insert(std::make_pair(landmark, newFactor));
                        old_smart_factors.insert(
                                std::make_pair(landmark, std::make_pair(newSmartFactor_[landmark], -1)));
                    } else {
                        auto old_smart_factors_it = old_smart_factors.find(landmark);
                        if (old_smart_factors_it != old_smart_factors.end()) {
                            const auto &old_factor = old_smart_factors_it->second.first;

                            RigFactor::shared_ptr newFactor(new RigFactor(*old_factor));
                            // printKeys just for verify that newFactor have save the measurement from old_factor
                            newFactor->printKeys();
                            newFactor->print();
                            for (int idx = 0; idx < u_coord.size(); idx++) {
                                gtsam::Point2 measurement(u_coord[idx], v_coord[idx]);
                                // Add to the smart factor
                                newFactor->add(measurement, X(pose_ids[idx]), cam_ids[idx]);
                            }
                            newFactor->printKeys();
                            newFactor->print();
                            Slot slot = old_smart_factors_it->second.second;
                            if (slot != -1) {
                                newSmartFactor_.insert(std::make_pair(landmark, newFactor));
                            } else {
                                std::cerr << "When updating the smart factor, its slot should not be -1!"
                                             " Offensive lmk_id: "
                                          << landmark << std::endl;
                            }
                            old_smart_factors_it->second.first = newFactor;
                        }
                    }
                }
// //               don't think this else is needed as we store the everything in a vector and add and in new SF everything is a new lm, if needed will use later
//               else {
//                   newSmartFactor_[landmark]->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
//               }
                u_coord.clear();
                v_coord.clear();
                pose_ids.clear();
                cam_ids.clear();
            } else {
                u_coord.clear();
                v_coord.clear();
                pose_ids.clear();
                cam_ids.clear();
            }
            in_frontend >> landmark;
            continue;
        } else if (line == "e") {
            int CamID;
            double measurement_u, measurement_v;
            in_frontend >> frame >> CamID >> measurement_u >> measurement_v;
            // Don't add the negative measurements.
            if (measurement_u < 0.0 || measurement_v < 0.0) {
                continue;
            }
            if (chosen_poses.find(frame) != chosen_poses.end()) {
                u_coord.push_back(measurement_u);
                v_coord.push_back(measurement_v);
                pose_ids.push_back(frame);
                cam_ids.push_back(CamID);
            }
            continue;
        }
    }
    return 0;
}

//        char line_frontend[1024];
//        in_frontend.getline(line_frontend, sizeof(line_frontend));
//        stringstream ss_frontend(line_frontend);
//
//        ss_frontend >> type_frontend;
//        ss_frontend >> frame;

        // adding measurement from txt
//        if (type_frontend == 'l') {
//            std::string temp;
//            LandmarkId landmark;
//            int CamID;
//            double measurement_u, measurement_v;
//            ss_frontend >> temp >> landmark;
//            ss_frontend >> temp >> CamID;
//            ss_frontend >> temp >> measurement_u >> measurement_v;
//
//            if (newSmartFactor_.count(landmark) == 0 ) {
//                if (old_smart_factors.count(landmark) == 0) {
//
//                    SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
//                    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(2, 50.0);
//                    RigFactor::shared_ptr newFactor(new RigFactor(gaussian, cameraRig, params));
//                    newFactor->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
//                    newSmartFactor_.insert(std::make_pair(landmark, newFactor));
//                    old_smart_factors.insert(std::make_pair(landmark, std::make_pair(newSmartFactor_[landmark], -1)));
//                    // old_smart_factors[landmark] = std::make_pair(newSmartFactor[landmark], -1);
//                }
//                else {
//                    auto old_smart_factors_it = old_smart_factors.find(landmark);
//                    if (old_smart_factors_it != old_smart_factors.end()) {
//                        const auto& old_factor = old_smart_factors_it->second.first;
//
//                        RigFactor::shared_ptr newFactor(new RigFactor(*old_factor));
//                        // printKeys just for verify that newFactor have save the measurement from old_factor
//                        newFactor->printKeys();
//                        newFactor->print();
//                        newFactor->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
//                        newFactor->printKeys();
//                        newFactor->print();
//                        Slot slot = old_smart_factors_it->second.second;
//                        if (slot != -1) {
//                            newSmartFactor_.insert(std::make_pair(landmark, newFactor));
//                        } else {
//                            std::cerr << "When updating the smart factor, its slot should not be -1!"
//                                         " Offensive lmk_id: "
//                                      << landmark << std::endl;
//                        }
//                        old_smart_factors_it->second.first = newFactor;
//                    }
//                }
//            }
//            else {
//
//                newSmartFactor_[landmark]->add(gtsam::Point2(measurement_u, measurement_v), X(frame), CamID);
//
//
//            }
//
//        } else {
//            throw runtime_error("unexpected data type: " + string(1, type_frontend));
//        }

//        char line_initial[1024];
//        in_initial.getline(line_initial, sizeof(line_initial));
//        stringstream ss_initial(line_initial);
//
//        ss_initial >> type_initial >> frame_initial;
//
//        ss_initial >> x1 >> x2 >> x3 >> x4 >> x5 >> x6; // x4:roll, x5:pitch, x6:yaw
//        if (!initialized) {
//            priorFactorGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(6), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1 - 0.001, x2 - 0.001, x3 - 0.001)), priorPoseNoise);
//            initialEstimate.insert(X(frame_initial), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
//            PoseTimestamps[X(frame_initial)] = frame_initial;
//
//            initialized = true;
//        }
//        else {
//            initialEstimate.insert(X(frame_initial), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
//            PoseTimestamps[X(frame_initial)] = frame_initial;
//        }

        // if (frame_initial > 6){
//        gtsam::FactorIndices toDelete;
//        std::vector<LandmarkId> lmk_ids_of_new_smart_factors;
//        size_t size = newSmartFactor_.size();
//        lmk_ids_of_new_smart_factors.reserve(size);
//        gtsam::NonlinearFactorGraph new_factors_tmp;
//        for (const auto& new_smart_factor : newSmartFactor_)
//        {
//            LandmarkId lmk_id = new_smart_factor.first;
//            const auto& old_smart_factors_it = old_smart_factors.find(lmk_id);
//            if (old_smart_factors_it != old_smart_factors.end()) {
//                Slot slot = old_smart_factors_it->second.second;
//                if (slot != -1) { // adding before, already exists // slot=-1 means the lmkID is new
//                    if (smootherISAM2.getFactors().exists(slot)){
//                        toDelete.push_back(slot);
//                        new_factors_tmp.push_back(new_smart_factor.second);
//                        lmk_ids_of_new_smart_factors.push_back(lmk_id);
//                    }
//                    else {
//                        old_smart_factors.erase(old_smart_factors_it);
//                        std::cout << "slot not exist in smootherBatch.getFactors() " << std::endl;
//                    }
//                }
//                else {
//                    new_factors_tmp.push_back(new_smart_factor.second);
//                    lmk_ids_of_new_smart_factors.push_back(lmk_id);
//                }
//            }
//
//        }



//        if (frame_initial > 5) {
            // hard coded here, for adding prior
//            if (frame_initial == 6) {
//                new_factors_tmp.push_back(priorFactorGraph.begin(), priorFactorGraph.end());
//                priorFactorGraph.resize(0);
//            }
//
//            smootherISAM2.update(new_factors_tmp, initialEstimate, PoseTimestamps, toDelete);
//
//            gtsam::NonlinearFactorGraph grapg_in_smoother = smootherISAM2.getFactors();
//
//            std::cout << "grapg_in_smoother.print" << std::endl;
//
//            gtsam::Values currentEstimate = smootherISAM2.calculateEstimate();
//
//            currentEstimate.print("\t");
//
//            old_smart_factors = updateNewSmartFactorsSlots(lmk_ids_of_new_smart_factors, old_smart_factors, smootherISAM2);
//            new_factors_tmp.resize(0);
//            graph.resize(0);
//            graph_tmp.resize(0);
//            initialEstimate.clear();
//            PoseTimestamps.clear();
//            // toDelete.clear();
//            newSmartFactor.clear();
//            newSmartFactor_.clear();
//        }
//        // }
//        if (in_frontend.eof()) {
//            break;
//        }
//        lastFrame = frame;


//        if ( frame != lastFrame || in_frontend.eof()) {
//            char line_initial[1024];
//            in_initial.getline(line_initial, sizeof(line_initial));
//            stringstream ss_initial(line_initial);
//
//            ss_initial >> type_initial >> frame_initial;
//
//            ss_initial >> x1 >> x2 >> x3 >> x4 >> x5 >> x6; // x4:roll, x5:pitch, x6:yaw
//            if (!initialized) {
//                priorFactorGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(6), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1 - 0.001, x2 - 0.001, x3 - 0.001)), priorPoseNoise);
//                initialEstimate.insert(X(frame_initial), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
//                PoseTimestamps[X(frame_initial)] = frame_initial;
//
//                initialized = true;
//            }
//            else {
//                initialEstimate.insert(X(frame_initial), Pose3(Rot3::Ypr(x6, x5, x4), Point3(x1, x2, x3)));
//                PoseTimestamps[X(frame_initial)] = frame_initial;
//            }
//
//
//
//            // if (frame_initial > 6){
//            gtsam::FactorIndices toDelete;
//            std::vector<LandmarkId> lmk_ids_of_new_smart_factors;
//            size_t size = newSmartFactor_.size();
//            lmk_ids_of_new_smart_factors.reserve(size);
//            gtsam::NonlinearFactorGraph new_factors_tmp;
//            for (const auto& new_smart_factor : newSmartFactor_)
//            {
//                LandmarkId lmk_id = new_smart_factor.first;
//                const auto& old_smart_factors_it = old_smart_factors.find(lmk_id);
//                if (old_smart_factors_it != old_smart_factors.end()) {
//                    Slot slot = old_smart_factors_it->second.second;
//                    if (slot != -1) { // adding before, already exists // slot=-1 means the lmkID is new
//                        if (smootherISAM2.getFactors().exists(slot)){
//                            toDelete.push_back(slot);
//                            new_factors_tmp.push_back(new_smart_factor.second);
//                            lmk_ids_of_new_smart_factors.push_back(lmk_id);
//                        }
//                        else {
//                            old_smart_factors.erase(old_smart_factors_it);
//                            std::cout << "slot not exist in smootherBatch.getFactors() " << std::endl;
//                        }
//                    }
//                    else {
//                        new_factors_tmp.push_back(new_smart_factor.second);
//                        lmk_ids_of_new_smart_factors.push_back(lmk_id);
//                    }
//                }
//
//            }
//
//
//
//            if (frame_initial > 5) {
//                // hard coded here, for adding prior
//                if (frame_initial == 6) {
//                    new_factors_tmp.push_back(priorFactorGraph.begin(), priorFactorGraph.end());
//                    priorFactorGraph.resize(0);
//                }
//
//                smootherISAM2.update(new_factors_tmp, initialEstimate, PoseTimestamps, toDelete);
//
//                gtsam::NonlinearFactorGraph grapg_in_smoother = smootherISAM2.getFactors();
//
//                std::cout << "grapg_in_smoother.print" << std::endl;
//
//                gtsam::Values currentEstimate = smootherISAM2.calculateEstimate();
//
//                currentEstimate.print("\t");
//
//                old_smart_factors = updateNewSmartFactorsSlots(lmk_ids_of_new_smart_factors, old_smart_factors, smootherISAM2);
//                new_factors_tmp.resize(0);
//                graph.resize(0);
//                graph_tmp.resize(0);
//                initialEstimate.clear();
//                PoseTimestamps.clear();
//                // toDelete.clear();
//                newSmartFactor.clear();
//                newSmartFactor_.clear();
//            }
//            // }
//            if (in_frontend.eof()) {
//                break;
//            }
//        }
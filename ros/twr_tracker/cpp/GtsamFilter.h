#ifndef GTSAMFILTER_H
#define GTSAMFILTER_H

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <slam/BedPose.h>
#include <slam/AnchorPoint.h>
#include <slam/LocationGTSAM.h> // For the typedefs
#include <slam/SvgMap.h>

#include <vector>
#include <deque>
#include <boost/tuple/tuple.hpp>
namespace NM = gtsam::noiseModel;

typedef struct {
    double t;
    float x;
    float y;
    float z;
    float theta;
} simple_location_t;

typedef struct {
    double t; // timestamp
    float r; // range in meters
    uint16_t tag_id;
    uint16_t anchor_id;
} lps_range_t;

typedef struct {
    unsigned inliers;
    double error;
    std::vector<int> inlier_index;
} support_check_t;

#ifndef mi_odometry_t 
typedef struct {
    double t;
    double s;
    double theta; // left/right
    double phi;   // up/down
} mi_odometry_t;
#endif

class GtsamFilter
{
public:
    GtsamFilter();

    bool initialise(unsigned nranges_to_use=40, std::vector<double> Z=std::vector<double>(1,0.0));
    void setInit(gtsam::BedPose p, gtsam::Vector priorSigmas = gtsam::Vector4(1,1,0.1,M_PI/4));
    void setSuggestedInit(gtsam::BedPose p, gtsam::Vector priorSigmas = gtsam::Vector4(1,1,0.1,M_PI/4));

    bool addOdometry(const mi_odometry_t &odo);
    bool addRange(lps_range_t r);
    bool addTags(std::vector<gtsam::AnchorPoint> &anchors);
    bool addMap(SvgMap &map);
    void addZlevel(double z);
    void clearZlevels() {_init_z_levels.clear();}

    double getRangeError(lps_range_t r);

    void addNewFactorsToGraph(bool force=false);
    bool update(bool force=false);
    void postUpdate();


    void plot(bool pause=true);
    void plot3d(bool pause=true);
    gtsam::BedPose currentPose();
    double currentError() { return _last_error; }
    gtsam::Matrix currentCovariance() {return _last_covariance;}
    bool isInitialized() const { return _initialized; }

    void restart();
private:
    support_check_t checkSupport(gtsam::BedPose &pose, double threshold);

    gtsam::BedPose _body_p_anchor0;
    gtsam::BedPose _pose0;
    gtsam::BedPose _last_pose;
    gtsam::Matrix  _last_covariance;
    gtsam::BedPose _last_updated_pose;
    gtsam::Matrix  _last_updated_covariance;
    std::vector<gtsam::AnchorPoint> _tags;
    std::vector<bool> _tags_added_to_graph;
    bool _initialized;
    SvgMap _map;

    std::deque<mi_odometry_t> _new_odo_factors;
    unsigned _new_odofactors_since_update;
    unsigned _total_odofactors_added;

    std::deque<lps_range_t> _new_rangefactors;
    unsigned _new_rangefactors_since_update;
    unsigned _total_rangefactors_added;

    std::deque<mi_odometry_t> _init_odofactors;
    std::deque<lps_range_t> _init_rangefactors;

    gtsam::ISAM2 *_isam;
    unsigned _current_step;
    std::vector<double> _step_time; 
    double _last_error;
    double _current_error;

    gtsam::NonlinearFactorGraph _newFactors;
    gtsam::Values _updated_result;
    gtsam::Values _result;
    gtsam::Values _initial;

    std::vector<double> _init_z_levels;
    gnuplot_ctrl* _gnuplot_handle;

    bool _post_update_needed;
    bool _do_restart;

    gtsam::BedPose _suggested_pose;
    gtsam::Vector _suggested_prior;
    bool _have_suggested_pose;
};

#endif // GTSAMFILTER_H

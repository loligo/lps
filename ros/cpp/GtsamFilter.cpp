/**
 * @file RangeISAMExample_plaza1.cpp
 * @brief A 2D Range SLAM example
 * @date June 20, 2013
 * @author FRank Dellaert
 */

// Both relative poses and recovered trajectory poses will be stored as Pose2 objects
#include <gtsam/geometry/Pose2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the range-SLAM problem incrementally
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// We will use a non-liear solver to batch-inituialize from the first 150 frames
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>

// Plotting 
#include "slam/gnuplot_i.h"

// Options and log
#include <core/Options.h>
#include <core/Log.h>
#include <slam/Ransac.h>

#include "GtsamFilter.h"


#include <stdio.h>
#include <termios.h>
#include <unistd.h>

using namespace gtsam;

// Constants
const double location_init_circle_threshold = 0.01;
const int min_rangefactors_for_init = 4;
const int min_rangefactors_for_update = 2;
const int min_odofactors_for_init = 1;
const int prior_z_lockdown_interval = 50;
const int max_ranges_added_before_restart = 400;
const double location_init_inlier_th = 0.075;

// How certain we are about the location of the anchors and their bias
const Vector lgt_tagsPriorSigmas = Vector4(0.01,0.01,0.05,0.5);  
const Vector lgt_odoSigmas = Vector4(1e-1, 1e-1, 1e-3, 0.1);
const double lgt_sigmaR = 0.20; // range standard deviation
const bool lgt_robust = true;

const NM::Base::shared_ptr // all same type
    tagsPriorNoise = NM::Diagonal::Sigmas(lgt_tagsPriorSigmas), //prior for anchors
    odoNoise = NM::Diagonal::Sigmas(lgt_odoSigmas), // odometry
    lgt_gaussian = NM::Isotropic::Sigma(1, lgt_sigmaR), // non-robust
    lgt_tukey = NM::Robust::Create(NM::mEstimator::Tukey::Create(15), lgt_gaussian), //robust
    rangeNoise = lgt_robust ? lgt_tukey : lgt_gaussian;


GtsamFilter::GtsamFilter() : _body_p_anchor0(0,0,0,0), 
    _initialized(false), _new_rangefactors_since_update(0), _total_rangefactors_added(0),
    _current_step(0), _last_error(0.0), _current_error(0.0), _init_z_levels(vector<double>(1,0.0)), _gnuplot_handle(0),
    _post_update_needed(false), _do_restart(false), _have_suggested_pose(false)
{
    // Constructor
    _gnuplot_handle = gnuplot_init() ;

    _isam = new gtsam::ISAM2();

    _last_updated_pose = _last_pose = BedPose(0,0,0,0);
    _last_updated_covariance = gtsam::Matrix(4,4);
    for (unsigned j=0;j<4;j++) 
        for (unsigned i=0;i<4;i++)
            _last_updated_covariance(j,i)=30.0*30.0;
    _last_covariance = _last_updated_covariance;
}

bool GtsamFilter::initialise(unsigned nranges_to_use, vector<double> Z)
{
    Vector priorSigmas = Vector4(0.5,0.5,0.05,M_PI/8);

    // Use the last two measurements from different tags
    deque<lps_range_t>::reverse_iterator mi = _init_rangefactors.rbegin();
    if (mi == _init_rangefactors.rend()) return false;
    
    int c1_tagid=mi->tag_id;
    double r1=mi->r;
    AnchorPoint c1(_tags[mi->tag_id]);
    Log::print(LOG_DEBUG, "GtsamFilter::initialise First range from tag %d p(%.1f,%.1f,%.1f) r%.3f\n", 
               c1_tagid, c1.x(),c1.y(),c1.z(),r1);

    // Take next measurement that is not from the same tag
    mi++;
    while (mi!= _init_rangefactors.rend() && mi->tag_id == c1_tagid) mi++;
    if (mi == _init_rangefactors.rend()) return false;

    AnchorPoint c2(_tags[mi->tag_id]);
    int c2_tagid=mi->tag_id;
    double r2=mi->r;
    Log::print(LOG_DEBUG, "GtsamFilter::initialise Second range from tag %d p(%.1f,%.1f,%.1f) r%.3f\n", 
               c2_tagid, c2.x(),c2.y(),c2.z(),r2);
    double tol=0.8;
    list<Point3> solutions=AnchorPoint::CircleCircleIntersection(c1, r1, c2, r2, 1.70, tol);
    Log::print(LOG_DEBUG, "GtsamFilter::initialise %d intersections\n", solutions.size());

    // Take next measurement that is not from either of the first tags
    mi++;
    while (mi!= _init_rangefactors.rend() && (mi->tag_id == c1_tagid || mi->tag_id == c2_tagid)) mi++;
    if (mi == _init_rangefactors.rend()) return false;
    
    // determine real solution by comparing to third available tag
    AnchorPoint c3 = _tags[mi->tag_id];
    double r3 = mi->r;
    Log::print(LOG_DEBUG, "GtsamFilter::initialise Third range from tag %d p(%.1f,%.1f,%.1f) r%.3f\n", 
               mi->tag_id, c3.x(),c3.y(),c3.z(),r3);
    BedPose best_solution;
    double best_err = std::nan("");
    for (list<Point3>::iterator ii=solutions.begin();ii!=solutions.end();ii++)
    {
        double err = fabs(c3.distance(*ii)-r3);
        if (err < tol && (err < best_err || ::isnan(best_err)))
        {
            best_solution = BedPose((*ii).x(),(*ii).y(),(*ii).z(),0);
            best_err = err;
        }
    }
    if (::isnan(best_err))
    {
        Log::print(LOG_INFO, "GtsamFilter::initialise location is NAN\n"); 
        return false;
    }

    Log::print(LOG_VERBOSE, "GtsamFilter::initialise new location (%.3f,%.3f,%.3f) err=%.3f\n", 
               best_solution.x(),best_solution.y(),best_solution.z(), best_err);
    
    for(unsigned i=0;i<_init_rangefactors.size();i++)
    {
        size_t tag = _init_rangefactors[i].tag_id;
        size_t anchor = _init_rangefactors[i].anchor_id;
        double range = _init_rangefactors[i].r;
        RangeFactor<BedPose, AnchorPoint> rf = RangeFactor<BedPose, AnchorPoint>(0, 1, range, NM::Isotropic::Sigma(1, 1.0), _body_p_anchor0);
        if (tag >= _tags.size())
        {
            Log::print(LOG_ERROR, "GtsamFilter::init - tag id out of range (%d > %zd)\n", tag, _tags.size());
            return false;
        }
        
        Vector error = rf.evaluateError(best_solution, _tags[tag]);
        double thiserror = error.norm();
        Log::print(LOG_DEBUG, " res: t:%d a:%d range:%.3f error:%.3f\n", tag, anchor, range, thiserror);

        // Only include inliers in the first optimisation
        if(thiserror < location_init_inlier_th)
            _new_rangefactors.push_back(_init_rangefactors[i]);
    }

    setInit(best_solution, priorSigmas);
    return true;
}

void GtsamFilter::setInit(BedPose p, Vector priorSigmas)
{
    _pose0 = p;
    _last_pose = _pose0;
    _last_updated_pose = _pose0;
    //_last_covariance = NM::Diagonal::Sigmas(priorSigmas));
    //_last_updated_covariance = NM::Diagonal::Sigmas(priorSigmas))
    _initial.insert(0, _pose0);
    _initialized = true;
    _step_time.clear();
    if (_new_odo_factors.size())
        _step_time.push_back(_new_odo_factors.front().t);
    Log::print(LOG_INFO, "GtsamFilter::setInit() Pose0: (%.2f,%.2f,%.2f,%.2f(=%.2fdeg))\n", _pose0.x(), _pose0.y(), _pose0.z(), _pose0.theta(), _pose0.theta()*180.0/M_PI);
    _newFactors.push_back(PriorFactor<BedPose>(0, _pose0, NM::Diagonal::Sigmas(priorSigmas)));
}

support_check_t GtsamFilter::checkSupport(BedPose &pose, double threshold)
{
    support_check_t support;
    support.inliers = 0;
    support.error = 0.0;
    for(unsigned i=0;i<_init_rangefactors.size();i++)
    {
        size_t tag = _init_rangefactors[i].tag_id;
        size_t anchor = _init_rangefactors[i].anchor_id;
        double range = _init_rangefactors[i].r;
        RangeFactor<BedPose, AnchorPoint> rf;
        rf = RangeFactor<BedPose, AnchorPoint>(0, 1, range, NM::Isotropic::Sigma(1, 1.0), _body_p_anchor0);
        if (tag >= _tags.size())
        {
            Log::print(LOG_ERROR, "GtsamFilter::checkSupport - tag id out of range (%d > %zd)\n", tag, _tags.size());
            support.inliers=0;
            return support;
        }
        
        Vector error = rf.evaluateError(pose, _tags[tag]);
        double thiserror = error.norm();
        Log::print(LOG_VERBOSE, "GtsamFilter::checkSupport[%d] tag=%d anchor=%d error=%f\n", i, tag, anchor, thiserror);
        if(thiserror < threshold) // inlier
        {
            support.inliers++;
            support.error += thiserror;
            support.inlier_index.push_back(i);
        }
    }
    return support;
}

void GtsamFilter::setSuggestedInit(BedPose p, Vector priorSigmas)
{
    // Set the other factors as well to avoid pushing
    // false (zeroes) to database
    _pose0 = p;
    _last_pose = _pose0;
    _last_updated_pose = _pose0;

    for (unsigned j=0;j<4;j++) 
        for (unsigned i=0;i<4;i++)
            _last_updated_covariance(j,i)=30.0*30.0;

    _last_covariance = _last_updated_covariance;

    _suggested_pose = p;
    Log::print(LOG_INFO, "GtsamFilter::setSuggestedInit() Pose: (%.2f,%.2f,%.2f,%.2f(=%.2fdeg))\n", 
               p.x(), p.y(), p.z(), p.theta(), p.theta()*180.0/M_PI);
    _suggested_prior = priorSigmas;
    _have_suggested_pose = true;
}


bool GtsamFilter::addTags(vector<AnchorPoint> &anchors)
{
    _tags.clear();
    vector<AnchorPoint>::iterator ii;
    for (ii=anchors.begin();ii!=anchors.end();ii++)
    {
        Log::print(LOG_VERBOSE, "%s:addAnchors [0x%.2x]@(%.1f,%.1f,%.1f)\n", __FILE__, _tags.size(), ii->x(),ii->y(),ii->z());
        _tags.push_back(*ii);
    }
    _tags_added_to_graph.resize(_tags.size(),0);
    return true;
}

bool GtsamFilter::addMap(SvgMap &map)
{
    _map = map;
    _tags.clear();
    vector<anchor_t> map_a = _map.getAnchors();
    for (unsigned i=0;i<map_a.size();i++)
    {
        if ((map_a[i].id+1) > (int)_tags.size()) _tags.resize(map_a[i].id+1);
        Log::print(LOG_VERBOSE, "%s:addMap: tags[0x%.2x]@(%.2f,%.2f,%.2f)\n", 
                   __FILE__, map_a[i].id, map_a[i].p.x(), map_a[i].p.y(), map_a[i].p.z());
        _tags[map_a[i].id] = AnchorPoint(map_a[i].p.x(), map_a[i].p.y(), map_a[i].p.z());
    }
    _tags_added_to_graph.resize(_tags.size(),0);

    // Add all Z levels from map
    clearZlevels();
    std::vector<std::list<gtsam::Point3> > floor = _map.getFloorSpace();
    for (unsigned i=0;i<floor.size();i++)
    {
        for (std::list<gtsam::Point3>::iterator ii=floor[i].begin();ii!=floor[i].end();ii++)
            addZlevel(ii->z());
    }

    return true;
}

void GtsamFilter::addZlevel(double z) 
{
    if (std::find(_init_z_levels.begin(),_init_z_levels.end(), z) != _init_z_levels.end()) return;


    for (unsigned i=0;i<_init_z_levels.size();i++)
    {
        if (fabs(z-_init_z_levels[i]) < 0.1) 
            return;
    }

    Log::print(LOG_INFO, "GtsamFilter::addZlevel: %.2fm\n", z);
    _init_z_levels.push_back(z);
    std::sort(_init_z_levels.begin(),_init_z_levels.end());
}


bool GtsamFilter::addOdometry(const mi_odometry_t &odometry)
{
    _init_odofactors.push_back(odometry);
    _new_odo_factors.push_back(odometry);
    _total_odofactors_added++;

    return true;
}

bool GtsamFilter::addRange(lps_range_t r)
{
    _init_rangefactors.push_back(r);
    _total_rangefactors_added++;

    if (_total_rangefactors_added > max_ranges_added_before_restart) 
        _do_restart = true;

    if (_initialized)
        _new_rangefactors.push_back(r);

    Log::print(LOG_SPAM, "GtsamFilter::addRange: %.2x->%.2x tot:%d\n", r.tag_id, r.anchor_id, _total_rangefactors_added);
    return true;
}


double GtsamFilter::getRangeError(lps_range_t r)
{
    size_t tag    = r.tag_id;
    size_t anchor = r.anchor_id;
    double mrange  = r.t;
    
    if (tag >= _tags.size())
    {
        Log::print(LOG_ERROR, "GtsamFilter::update - tag id out of range (%d > %zd)\n", anchor, _tags.size());
        return nan("");
    }

    BedPose pp = _last_pose;
    double erange = pp.range(_tags[tag]);


    return mrange-erange;
}


BedPose GtsamFilter::currentPose() 
{
    BedPose pp = _last_pose;

    for (deque<mi_odometry_t>::iterator oi=_new_odo_factors.begin();oi!=_new_odo_factors.end();oi++)
    {
        BedPose odo(oi->s,0.0,oi->s*sin(oi->phi),oi->theta);
        pp = pp.compose(odo);
    }

    return pp;
}


void GtsamFilter::addNewFactorsToGraph(bool force)
{
    if (!_initialized && !force) return;

    // Add all new available data to the graph
    while (_new_odo_factors.size()) 
    {
        // Increment step counter
        _current_step++;
        
        _step_time.resize(_current_step+1);

        // add odometry factor
        mi_odometry_t oi = _new_odo_factors.front();
        _new_odo_factors.pop_front();
        double t = oi.t;
        _step_time[_current_step] = t;
        BedPose odo(oi.s,0.0,oi.s*sin(oi.phi),oi.theta);

        _newFactors.push_back(BetweenFactor<BedPose>(_current_step-1, _current_step, odo, NM::Diagonal::Sigmas(lgt_odoSigmas)));
        Log::print(LOG_SPAM, "GtsamFilter::addNewFactorsToGraph: Odometryfactor@t=%.3f [%.4f %.4f %.4f %.4f]\n", 
                   t, odo.x(), odo.y(), odo.z(), odo.theta());
        
        // predict pose and add as initial estimate
        BedPose pp = _last_updated_pose.compose(odo);
        Log::print(LOG_SPAM, "GtsamFilter::addNewFactorsToGraph: Odometryfactor@t=%.3f predictedPose=[%.3f %.3f %.3f %.3f]\n", 
                   t, pp.x(), pp.y(), pp.z(), pp.theta());
        _last_updated_pose = pp;
        _initial.insert(_current_step, pp);

        // Add all available range factors to the graph
        // Check if there are range factors to be added
        while (_new_rangefactors.size() && t >= _new_rangefactors.front().t) 
        {

            size_t tag    = _new_rangefactors.front().tag_id;
            size_t anchor = _new_rangefactors.front().anchor_id;
            double range  = _new_rangefactors.front().r;
            
            if (tag >= _tags.size())
            {
                Log::print(LOG_ERROR, "GtsamFilter::addNewFactorsToGraph: - tag id out of range (%d > %zd)\n", tag, _tags.size());
                _new_rangefactors.pop_front();
                continue;
            }

            Log::print(LOG_SPAM, "GtsamFilter::addNewFactorsToGraph: Rangefactor@t=%.3f tag %.2x to anchor %.2x %.3fm\n", 
                       t, tag, anchor, range);
            
            _newFactors.push_back(RangeFactor<BedPose, AnchorPoint>(_current_step, symbol('L', tag), range, rangeNoise, _body_p_anchor0));
            
            // Add initial location and uncertainty of the anchors (this is done as the robot first sees them)
            if (!_initial.exists(symbol('L', tag)) && !_tags_added_to_graph[tag]) 
            {
                _newFactors.push_back(PriorFactor<AnchorPoint>(symbol('L', tag), _tags[tag], tagsPriorNoise));
                _initial.insert(symbol('L', tag), _tags[tag]);
                _tags_added_to_graph[tag]=true;
            }
            _new_rangefactors_since_update++;
            _new_rangefactors.pop_front();
        }

        // Force bed to "stay on the floor"
        if (_current_step%prior_z_lockdown_interval == 0)
        {
            // Find most likely z from map
            float z = 1.70; 
            bool b = _map.getZofPoint(_last_updated_pose.x(), _last_updated_pose.y(), z);

            // Add as a prior but only allow it to impact Z
            if (b)
            {
                Vector priorSigmas = Vector4(500,500,0.05,500);
                BedPose p(_last_updated_pose.x(), _last_updated_pose.y(), z ,_last_updated_pose.theta());
                _newFactors.push_back(PriorFactor<BedPose>(_current_step, p, NM::Diagonal::Sigmas(priorSigmas)));
                Log::print(LOG_DEBUG, "GtsamFilter::addNewFactorsToGraph: Setting z prior to %.2f\n", z);
            }

        }
    }

    // Keep a copy of the last bunch of factors if we need to reinit
    while (_init_rangefactors.size() > (unsigned)min_rangefactors_for_init) _init_rangefactors.pop_front();
    while (_init_odofactors.size() > (unsigned)min_odofactors_for_init) _init_odofactors.pop_front();
}


bool GtsamFilter::update(bool force)
{
    if (_do_restart)
    {
        delete _isam;
        _isam = new gtsam::ISAM2();
        _tags_added_to_graph.resize(0);
        _tags_added_to_graph.resize(_tags.size(),0);
        _newFactors = NonlinearFactorGraph();
        _initial = Values();
        _result = Values();
        _current_step = 0;
        _step_time.resize(100,0);
        _total_rangefactors_added = _init_rangefactors.size() + 1;

        setSuggestedInit(_last_pose, Vector4(1,1,0.05,0.1)); 
        _initialized = false; // to force init bundling
        _new_odo_factors.insert(_new_odo_factors.begin(), _init_odofactors.begin(), _init_odofactors.end());
        _new_rangefactors.insert(_new_rangefactors.begin(), _init_rangefactors.begin(), _init_rangefactors.end());
    }        


    // Check whether to update iSAM 2
    if (force || ((_total_rangefactors_added > (unsigned)min_rangefactors_for_init) && 
                  ((_new_rangefactors_since_update > (unsigned)min_rangefactors_for_update) || !_initialized))) 
    {
        Log::print(LOG_VERBOSE, "GtsamFilter::update() Performing update\n");
        //_result.print("\nLast estimate:\n"); 
        //_newFactors.print("\nFactor Graph:\n"); 
        if (!_initialized) 
        { 
            bool ret =false;
            if (_have_suggested_pose)
            {
                Log::print(LOG_INFO, "GtsamFilter::update() Evaluating suggested pose\n");
                support_check_t support = checkSupport(_suggested_pose, 1.0);
                Log::print(LOG_VERBOSE, "GtsamFilter::update() Suggested pose data: (%d,%f), rangefactors=(actual%d,min%d)\n", 
                           support.inliers, support.error, _init_rangefactors.size(), min_rangefactors_for_init);
                if (support.inliers > 5 && 
                    (support.inliers > _init_rangefactors.size()/5 || support.inliers > min_rangefactors_for_init/5))
                {
                    setInit(_suggested_pose, _suggested_prior);
                    _have_suggested_pose = false;
                    ret = true;
                }
            }
            
            if (!ret)
                ret = initialise(min_rangefactors_for_init, _init_z_levels);

            if (!ret) 
            {
                Log::print(LOG_INFO, "GtsamFilter::update() Failed initialization at step %d, retry next step\n", _current_step);
                return false;
            }

            // Force adding in spite of not having init
            addNewFactorsToGraph(true);

            // Do a full optimize for first minK ranges
            try {
                gttic_(batchInitialization);
                LevenbergMarquardtOptimizer batchOptimizer(_newFactors, _initial);
                Log::print(LOG_VERBOSE, "GtsamFilter::update() Init.optim\n");
                _initial = batchOptimizer.optimize();
                _initialized = true;
                _do_restart = false;
                gttoc_(batchInitialization);
                Log::print(LOG_VERBOSE, "GtsamFilter::update() Performed initialization at step %d\n", _current_step);
                BedPose pp = _initial.at<BedPose>( _current_step );
                Log::print(LOG_VERBOSE, "GtsamFilter::update(): init post LM pose=[%.3f %.3f %.3f %.3f]\n", 
                           pp.x(), pp.y(), pp.z(), pp.theta());
            }
            catch (const std::invalid_argument& ia)
            {
                std::cerr << "Invalid argument: " << ia.what() << '\n';
            }
            catch (const gtsam::InvalidMatrixBlock& imb)
            {
                std::cerr << "Invalid matrix block: " << imb.what() << '\n';
            }
            catch (const gtsam::IndeterminantLinearSystemException& ilsm)
            {
                _result.print("\nLast estimate:\n"); 
                std::cerr << "IndeterminantLinearSystemException:" << ilsm.what() << '\n';
            }
        }

        gttic_(update);
        //if (_current_step > 1000)
            //std::vector<size_t> indicies_to_remove(_current_step-1000, 0);
        ISAM2Result result = _isam->update(_newFactors, _initial);
        gttoc_(update);
        gttic_(calculateEstimate);
        _updated_result = _isam->calculateEstimate();
        gttoc_(calculateEstimate);

        // const NonlinearFactorGraph& factor_graph = _isam->getFactorsUnsafe();
        // factor_graph.print();

        _last_updated_pose = _updated_result.at<BedPose>(_current_step);
        _newFactors = NonlinearFactorGraph();
        _last_updated_covariance = _isam->marginalCovariance(_current_step);
        _initial = Values();
        _new_rangefactors_since_update = 0;
        _current_error = _isam->error(_isam->getDelta());
        Log::print(LOG_VERBOSE, "GtsamFilter::update() Finished update at step=%d, error=%.3e\n", 
                   _current_step,  
                   _current_error
            ); 
        BedPose pp = _last_updated_pose;
        Log::print(LOG_DEBUG, "GtsamFilter::update(): updated pose=[%.3f %.3f %.3f %.3f] covariance=[%.3f %.3f %.3f %.3f]\n", 
                   pp.x(), pp.y(), pp.z(), pp.theta(), 
                   _last_updated_covariance(0,0),_last_updated_covariance(1,1),_last_updated_covariance(2,2),_last_updated_covariance(3,3));

        // Post update needed
        _post_update_needed = true;
    }

    return true;
}

void GtsamFilter::postUpdate()
{
    // Always update pose (from odo)
    _last_pose = _last_updated_pose;

    if (_post_update_needed)
    {
        _post_update_needed = false;
        _result = _updated_result;
        _last_error = _current_error;
        _last_covariance = _last_updated_covariance;
    }
}



void GtsamFilter::plot(bool pause)
{

    //_result.print("\nPrinted estimate:\n"); 

    int nkeys=0;
    KeyList keys = _result.keys();
    for (KeyList::iterator ii=keys.begin();ii!=keys.end();ii++)
    {
        nkeys++;
    }
    
    gnuplot_resetplot(_gnuplot_handle) ;

    _map.plot(_gnuplot_handle);
    
    double *graphx = (double*)malloc(sizeof(double)*nkeys);
    double *graphy = (double*)malloc(sizeof(double)*nkeys);

    int n=0;
    for(int j=0;j<nkeys;j++)
    {
        if (!_result.exists(j)) continue;
        BedPose x = _result.at<BedPose>(j);
        graphx[j]=x.x();
        graphy[j]=x.y();
        n++;
        //printf("%d %f,%f\n", j, x.x(), x.y());
    }

    gnuplot_setstyle(_gnuplot_handle, (const char*)"lines") ;
    gnuplot_plot_xy(_gnuplot_handle, graphx, graphy, n, (const char*)"bed location");
    free(graphx);free(graphy);

    // Landmarks
    n=0;
    double lx[32];
    double ly[32];
    for (int j=0;j<32;j++)
    {
        lx[j]=ly[j]=nan("");
        if (!_result.exists(symbol('L', j))) continue;
        AnchorPoint lxy = _result.at<AnchorPoint>(symbol('L', j));
        lx[j]=lxy.x();ly[j]=lxy.y();
        n++;
        //printf("lxy: %.3e,%.3e\n", lxy.x(), lxy.y());
    }
    gnuplot_setstyle(_gnuplot_handle, (const char*)"points") ;
    gnuplot_plot_xy(_gnuplot_handle, lx, ly, 32, (const char*)"landmarks") ;

    // Covariance ellipses
    const int ellipsesteps = 16;
    double *ex = (double*)malloc(sizeof(double)*(nkeys*ellipsesteps + nkeys));
    double *ey = (double*)malloc(sizeof(double)*(nkeys*ellipsesteps + nkeys));

    n = 0;
    double kf=2.296; // 68% of all prob (11.82 = 99%)
    for(int j=0;j<nkeys;j+=100)
    {
        if (!_result.exists(j)) continue;
        BedPose x = _result.at<BedPose>(j);
        // printf("#%d:\n", j);
        // Matrix covariance3 = isam.marginalCovariance(j).topLeftCorner(3,3);
        // cout << covariance3 << endl;
        Matrix covariance = _isam->marginalCovariance(j).topLeftCorner(2,2);
        Matrix U, V;
        Vector S;
        svd(covariance, U, S, V);
        Vector a = sqrt(S[0]*kf)*V.col(0);
        Vector b = sqrt(S[0]*kf)*V.col(1);
        for (int k=0;k<ellipsesteps;k++)
        {
            Vector2 v(x.x(), x.y());
            v += a*cos(2*k*M_PI/(ellipsesteps-1)) + b*sin(2*k*M_PI/(ellipsesteps-1));
            ex[n] = v[0];
            ey[n++] = v[1];
        }
        ex[n] = nan("");
        ey[n++] = nan("");
    }    
    for(int j=0;j<32;j++)
    {
        if (!_result.exists(symbol('L', j))) continue;
        AnchorPoint x = _result.at<AnchorPoint>(symbol('L', j));
        Matrix covariance = _isam->marginalCovariance(symbol('L', j)).topLeftCorner(2,2);
        Matrix U, V;
        Vector S;
        svd(covariance, U, S, V);
        Vector a = sqrt(S[0]*kf)*V.col(0);
        Vector b = sqrt(S[0]*kf)*V.col(1);
        for (int k=0;k<ellipsesteps;k++)
        {
            Vector2 v(x.x(), x.y());
            v += a*cos(2*k*M_PI/(ellipsesteps-1)) + b*sin(2*k*M_PI/(ellipsesteps-1));
            ex[n] = v[0];
            ey[n++] = v[1];
        }
        ex[n] = nan("");
        ey[n++] = nan("");
    }    

    gnuplot_setstyle(_gnuplot_handle, "lines") ;
    gnuplot_cmd(_gnuplot_handle, "set size ratio -1");
    gnuplot_cmd(_gnuplot_handle, "set grid ytics xtics");
    gnuplot_cmd(_gnuplot_handle, "set grid");
    gnuplot_plot_xy(_gnuplot_handle, ex, ey, n, "covariance") ;
    free(ex);free(ey);

    if (pause)
    {
        printf("press ENTER to continue\n");
        while (getchar()!='\n') {}
        gnuplot_close(_gnuplot_handle);
    }
}

void GtsamFilter::restart()
{
    _do_restart = true;
}

void GtsamFilter::plot3d(bool pause)
{
    //_result.print("\nPrinted estimate:\n"); 

    int nkeys=0;
    KeyList keys = _result.keys();
    for (KeyList::iterator ii=keys.begin();ii!=keys.end();ii++)
    {
        nkeys++;
    }

    gnuplot_resetplot(_gnuplot_handle) ;

    _map.plot3d(_gnuplot_handle);
    
    double *graphx = (double*)malloc(sizeof(double)*nkeys);
    double *graphy = (double*)malloc(sizeof(double)*nkeys);
    double *graphz = (double*)malloc(sizeof(double)*nkeys);

    int n=0;
    for(int j=0;j<nkeys;j++)
    {
        if (!_result.exists(j)) continue;
        BedPose x = _result.at<BedPose>(j);
        graphx[j]=x.x();
        graphy[j]=x.y();
        graphz[j]=x.z();
        n++;
        //printf("%d %f,%f\n", j, x.x(), x.y());
    }

    gnuplot_setstyle(_gnuplot_handle, (const char*)"lines") ;
    gnuplot_plot_xyz(_gnuplot_handle, graphx, graphy, graphz, n, (const char*)"bed location");
    free(graphx);free(graphy);free(graphz);

    // Landmarks
    n=0;
    double lx[32];
    double ly[32];
    double lz[32];
    for (int j=0;j<32;j++)
    {
        lx[j]=ly[j]=nan("");
        if (!_result.exists(symbol('L', j))) continue;
        AnchorPoint lxy = _result.at<AnchorPoint>(symbol('L', j));
        lx[j]=lxy.x();ly[j]=lxy.y();lz[j]=lxy.z();
        n++;
        //printf("lxy: %.3e,%.3e\n", lxy.x(), lxy.y());
    }
    gnuplot_setstyle(_gnuplot_handle, (const char*)"points") ;
    gnuplot_plot_xyz(_gnuplot_handle, lx, ly, lz, 32, (const char*)"landmarks") ;

#if 0
    // Covariance ellipses
    const int ellipsesteps = 16;
    double *ex = (double*)malloc(sizeof(double)*(nkeys*ellipsesteps + nkeys));
    double *ey = (double*)malloc(sizeof(double)*(nkeys*ellipsesteps + nkeys));
    double *ez = (double*)malloc(sizeof(double)*(nkeys*ellipsesteps + nkeys));

    n = 0;
    double kf=2.296; // 68% of all prob (11.82 = 99%)
    for(int j=0;j<nkeys;j+=100)
    {
        if (!_result.exists(j)) continue;
        BedPose x = _result.at<BedPose>(j);
        // printf("#%d:\n", j);
        // Matrix covariance3 = isam.marginalCovariance(j).topLeftCorner(3,3);
        // cout << covariance3 << endl;
        Matrix covariance = _isam->marginalCovariance(j).topLeftCorner(2,2);
        Matrix U, V;
        Vector S;
        svd(covariance, U, S, V);
        Vector a = sqrt(S[0]*kf)*V.col(0);
        Vector b = sqrt(S[0]*kf)*V.col(1);
        for (int k=0;k<ellipsesteps;k++)
        {
            Vector2 v(x.x(), x.y());
            v += a*cos(2*k*M_PI/(ellipsesteps-1)) + b*sin(2*k*M_PI/(ellipsesteps-1));
            ex[n] = v[0];
            ey[n] = v[1];
            ez[n++] = x.z();
        }
        ex[n] = nan("");
        ey[n] = nan("");
        ez[n++] = nan("");
    }    
    for(int j=0;j<32;j++)
    {
        if (!_result.exists(symbol('L', j))) continue;
        AnchorPoint x = _result.at<AnchorPoint>(symbol('L', j));
        Matrix covariance = _isam->marginalCovariance(symbol('L', j)).topLeftCorner(2,2);
        Matrix U, V;
        Vector S;
        svd(covariance, U, S, V);
        Vector a = sqrt(S[0]*kf)*V.col(0);
        Vector b = sqrt(S[0]*kf)*V.col(1);
        for (int k=0;k<ellipsesteps;k++)
        {
            Vector2 v(x.x(), x.y());
            v += a*cos(2*k*M_PI/(ellipsesteps-1)) + b*sin(2*k*M_PI/(ellipsesteps-1));
            ex[n] = v[0];
            ey[n] = v[1];
            ez[n++] = x.z();
        }
        ex[n] = nan("");
        ey[n] = nan("");
        ez[n++] = nan("");
    }    

    gnuplot_setstyle(_gnuplot_handle, "lines") ;
    gnuplot_cmd(_gnuplot_handle, "set size ratio -1");
    gnuplot_cmd(_gnuplot_handle, "set grid ytics xtics");
    gnuplot_cmd(_gnuplot_handle, "set grid");
    gnuplot_plot_xyz(_gnuplot_handle, ex, ey, ez, n, "covariance") ;
    free(ex);free(ey);free(ez);
#endif

    if (pause)
    {
        printf("press ENTER to continue\n");
        while (getchar()!='\n') {}
        gnuplot_close(_gnuplot_handle);
    }
}

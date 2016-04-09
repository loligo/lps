#ifndef MOBILE_ANCHOR_H
#define MOBILE_ANCHOR_H

#include <gtsam/geometry/Point3.h>
#include <loligo/slam/SvgMap.h>
#include "GtsamFilter.h"

using namespace gtsam;

class MobileAnchor
{
public:
    MobileAnchor() : _x(0,0,0,0), _mag_min(1e10,1e10,1e10), _mag_max(-1e10,-1e10,-1e10), _mag_bias(0,0,0)
    {
        _location.addZlevel(1.7);
    }

    double x() {return _x.x();}
    double y() {return _x.y();}
    double z() {return _x.z();}
    double theta() {Point3 v=this->m();return atan2(v.z(),v.x());}

    void setMagnetometer(Point3 p);
    
    Point3 m();

    void addMap(SvgMap &m);

    bool addOdometry(const mi_odometry_t &odo) { return _location.addOdometry(odo); }
    void addRange(lps_range_t &r)
    {
        _location.addRange(r);
    }
    void update();

    gtsam::BedPose currentPose();

    gtsam::Matrix currentCovariance();
    bool addTags(std::vector<gtsam::AnchorPoint> &anchors) { return _location.addTags(anchors); }

private:
    BedPose _x;
    Point3 _m;
    Vector3 _mag_min;
    Vector3 _mag_max;
    Vector3 _mag_bias;
    Vector3 _mag_scale;

    GtsamFilter _location;
};


#endif

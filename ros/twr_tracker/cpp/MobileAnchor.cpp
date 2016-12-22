#include "MobileAnchor.h"

#include <loligo/core/Log.h>

#include <loligo/db/MysqlWrap.h>
#include <loligo/slam/SvgMap.h>
#include <loligo/slam/AnchorPoint.h>
#include <gtsam/geometry/Point3.h>

void MobileAnchor::setMagnetometer(Point3 p) 
{
    if (p.x()==0 && p.y()==0 && p.z()==0) return;
    _m=p;
    if (_m.x() < _mag_min(0)) _mag_min(0) = _m.x();
    if (_m.x() > _mag_max(0)) _mag_max(0) = _m.x();
    if (_m.y() < _mag_min(1)) _mag_min(1) = _m.y();
    if (_m.x() > _mag_max(1)) _mag_max(1) = _m.y();
    if (_m.z() < _mag_min(2)) _mag_min(2) = _m.z();
    if (_m.z() > _mag_max(2)) _mag_max(2) = _m.z();
    Log::print(LOG_SPAM, "mag min(%f,%f,%f) max(%f,%f,%f)\n", 
               _mag_min(0), _mag_min(1), _mag_min(2), 
               _mag_max(0), _mag_max(1), _mag_max(2));
    _mag_bias = (_mag_max + _mag_min)/2;
    Vector3 average = (_mag_max - _mag_min)/2;
    for (int i=0;i<3;++i) _mag_scale(i) = average.sum()/3.0/average(i);
}
    
Point3 MobileAnchor::m()
{
    Vector3 v = _m.vector() - _mag_bias;
    for (int i=0;i<3;++i) v(i) = v(i)*_mag_scale(i);
    return Point3(v);
}

void MobileAnchor::addMap(SvgMap &m)
{
    _location.addMap(m);
}

gtsam::BedPose MobileAnchor::currentPose() 
{
    return _location.currentPose();
}

gtsam::Matrix MobileAnchor::currentCovariance() 
{
    return _location.currentCovariance();
}

void MobileAnchor::update()
{
    _location.addNewFactorsToGraph();
    _location.update();
    _location.postUpdate();
    _x = _location.currentPose();
}

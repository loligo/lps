#include <thread>
#include <mutex>
#include <vector>
#include <deque>

#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <signal.h>
#include <semaphore.h>

// ROS
#include "ros/ros.h"
#include <lps/LPSRange.h>

// System V IPC
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>		//Used for shared memory

union semun {
    int              val;    /* Value for SETVAL */
    struct semid_ds *buf;    /* Buffer for IPC_STAT, IPC_SET */
    unsigned short  *array;  /* Array for GETALL, SETALL */
    struct seminfo  *__buf;  /* Buffer for IPC_INFO
                                (Linux-specific) */
};

#include <math.h>
#include <Options.h>
#include <Log.h>
#include <StringUtils.h>

#include <loligo/db/MysqlWrap.h>
#include <loligo/slam/SvgMap.h>
#include <loligo/slam/AnchorPoint.h>
#include <gtsam/geometry/Point3.h>

#define SHMOBJ_PATH         "/shmjeshu"

// System V IPC
#define	SEMAPHORE_KEY			0x282828 //Semaphore unique key
#define	SHARED_MEMORY_KEY 		0x292929 //Shared memory unique key

using namespace gtsam;

sem_t * sem_id;

/* message structure for messages in the shared segment */

struct shared_data {
    char json[2048];
};

MysqlWrap *_mw = 0;


static struct Option options[] = 
{
    { "log-files",           OpType(OP_REQ),    "stdout",
      "comma separated list of files to send logging output"
    },
    { "log-level",           OpType(OP_REQ),    "verbose",
      "output messages up to this level"
    },
    { "map=<filename>",       OpType(OP_REQ),   "",
      "svg file to load map from"
    },
    { "help",                OpType(OP_NON),    "",       
      "-show help message"},
    { "", 0, "", ""}
};

class MobileAnchor
{
public:
    MobileAnchor() : _mag_min(1e10,1e10,1e10), _mag_max(-1e10,-1e10,-1e10), _mag_bias(0,0,0)
    {
    }

    double x() {return _x.x();}
    double y() {return _x.y();}
    double z() {return _x.z();}
    double theta() {Point3 v=this->m();return atan2(v.z(),v.x());}

    void setLocation(Point3 &p) {_x=p;}
    void setMagnetometer(Point3 p) 
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
    
    Point3 m()
    {
        Vector3 v = _m.vector() - _mag_bias;
        for (int i=0;i<3;++i) v(i) = v(i)*_mag_scale(i);
        return Point3(v);
    }

private:
    Point3 _x;
    Point3 _m;
    Vector3 _mag_min;
    Vector3 _mag_max;
    Vector3 _mag_bias;
    Vector3 _mag_scale;
};

typedef struct {
    double t; // timestamp
    float r; // range in meters
    uint16_t tag_id;
    uint16_t anchor_id;
} lps_range_t;


// Structures
vector<deque<lps_range_t> > _anchor_ranges;
vector<MobileAnchor> _anchors;
vector<Point3> _tags;
SvgMap _map;
struct shared_data *_shared_msg;      /* the shared segment, and head of the messages list */

// System V IPC
int _semaphore1_id;
struct shared_data *_shared_memory1;
void* _shared_memory1_pointer;
int _shared_memory1_id;

    
void addMap(SvgMap &map)
{
    _map = map;
    _tags.clear();
    vector<anchor_t> map_a = _map.getAnchors();
    for (unsigned i=0;i<map_a.size();i++)
    {
        if ((map_a[i].id+1) > (int)_tags.size()) _tags.resize(map_a[i].id+1);
        Log::print(LOG_VERBOSE, "%s:addMap: Tag[0x%.2x]@(%.2f,%.2f,%.2f)\n", 
                   __FILE__, map_a[i].id, map_a[i].p.x(), map_a[i].p.y(), map_a[i].p.z());
        _tags[map_a[i].id] = Point3(map_a[i].p.x(), map_a[i].p.y(), map_a[i].p.z());
    }
}

void init_posix_shm()
{
    int shared_seg_size = (1 * sizeof(struct shared_data));   /* want shared segment capable of storing 1 message */
    
    /* creating the shared memory object    --  shm_open()  */
    int shmfd = shm_open(SHMOBJ_PATH, O_CREAT | O_RDWR, S_IRWXU | S_IRWXG);
    if (shmfd < 0)
    {
        perror("In shm_open()");
        exit(1);
    }
    
    ROS_INFO("Created shared memory object %s\n", SHMOBJ_PATH);
    
    /* adjusting mapped file size (make room for the whole segment to map)      --  ftruncate() */
    ftruncate(shmfd, shared_seg_size);
    
    /**
     * Semaphore open
     */
    sem_id=sem_open("/mysem", O_CREAT, S_IRUSR | S_IWUSR, 1);
    
    /* requesting the shared segment    --  mmap() */
    _shared_msg = (struct shared_data *)mmap(NULL, shared_seg_size, PROT_READ | PROT_WRITE, MAP_SHARED, shmfd, 0);
    if (_shared_msg == NULL)
    {
        perror("In mmap()");
        exit(1);
    }
};

int semaphore1_get_access(void)
{
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op = -1; /* P() */
    sem_b.sem_flg = SEM_UNDO;
    if (semop(_semaphore1_id, &sem_b, 1) == -1)		//Wait until free
    {
        perror("sem acces");
        ROS_ERROR("semaphore1_get_access failed: id=%d", _semaphore1_id);
        return(0);
    }
    return(1);
}

int semaphore1_release_access(void)
{
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op = 1; /* V() */
    sem_b.sem_flg = SEM_UNDO;
    if (semop(_semaphore1_id, &sem_b, 1) == -1)
    {
        ROS_ERROR("semaphore1_release_access failed\n");
        return(0);
    }
    return(1);
}

void init_systemV_ipc()
{
    _semaphore1_id = semget((key_t)SEMAPHORE_KEY, 3, 0666 | IPC_CREAT);
    if (_semaphore1_id == -1)
    {
        perror("semget");
        ROS_ERROR("Could not create semaphore\n");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("semaphore: %d\n", _semaphore1_id);

    union semun sem_union_init;
    sem_union_init.val = 1;
    if (semctl(_semaphore1_id, 0, SETVAL, sem_union_init) == -1)
    {
        perror("semaphore failed");
        ROS_ERROR("Creating semaphore failed to initialize\n");
        exit(EXIT_FAILURE);
    }
        
    //Create the shared memory
    _shared_memory1_id = shmget((key_t)SHARED_MEMORY_KEY, sizeof(struct shared_data), 0666 | IPC_CREAT);		//Shared memory key , Size in bytes, Permission flags
    if (_shared_memory1_id == -1)
    {
        ROS_ERROR("Shared memory shmget() failed");
        exit(EXIT_FAILURE);
    }
        
    //Make the shared memory accessible to the program
    _shared_memory1_pointer = shmat(_shared_memory1_id, (void *)0, 0);
    if (_shared_memory1_pointer == (void *)-1)
    {
        ROS_ERROR("Shared memory shmat() failed\n");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Shared memory attached at %p\n", _shared_memory1_pointer);
        
    //Assign the shared_memory segment
    _shared_memory1 = (struct shared_data *)_shared_memory1_pointer;
}
    
void deinit()
{
    ROS_INFO("Destructor Start\n");
    sem_wait(sem_id);
    if (shm_unlink(SHMOBJ_PATH) != 0) {
        perror("In shm_unlink()");
        exit(1);
    }
    /**
     * Semaphore Close: Close a named semaphore
     */
    if ( sem_close(sem_id) < 0 )
    {
        perror("sem_close");
    }
        
    /**
     * Semaphore unlink: Remove a named semaphore  from the system.
     */
    if ( sem_unlink("/mysem") < 0 )
    {
        perror("sem_unlink");
    }

    if (shmdt(_shared_memory1_pointer) == -1)
    {
        fprintf(stderr, "shmdt failed\n");
        //exit(EXIT_FAILURE);
    }
    if (shmctl(_shared_memory1_id, IPC_RMID, 0) == -1)
    {
        fprintf(stderr, "shmctl(IPC_RMID) failed\n");
        //exit(EXIT_FAILURE);
    }
    //Delete the Semaphore
    //It's important not to unintentionally leave semaphores existing after program execution. It also may cause problems next time you run the program.
    union semun sem_union_delete;
    if (semctl(_semaphore1_id, 0, IPC_RMID, sem_union_delete) == -1)
        fprintf(stderr, "Failed to delete semaphore\n");
    Log::print(LOG_INFO, "LPSThread::Destructor End\n");
}

lps_range_t getTagRange(int anchorid, int tagid)
{
    lps_range_t ret;
    ret.t = 0;
    ret.r = -1;
    deque<lps_range_t>::reverse_iterator mi = _anchor_ranges[anchorid].rbegin();
    while (mi!= _anchor_ranges[anchorid].rend() && mi->tag_id != tagid) mi++;
    if (mi == _anchor_ranges[anchorid].rend()) return ret;
    return *mi;
}



void updateShm()
{
    // Prepare new json
    string json = "{";
    int n=0;
    for (unsigned i=0;i<_anchor_ranges.size();i++)
    {
        //if (_anchor_ranges[i].size()==0) continue;
        if (n++!=0) json += ",";
        json += StringUtils::stringf("\"Anchor%.2d\":{", i);
        json += StringUtils::stringf("\"ranges\":[");
        // Only exporting info for max 4 tags
        for (unsigned j=0;j<4;j++)
        {
            lps_range_t lr = getTagRange(i,j);
            if (j!=0) json += ",";
            double td = ros::Time::now().toSec()-lr.t;
            td = (abs(td >10))? 10.0 : td;
            json += StringUtils::stringf("{\"t\":\"%.3f\",\"r\":\"%.3f\"}", td, lr.r);
        }
        json += StringUtils::stringf("],");
        json += StringUtils::stringf("\"location\": ");
        json += StringUtils::stringf("[\"%.3f\",\"%.3f\",\"%.3f\",\"%.3f\"]", _anchors[i].x(), _anchors[i].y(), _anchors[i].z(), _anchors[i].theta());
        json += StringUtils::stringf("}", i);
    }
    json += "}";

#if 0
    // Posix SHM
    printf("Waiting \n");
    sem_wait(sem_id);
    memset(_shared_msg->json,0,sizeof(_shared_msg->json));
    strncpy(_shared_msg->json, json.c_str(), sizeof(_shared_msg->json));
    sem_post(sem_id);
    printf("posting %zd\n", json.size());
#endif

    //----- SEMAPHORE GET ACCESS -----
    if (!semaphore1_get_access())
        exit(EXIT_FAILURE);

    strncpy(_shared_memory1->json, json.c_str(), sizeof(_shared_msg->json));
        
    //----- SEMAPHORE RELEASE ACCESS -----
    if (!semaphore1_release_access())
        exit(EXIT_FAILURE);
}
    
void updateLocations()
{
    Log::print(LOG_DEBUG, "LPSThread::updateLocations size of _anchors=%d\n", _anchors.size());
    for (unsigned i=0;i<_anchors.size();i++)
    {
        Log::print(LOG_DEBUG, "LPSThread::updateLocations[%d] start\n", i);
        // Use the last two measurements from different tags
        deque<lps_range_t>::reverse_iterator mi = _anchor_ranges[i].rbegin();
        if (mi == _anchor_ranges[i].rend()) continue;

        int c1_tagid=mi->tag_id;
        double r1=mi->r;
        AnchorPoint c1(_tags[mi->tag_id]);
        Log::print(LOG_DEBUG, "LPSThread::updateLocations[%d] First range from tag %d p(%.1f,%.1f,%.1f) r%.3f\n", 
                   i, c1_tagid, c1.x(),c1.y(),c1.z(),r1);

        // Take next measurement that is not from the same tag
        mi++;
        while (mi!= _anchor_ranges[i].rend() && mi->tag_id == c1_tagid) mi++;
        if (mi == _anchor_ranges[i].rend()) continue;

        AnchorPoint c2(_tags[mi->tag_id]);
        int c2_tagid=mi->tag_id;
        double r2=mi->r;
        Log::print(LOG_DEBUG, "LPSThread::updateLocations[%d] Second range from tag %d p(%.1f,%.1f,%.1f) r%.3f\n", 
                   i, c2_tagid, c2.x(),c2.y(),c2.z(),r2);
        double tol=0.8;
        list<Point3> solutions=AnchorPoint::CircleCircleIntersection(c1, r1, c2, r2, 1.70, tol);
        Log::print(LOG_DEBUG, "LPSThread::updateLocations[%d] %d intersections\n", i, solutions.size());

        // Take next measurement that is not from either of the first tags
        mi++;
        while (mi!= _anchor_ranges[i].rend() && (mi->tag_id == c1_tagid || mi->tag_id == c2_tagid)) mi++;
        if (mi == _anchor_ranges[i].rend()) continue;

        // determine real solution by comparing to third available tag
        Point3 c3 = _tags[mi->tag_id];
        double r3 = mi->r;
        Log::print(LOG_DEBUG, "LPSThread::updateLocations[%d] Third range from tag %d p(%.1f,%.1f,%.1f) r%.3f\n", 
                   i, mi->tag_id, c3.x(),c3.y(),c3.z(),r3);
        Point3 best_solution;
        double best_err = std::nan("");
        for (list<Point3>::iterator ii=solutions.begin();ii!=solutions.end();ii++)
        {
            double err = fabs(c3.distance(*ii)-r3);
            if (err < tol && (err < best_err || ::isnan(best_err)))
            {
                best_solution = *ii;
                best_err = err;
            }
        }
        if (!::isnan(best_err))
        {
            _anchors[i].setLocation(best_solution);
            Log::print(LOG_VERBOSE, "LPSThread::updateLocations[%d] new location (%.3f,%.3f,%.3f) err=%.3f\n", 
                       i, best_solution.x(),best_solution.y(),best_solution.z(), best_err);
            updateShm();
        }
    }
}


    
void addLPSRange(lps_range_t &lr)
{
    if ((int)_anchor_ranges.size() < lr.anchor_id+1)
    {
        _anchor_ranges.resize(lr.anchor_id+1);
        _anchors.resize(lr.anchor_id+1);
    }
        
    _anchor_ranges[lr.anchor_id].push_back(lr);
    while (_anchor_ranges[lr.anchor_id].size() > 20) 
        _anchor_ranges[lr.anchor_id].pop_front();
    Log::print(LOG_INFO, "anchor[%d]: size=%d\n",lr.anchor_id,_anchor_ranges[lr.anchor_id].size());
    updateShm();
}

void updateFromMysql()
{
    if (!_mw) return;
    string sql = "SELECT name,value FROM config";
        
    if (!_mw->query(sql)) 
    {
        ROS_ERROR("updateFromMysql: failed sql\n");
        return;
    }
        
    // Use all the config to update stuff
    int nrows = _mw->num_rows();
    for (int i=0;i<nrows;++i)
    {
        char** r = _mw->fetch_row();
        Log::print(LOG_VERBOSE, "updateFromMysql: %s %s\n", r[0], r[1]);
        if (_mw->num_fields() == 0) continue;
        if (strstr(r[0], "Tag") != 0)
        {
            vector<string> v;
            StringUtils::tokenize_string(r[1],v,",");
            Vector3 d;
            for (unsigned j=0;j<v.size();j++)
                d[j] = strtof(v[j].c_str(), NULL);

            unsigned tag_id = strtol(r[0]+3, NULL, 0);
            ROS_INFO("updateFromMysql: tag_id=%d, (%f,%f,%f)\n", tag_id, d[0], d[1], d[2]);
            if (tag_id<_tags.size())
                _tags[tag_id] = Point3(d);
        }

        if (strstr(r[0], "MapArea") != 0)
        {
            ROS_INFO("updateFromMysql: maparea='%s'\n", r[1]);
            _map.clearFloorspace();
            _map.addPath(r[1],0.0);
        }
    }
        
}

void initStructures()
{
    ROS_INFO("initStructures");
    _anchor_ranges.resize(10);
    _anchors.resize(10);
    
    //init_posix_shm();
    init_systemV_ipc();
    
    _tags.resize(4, Point3(0,0,0));
    _tags[0] = Point3(5.60000,1.8000,2.25600);
    _tags[1] = Point3(2.70000,0.0000,2.21000);
    _tags[2] = Point3(0.00000,1.9500,1.93000);
    _tags[3] = Point3(3.05000,3.8000,2.25000);
    
    updateShm();
    ROS_INFO("initStructures: done");
}


double _updated_map_and_tags = 0;
void lpsrangeCallback(const lps::LPSRange::ConstPtr& msg)
{
    char result[512];
    lps_range_t lr;
    lr.t=msg->header.stamp.toSec();
    lr.r=msg->dist_mm/1000.0;
    lr.tag_id = msg->tag_id;
    lr.anchor_id = msg->anchor_id;
    addLPSRange(lr);

    updateLocations();

    if (ros::Time::now().toSec() - _updated_map_and_tags > 30.0)
    {
        _updated_map_and_tags = ros::Time::now().toSec();
        updateFromMysql();
    }
}


int main(int argc, char *argv[])
{
    Options opts(argc, argv, options);

    // initialize logging
    Log::init(&opts);

    ros::init(argc, argv, "locationtracker");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("lpsranges", 100, lpsrangeCallback);

    ROS_INFO("MysqlThread: Connecting to database\n");
    _mw = new MysqlWrap("localhost", "md_usr", "peoplesleepingbetweensheets", "mddb");
    _mw->connect();

    initStructures();

    string program_ident;

    // Load map if there is one
    SvgMap map;
    if (opts.defined("map"))
    {
        map.init(opts.getString("map"));
        addMap(map);
    }

    ros::spin();
    deinit();

	return 0;
}



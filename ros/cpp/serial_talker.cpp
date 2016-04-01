#include <vector>
#include <deque>

#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdlib.h>

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <lps/LPSRange.h>

#include <math.h>
#include <loligo/core/Options.h>
#include <loligo/core/Log.h>
#include <loligo/serial_cobs/Serial.h>
#include <loligo/core/StringUtils.h>
#include <loligo/serial_cobs/RangePacket.h>

static struct Option options[] = 
{
    { "device",              OpType(OP_REQ),    "/dev/ttyUSB*",
      "tty interface for lps, generally /dev/ttyUSBx"
    },
    { "device-speed",        OpType(OP_REQ),    "115200",
      "serial speed"
    },
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



std::vector<Serial*> _s;

bool addSerialDevice(const string device, const string speed)
{
    if (!Serial::checkDeviceFile(device.c_str())) return false;
    Serial *s = new Serial('\0');
    if (!s->openDevice(device.c_str(), speed.c_str())) 
    {
        delete s;
        return false;
    }
    _s.push_back(s);
    return true;
}


int main(int argc, char *argv[])
{
    Options opts(argc, argv, options);

    // initialize logging
    Log::init(&opts);

    if (opts("help"))
    {
        std::cerr << "lpsmonitor - monitor lps tags." << std::endl;
        std::cerr << "Options:" << std::endl;

        opts.printHelp(0);
        return 0;
    }

    ros::init(argc, argv, "lps_talker");

    ros::NodeHandle n;
    ros::Publisher lps_pub = n.advertise<lps::LPSRange>("lpsranges", 100);
    ros::Rate loop_rate(10);
    
    // Open serial ports
    std::vector<std::string> device = opts.getStrings("device");
    std::string speed = opts.getString("device-speed");
    for (unsigned i=0;i<device.size();i++)
    {
        string dev = device[i];
        
        // check for wildcards 
        size_t wc_pos = dev.find("*");
        if (wc_pos != string::npos)
        {
            for (unsigned i=0;i<100;i++)
            {
                string devn = dev.substr(0, wc_pos) + StringUtils::stringf("%d", i);
                addSerialDevice(devn,speed);
            }
            continue;
        }
        
        addSerialDevice(dev,speed);
    }

    vector<int> stimeouts(_s.size(),0);
    char result[512];
    while (ros::ok())
    {
        Log::print(LOG_INFO, "Loop start\n");
        bool didsomething = false;
        for (unsigned i=0;i<_s.size();i++)
        {
            if (stimeouts[i]>10) continue;
            // Trigger transmission
            uint8_t d=0;_s[i]->write_bytes(&d,1);
            usleep(10000);
            if (!_s[i]->read_available())
            {
                if (_s[i]->wait_for_data(50) != 1)
                {
                    ROS_DEBUG("%d no data?\n",i);
                    stimeouts[i]++;
                    continue;
                }
            }
            int n=20;
            while (_s[i]->wait_for_data(50) > 0 && n-->0)
            {
                size_t plen = _s[i]->read_packet(result, sizeof(result)-1);
                if (plen == 0) continue;
                if (plen < 0) continue;
                ros::Time t = ros::Time::now();
                
                Packet p;
                bool preadok = p.readFrom((uint8_t*)result, plen);
                if (!preadok) continue;
                if (p.type() != PACKET_TYPE_RANGE) continue;
                
                RangePacket rp(p);
                if (rp.anchorid() > 0xff) continue;

                lps::LPSRange rangemsg;
                rangemsg.tag_id=rp.from();
                rangemsg.dist_mm=rp.dist();
                rangemsg.anchor_id=rp.anchorid();
                rangemsg.power=rp.power();
                ROS_INFO("%.2x->%.2x %6dmm",rp.from(),rp.anchorid(),rp.dist());

                lps_pub.publish(rangemsg);
                didsomething = true;
            }
        }
        if (!didsomething)
            for (unsigned i=0;i<stimeouts.size();i++) stimeouts[i]--;

        ros::spinOnce();
        loop_rate.sleep();
    }
    
	return 0;
}



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
    { "device",              OpType(OP_REQ),    "/dev/ttyUSB.tag0,/dev/ttyUSB.tag1,/dev/ttyUSB.tag2,/dev/ttyUSB.tag3",
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
    Serial *s = new Serial('\0');
    if (!s->openDevice(device, speed)) 
    {
        ROS_ERROR("Can't add device: %s, will try later", device.c_str());
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
    ros::Publisher lps_pub = n.advertise<lps::LPSRange>("lpsranges", 20);
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

    // Sleep to allow arduinos to exit bootloader
    ros::Duration(2.0).sleep();

    // If there are no serial devices, there's no point in running
    if (_s.size() == 0)
    {
        ROS_ERROR("No Serial devices to listen to, exiting");
        exit(2);
    }

    vector<int> stimeouts(_s.size(),0);
    char result[512];
    uint32_t count=0;
    while (ros::ok() && _s.size()>0)
    {
        bool didsomething = false;
        for (unsigned i=0;i<_s.size();i++)
        {
            // Make sure to empty buffers 
            while (_s[i]->read_packet(result, sizeof(result)-1)!=0);
            _s[i]->flush();
            _s[i]->clearBuffers();

            // Trigger transmission
            ROS_DEBUG("Trigger %d",i); 
            uint8_t d=0;
            int ret = _s[i]->write_bytes(&d,1);
            if (ret < 0 || _s[i]->read_eof()) 
            {
                ROS_ERROR("Had to reopen '%s'", _s[i]->device().c_str());
                _s[i]->reopenDevice();
                continue;
            }
            double trigger_time = ros::Time::now().toSec();
            usleep(10000);

            if (!_s[i]->read_available())
            {
                ROS_DEBUG("Waiting for %d",i); 
                if (_s[i]->wait_for_data(200) < 1)
                {
                    ROS_DEBUG("%d no data?",i);
                    if (++stimeouts[i]>10)
                    {
                        ROS_ERROR("Reopened '%s' - no data for 10 cycles", _s[i]->device().c_str());
                        stimeouts[i] = 0;
                        _s[i]->reopenDevice();
                    }
                    continue;
                }
            }
            // We received something, reset timeouts
            stimeouts[i] = 0;

            int n=100;
            while (n-->0 && (ros::Time::now().toSec() - trigger_time < 0.300))
            {
                result[0]=0xff;
                size_t plen = _s[i]->read_packet(result, sizeof(result)-1);
                if (plen == 0)
                {
                    _s[i]->wait_for_data(50);
                    continue;
                }
                if (plen < 0) 
                {
                    ROS_ERROR("plen<0");
                    break;
                }
                ros::Time t = ros::Time::now();
                
                Packet p;
                bool preadok = p.readFrom((uint8_t*)result, plen);
                if (!preadok) continue;
                if (p.type() != PACKET_TYPE_RANGE) continue;
                
                RangePacket rp(p);
                if (rp.anchorid() == 0xeeee) break;
                ROS_DEBUG("%.2x->%.2x %6dmm %.1fdbm (%d)",rp.from(),rp.anchorid(),rp.dist(),rp.power(),n);
                if (rp.anchorid() > 0xff) continue;

                lps::LPSRange rangemsg;
                rangemsg.header.stamp=t;
                rangemsg.header.seq=count++;
                rangemsg.tag_id=rp.from();
                rangemsg.dist_mm=rp.dist();
                rangemsg.anchor_id=rp.anchorid();
                rangemsg.power=rp.power();

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



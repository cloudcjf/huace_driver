#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include "unistd.h"
#include "gps_processing.h"
#include "serial.h"
#include <zcm/zcm.h>
#include "zcm/zcm-cpp.hpp"
#include "MsgNavInfoSignal.hpp"
#include "MsgRTK.hpp"
#include "WGS84UTM.h"
#include "nature.h"
#include "coordinate_converter/basic_coordinate_converter.h"
constexpr uint32_t GpsAndSystemDiffSeconds = 315964800; // 1980-01-06 vs 1970-01-01
constexpr uint32_t SecondsPerWeek = 60 * 60 * 24 * 7;
const std::string GPCHC_BEGIN_CHAR = "$";
const std::string GPCHC_ENDLINE = "\r\n"; // 结束符
using namespace std;
zcm::ZCM pubzcm{"ipc"};


int main(int argc, char **argv) {
//        string path1 = "./data_receiver.txt";
//        string path2 = "./buff_string.txt";
//        string path3 = "./gps_processing.txt";
//        int len_total;

    GpsData gps;
    serial::Serial ser; //声明串口对象
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(460800);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 1second
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        cerr << "Unable to open port " << "\n";
        return -1;
    }
    // 检测串口是否已经打开，并给出提示信息
    if (ser.isOpen()) {
        cout << "Serial Port initialized" << "\n";
    } else {
        cerr << "[FATAL]~~~~~~~~Serial Port uninitialized" << "\n";
        return -1;
    }
    std::string buff_string;
    std::string buffer;
    size_t max_size = 65536;
    MsgNavInfoSignal msg_nav_temp;
    while (pubzcm.good()) {
        while (ser.available()) {
            // cout << "Reading from serial port" << "\n";
            buffer = ser.readline(max_size,GPCHC_ENDLINE);
            buff_string += buffer;
            int index = 0, start_index = -1, end_index = -1;
            while (index < buff_string.length()) {
                start_index = buff_string.find(GPCHC_BEGIN_CHAR);
                if (start_index == -1) {
                    cout << "datas in this time is useless" << "\n";
                    buff_string.clear();
                    break;
                } // 没有起始位,常出现于刚开始,丢弃所有数据
                else // 找到起始位
                {
                    end_index = buff_string.find(GPCHC_ENDLINE);
                    if (end_index == -1) { // 有起始位没有结束位
                        buff_string = buff_string.substr(start_index); // 留下后半段
                        break;
                    } else { // 有起始位也有结束位
                        index = end_index;
                        int valid_gps_data = gps.GpsProcessing(
                                buff_string.substr(start_index, end_index - start_index + 2), gps);
                        if (valid_gps_data > 0) // 正确解析数据
                        {
                            //int gps_status_high, gps_status_low;
                            msg_nav_temp.timestamp = ((static_cast<int64_t>(GpsAndSystemDiffSeconds + gps.week * SecondsPerWeek) - 18) * static_cast<int64_t>(100) +
                                   			static_cast<int64_t>(gps.time * 100.0 + 0.5) ) * static_cast<int64_t>(10000);
                            msg_nav_temp.latitude = gps.latitude;
                            msg_nav_temp.longitude = gps.longitude;
                            msg_nav_temp.altitude = gps.altitude;

   			    TiEV::Angle angle_lon, angle_lat;
    			    angle_lon.setByDegree(msg_nav_temp.longitude);
    			    angle_lat.setByDegree(msg_nav_temp.latitude);
    			    TiEV::WGS84Coor latlon_point((TiEV::LAT)angle_lat, (TiEV::LON)angle_lon);
    			    TiEV::UTMCoor utm_point = TiEV::latLonToUTMXY(latlon_point);

                          //  THIRD_PARTY_LIB::UTMCoor xy = THIRD_PARTY_LIB::LatLonToUTMXY(msg_nav_temp.latitude,msg_nav_temp.longitude,51);
                            msg_nav_temp.utm_x =utm_point.x ;
                            msg_nav_temp.utm_y =utm_point.y;

                            msg_nav_temp.speed = gps.speed;
                            msg_nav_temp.velocity_east = gps.speed_east;
                            msg_nav_temp.velocity_north = gps.speed_north;
                            msg_nav_temp.acceleration_x = gps.acceleration_x * 9.7964;
                            msg_nav_temp.acceleration_y = gps.acceleration_y * 9.7964;
			    if(gps.heading > 270 && gps.heading < 360){
				msg_nav_temp.angle_head = (450 - gps.heading) / 180 * pi;
			    }
			    else{
                            	msg_nav_temp.angle_head = (90 - gps.heading) / 180 * pi;
			    }
                            msg_nav_temp.angle_pitch = gps.pitch / 180 * pi;
                            msg_nav_temp.angle_roll = gps.roll / 180 * pi;
                            msg_nav_temp.angular_vel_z = gps.angular_z / 180 * pi;
                            msg_nav_temp.curvature = 0;
                            msg_nav_temp.gps_num_satellites = gps.num_satellites;
                            msg_nav_temp.HPOS_accuracy = 0;
                           // gps_status_high = gps.status / 10;
                           // gps_status_low = gps.status - 10 * gps_status_high;
                            if (gps.status == 42 || gps.status == 82) {
                                msg_nav_temp.RTK_status = 1;
                                msg_nav_temp.is_reckoning_vaild = 1;
                            } else {   
                                msg_nav_temp.RTK_status = 0;
                                msg_nav_temp.is_reckoning_vaild = 0;
                            }
			 std::cout <<"RTK status is: "<<(int)msg_nav_temp.RTK_status<<","<<gps.status <<"\n";
                            pubzcm.publish("navinfo", &msg_nav_temp);
                        }
                        //不管解析的gps数据是否发布,都应该从缓存中剔除,进入到这个条件分支下的缓存区还是原来的状态,
                        //没有减小,如果缓存中去掉此次gps的字符串还有剩余字符,则取子串,再进入while循环判断,
                        //否则清空缓存区跳出循环.因此,不需要检查缓存的大小模块,缓存不会超标
                        if (index + 2 < buff_string.length()) {
                            buff_string = buff_string.substr(end_index + 2);
                            index = 0;
                            start_index = -1;
                            end_index = -1; // reset
                            continue;
                        } else {
                            buff_string.clear();
                            break;
                        }
                    }
                }

            }
        }
        usleep(10000);
    }
    ser.close();
    return 0;
}

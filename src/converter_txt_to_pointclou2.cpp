#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stdint.h>
#include <bitset> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
//#include <opencv/cv.h>

using namespace std;

//global variables
vector<float>  x, y, z, I;
vector<int> r, g, b;

sensor_msgs::PointCloud cloud1;
sensor_msgs::PointCloud2 cloud2;

int i = 0;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZI PointTypeI;

pcl::PointCloud<PointType> cloudmap; 
pcl::PointCloud<PointTypeI> cloudmapI; 

std::string folder_path;
std::string name_cloud;
std::string type;
std::string file_name;
//string file_name;


//Auxiliary functions
float str2float(std::string s)
{
    float number;
    std::stringstream ss;
    ss << s;
    ss >> number;
    return number;
}

int str2int(std::string s)
{
    int number;
    std::stringstream ss;
    ss << s;
    ss >> number;
    return number;
}

uint32_t createRGB(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}

//function to red txt
void read_txt_file(const char* file_name)
{
    string line, aux;

    //read file in txt
    fstream map_txt;
    map_txt.open(file_name, ios::in);
    
    if (map_txt.is_open())
    {
        cout << "- open! \n";
        if (type=="rgb"){
            while (map_txt.good()) {
                // Output the text from the file
                getline (map_txt,aux,',');
                x.push_back(str2float(aux));
                getline (map_txt,aux,',');
                y.push_back(str2float(aux));
                getline (map_txt,aux,',');
                z.push_back(str2float(aux));
                getline (map_txt,aux,',');
                r.push_back(str2int(aux));
                getline (map_txt,aux,',');
                g.push_back(str2int(aux));
                getline (map_txt,aux);
                b.push_back(str2int(aux)); 
            }        
        }else if (type=="intensity"){
            while (map_txt.good()) {
                // Output the text from the file
                getline (map_txt,aux,',');
                x.push_back(str2float(aux));
                getline (map_txt,aux,',');
                y.push_back(str2float(aux));
                getline (map_txt,aux,',');
                z.push_back(str2float(aux));
                getline (map_txt,aux);
                I.push_back(str2int(aux));
            }    
        }else if (type=="xyz"){
            while (map_txt.good()) {
                // Output the text from the file
                getline (map_txt,aux,',');
                x.push_back(str2float(aux));
                getline (map_txt,aux,',');
                y.push_back(str2float(aux));
                getline (map_txt,aux);
                z.push_back(str2float(aux));
            }
        }
        
        map_txt.close();
    }else{
        cout << "- no open! \n";
    }
}

void create_pointcloud1()
{
    uint8_t R, G, B;
    uint32_t RGBColor;

    //header:
    cloud1.header.seq = i + 1;
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "/map";

    //data:
    cloud1.points.resize(x.size()-1);

    if (type=="rgb"){
        cloud1.channels.resize(1);
        cloud1.channels[0].name = "rgb";
        cloud1.channels[0].values.resize(x.size()-1);
        for (int k = 0; k < x.size() - 1; k++)
        {
            //x, y, and z coordinates
            cloud1.points[k].x = x[k];
            cloud1.points[k].y = y[k];
            cloud1.points[k].z = z[k];

            R = r[k];
            G = g[k];
            B = b[k];
            RGBColor = createRGB(R, G, B);
            cloud1.channels[0].values[k] = *(float *)(&RGBColor);
        }
    }else if (type=="intensity"){
        cloud1.channels.resize(1);
        cloud1.channels[0].name = "intensity";
        cloud1.channels[0].values.resize(x.size()-1);
        for (int k = 0; k < x.size() - 1; k++)
        {
            //x, y, and z coordinates
            cloud1.points[k].x = x[k];
            cloud1.points[k].y = y[k];
            cloud1.points[k].z = z[k];

            cloud1.channels[0].values[k] = I[k];
        }
    }else if (type=="xyz"){
        cloud1.channels.resize(1);
        cloud1.channels[0].name = "intensity";
        cloud1.channels[0].values.resize(x.size()-1);
        for (int k = 0; k < x.size() - 1; k++)
        {
            //x, y, and z coordinates
            cloud1.points[k].x = x[k];
            cloud1.points[k].y = y[k];
            cloud1.points[k].z = z[k];

            cloud1.channels[0].values[k] = 0;
        }
    }    
}

int main (int argc, char **argv)
{
    // node init
    ros::init(argc, argv, "mount_map_txt_pointcloud2");
    cout << "------------------Starting--------------------! \n";
    
    //publisher topics
    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_map", 10);

    //get parameters /converter_txt_to_pointclou2/
    ros::NodeHandle nh_;
    nh_.param("/converter_to_pointclou2/file_name", file_name, std::string("mapa_corredor_curto.txt"));
    nh_.param("/converter_to_pointclou2/path", folder_path, std::string("/tmp/"));
    nh_.param("/converter_to_pointclou2/cloud_name", name_cloud, std::string("file_name.pcd"));
    nh_.param("/converter_to_pointclou2/type", type, std::string("xyz"));
    
    cout << "HEADER: \n";
    cout << "- Converter txt file: \n";
    cout << "- Txt name: "+file_name+"\n";
    cout << "- Pcd name: "+name_cloud+"\n";
    cout << "- Saving in: "+folder_path+"\n";
    cout << "STATUS: \n";

    //read txt file
    std::string comp = folder_path+file_name;
    const char* txt_name = const_cast<char*>(comp.c_str());
    cout << "- Reading txt file! \n";
    read_txt_file(txt_name);

    //create a pointcloud
    create_pointcloud1();
    sensor_msgs::convertPointCloudToPointCloud2 (cloud1, cloud2);

    if (type=="rgb"){
        //pointcloud2 to PCL
        pcl::fromROSMsg(cloud2, cloudmap);

        // save final point cloud
        pcl::io::savePCDFileASCII(folder_path+name_cloud, cloudmap);
    }else{
        //pointcloud2 to PCL
        pcl::fromROSMsg(cloud2, cloudmapI);

        // save final point cloud
        pcl::io::savePCDFileASCII(folder_path+name_cloud, cloudmapI);        
    }
    cout << "- Salved! \n";
    cout << "Publishing the point cloud in the topic: /cloud_map and frame id: /map \n";


    ros::Rate r(10);
    while (ros::ok())
    {
        create_pointcloud1();
        sensor_msgs::convertPointCloudToPointCloud2 (cloud1, cloud2);
        cloud_pub.publish(cloud2);

        ros::spinOnce();
        r.sleep();
    }

    ros::spin();
    return 0;
}
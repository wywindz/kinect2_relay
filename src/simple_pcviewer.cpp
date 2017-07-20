#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>

std::string topicName="UER_K2_TOPIC_CLOUDXYZRGBA";
bool updateFlag=false;
bool modeXYZ=false;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr recvCloud_XYZRGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZ>::Ptr recvCloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
bool stopped=false;

void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::fromROSMsg(*msg,*recvCloud_XYZRGBA);
  if(modeXYZ)
  {
    recvCloud_XYZ->points.resize(recvCloud_XYZRGBA->points.size());
    for(size_t i=0;i<recvCloud_XYZ->points.size();++i)
    {
      recvCloud_XYZ->points[i].x=recvCloud_XYZRGBA->points[i].x;
      recvCloud_XYZ->points[i].y=recvCloud_XYZRGBA->points[i].y;
      recvCloud_XYZ->points[i].z=recvCloud_XYZRGBA->points[i].z;
    }
  }
  updateFlag=true;
}

void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *)
{
  if(event.keyUp())
  {
    switch(event.getKeyCode())
    {
      case 27:
      case 'q':
        stopped=true;
        break;
    }
  }
}

void printUsage(const char* progName)
{
  std::cout<<"\n\nUsage: "<<progName<<"  [Options]\r\n"
  <<"Options:\n"
  <<"-----------------------------------------------\r\n"
  <<"--xyz     show pointXYZ\r\n"
  <<"--xyzrgba show pointXYZRGBA\r\n"
  <<"-h        help\r\n"
  <<"\r\n\n";
}

int main(int argc, char** argv)
{
  //Parse Command line
  if(pcl::console::find_argument(argc,argv,"-h")>=0)
  {
    printUsage(argv[0]);
    return 0;
  }
  if(pcl::console::find_argument(argc,argv,"--xyz")>=0)
  {
    modeXYZ=true;
  }
  if(pcl::console::find_argument(argc,argv,"--xyzrgba")>=0)
  {
    modeXYZ=false;
  }

  ros::init(argc, argv,"myviewer");
  ros::NodeHandle nh("Kinect2_Relay_Msg");
  ros::Subscriber sub=nh.subscribe<sensor_msgs::PointCloud2>(topicName,100,msgCallback);

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate rate(20);
  while(!updateFlag)
  {
    if(!ros::ok())
      return -1;
  }

pcl::visualization::PCLVisualizer viewer("scene cloud");
//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> colorH(recvCloud,200,200,200);
if(modeXYZ)
{
  viewer.addPointCloud(recvCloud_XYZ,"sceneCloud");
}
else
{
  viewer.addPointCloud(recvCloud_XYZRGBA,"sceneCloud");
}
viewer.setBackgroundColor(0,0,0,0);
viewer.initCameraParameters();
viewer.setShowFPS(true);
viewer.setCameraPosition(0,0,0,0,-1,0);
viewer.registerKeyboardCallback(&keyboardEvent,(void*)(&viewer));

while(!viewer.wasStopped() && !stopped)
{
  if(updateFlag)
  {
    if(modeXYZ)
    {
      viewer.updatePointCloud(recvCloud_XYZ,"sceneCloud");
    }
    else
    {
      viewer.updatePointCloud(recvCloud_XYZRGBA,"sceneCloud");
    }
    updateFlag=false;
  }
  viewer.spinOnce(10);
  rate.sleep();
}

viewer.close();
return 0;
}

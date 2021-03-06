#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

class PointRelay
{
private:

  std::mutex lock;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;

  const std::string topicColor,topicDepth;
  const std::string pubCloudXYZTopic,pubCloudXYZRGBATopic;
  bool updateFlag,running;
  const size_t queueSize;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  ros::Publisher pubCloudXYZ;
  ros::Publisher pubCloudXYZRGBA;

  cv::Mat color,depth;
  cv::Mat cameraMatrixColor,cameraMatrixDepth;
  cv::Mat lookupX,lookupY;

  //Sync policy
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo,sensor_msgs::CameraInfo> ExactTimePolicy;
  message_filters::Synchronizer<ExactTimePolicy> *exactSync;
  image_transport::ImageTransport imgTrans;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

public:
    PointRelay(const std::string &topicColor,const std::string &topicDepth, const std::string &pubXYZTopic, const std::string &pubRGBATopic):topicColor(topicColor),topicDepth(topicDepth),pubCloudXYZTopic(pubXYZTopic),pubCloudXYZRGBATopic(pubRGBATopic),updateFlag(false),running(false),queueSize(5),nh("Kinect2_Relay_Msg"),spinner(0),imgTrans(nh)
    {
      cameraMatrixColor=cv::Mat::zeros(3,3,CV_64F);
      cameraMatrixDepth=cv::Mat::zeros(3,3,CV_64F);

      pubCloudXYZ=nh.advertise<sensor_msgs::PointCloud2>(pubCloudXYZTopic,1000);
      pubCloudXYZRGBA=nh.advertise<sensor_msgs::PointCloud2>(pubCloudXYZRGBATopic,1000);
    }

    ~PointRelay()
    {
    }

    void run()
    {
      start();
      stop();
    }
  private:
    void start()
    {
      running=true;

      //Subcribers
      image_transport::TransportHints hints("raw");
      subImageColor=new image_transport::SubscriberFilter(imgTrans,topicColor,queueSize,hints);
      subImageDepth=new image_transport::SubscriberFilter(imgTrans,topicDepth,queueSize,hints);
      
      std::string topicCameraInfoColor=topicColor.substr(0,topicColor.rfind('/'))+"/camera_info";
      std::string topicCameraInfoDepth=topicColor.substr(0,topicDepth.rfind('/'))+"/camera_info";

      subCameraInfoColor=new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,topicCameraInfoColor,queueSize);
      subCameraInfoDepth=new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,topicCameraInfoDepth,queueSize);

      exactSync=new message_filters::Synchronizer<ExactTimePolicy>(ExactTimePolicy(queueSize),*subImageColor,*subImageDepth,*subCameraInfoColor,*subCameraInfoDepth);
      exactSync->registerCallback(boost::bind(&PointRelay::callback,this,_1,_2,_3,_4));
    
      //create thread on each cpu 
      spinner.start();
      std::chrono::milliseconds duration(1);
      //wait for msg to come
      while(!updateFlag)
      {
        if(!ros::ok())
        {
          return;
        }
        std::this_thread::sleep_for(duration);
      }

      initPointCloud();
      createLookup(this->color.cols,this->color.rows);

      //create and publish pointClouds when receiving updated msg
      publishCloud();
    }

    void stop()
    {
      spinner.stop();
      delete exactSync;
      delete subImageColor;
      delete subImageDepth;
      delete subCameraInfoColor;
      delete subCameraInfoDepth;

      running=false;
    }

    void createCloudRGBA(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)const
    {
      const float badPoint=std::numeric_limits<float>::quiet_NaN();

      #pragma omp parallel for
      for(int r=0;r<depth.rows;++r)
      {
        pcl::PointXYZRGBA *itP=&cloud->points[r*depth.cols];
        const uint16_t *itD=depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC=color.ptr<cv::Vec3b>(r);
        const float y=lookupY.at<float>(0,r);
        const float *itX=lookupX.ptr<float>();

        for(size_t c=0;c<(size_t)depth.cols;++c,++itP,++itD,++itC,++itX)
        {
          register const float depthValue=*itD/1000.0f;
          //check for invalid measurements
          if(*itD==0)
          {
            //not valid
            itP->x=itP->y=itP->z=badPoint;
            itP->rgba=0;
            continue;
          }
          itP->z=depthValue;
          itP->x=*itX*depthValue;
          itP->y=y*depthValue;
          itP->b=itC->val[0];
          itP->g=itC->val[1];
          itP->r=itC->val[2];
          itP->a=255;
        }
      }
    }

    void createCloudXYZ(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
    {
      const float badPoint=std::numeric_limits<float>::quiet_NaN();

      #pragma omp parallel for
      for(int r=0;r<depth.rows;++r)
      {
        pcl::PointXYZ *itP=&cloud->points[r*depth.cols];
        const uint16_t *itD=depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC=color.ptr<cv::Vec3b>(r);
        const float y=lookupY.at<float>(0,r);
        const float *itX=lookupX.ptr<float>();

        for(size_t c=0;c<(size_t)depth.cols;++c,++itP,++itD,++itC,++itX)
        {
          register const float depthValue=*itD/1000.0f;
          //check for invalid measurements
          if(*itD==0)
          {
            //not valid
            itP->x=itP->y=itP->z=badPoint;
            continue;
          }
          itP->z=depthValue;
          itP->x=*itX*depthValue;
          itP->y=y*depthValue;
        }
      }
    }

    void createLookup(size_t width, size_t height)
    {
      const float fx=1.0f/cameraMatrixColor.at<double>(0,0);
      const float fy=1.0f/cameraMatrixColor.at<double>(1,1);
      const float cx=cameraMatrixColor.at<double>(0,2);
      const float cy=cameraMatrixColor.at<double>(1,2);

      float *it;
      lookupY=cv::Mat(1,height,CV_32F);
      it=lookupY.ptr<float>();
      for(size_t r=0;r<height;++r,++it)
      {
        *it=(r-cy)*fy;
      }

      lookupX=cv::Mat(1,width,CV_32F);
      it=lookupX.ptr<float>();
      for(size_t c=0;c<width;++c,++it)
      {
        *it=(c-cx)*fx;
      }
    }

    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth, const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
    {
      cv::Mat color,depth;

      readCameraInfo(cameraInfoColor,cameraMatrixColor);
      readCameraInfo(cameraInfoDepth,cameraMatrixDepth);
      readImage(imageColor,color);
      readImage(imageDepth,depth);

      //IR image input
      if(color.type()==CV_16U)
      {
        cv::Mat tmp;
        color.convertTo(tmp,CV_8U,0.02);
        cv::cvtColor(tmp,color,CV_GRAY2BGR);
      }

      lock.lock();
      this->color=color;
      this->depth=depth;
      updateFlag=true;
      lock.unlock();
    }

    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
    {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage=cv_bridge::toCvShare(msgImage,msgImage->encoding);
      pCvImage->image.copyTo(image);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
    {
      double *itC=cameraMatrix.ptr<double>(0,0);
      for(size_t i=0;i<9;++i,++itC)
      {
        *itC=cameraInfo->K[i];
      }
    }

    void initPointCloud()
    {
      
      cloud_rgba=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
      cloud_rgba->width=color.cols;
      cloud_rgba->height=color.rows;
      cloud_rgba->is_dense=false;
      cloud_rgba->points.resize(cloud_rgba->width*cloud_rgba->height);

      cloud_xyz=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      cloud_xyz->width=color.cols;
      cloud_xyz->height=color.rows;
      cloud_xyz->points.resize(cloud_xyz->width*cloud_xyz->height);
    }

    void publishCloud()
    {
     cv::Mat color,depth;
    
     for(; running && ros::ok();)
     {
      if(updateFlag)
      {
        lock.lock();
        color=this->color;
        depth=this->depth;
        lock.unlock();

        //createCloudXYZ(color,depth,cloud_xyz);
        createCloudRGBA(color,depth,cloud_rgba);

        //Convert the CloudT to sensorMsgs::PointCloud2 type
        /*sensor_msgs::PointCloud2 outputCloudXYZ;
        pcl::toROSMsg(*cloud_xyz,outputCloudXYZ);
        outputCloudXYZ.header.frame_id="odom";
*/
        sensor_msgs::PointCloud2 outputCloudRGBA;
        pcl::toROSMsg(*cloud_rgba,outputCloudRGBA);
        outputCloudRGBA.header.frame_id="odom"; 

  //      pubCloudXYZ.publish(outputCloudXYZ);
        pubCloudXYZRGBA.publish(outputCloudRGBA);

        updateFlag=false;
      }
     }

    }
};

    int main(int argc, char** argv)
    {
      //ros::init(argc,argv,"kinect2_relay",ros::init_options::AnonymousName);
      ros::init(argc,argv,"kinect2_realy");
      if(!ros::ok())
      {
        return 0;
      }

      std::string ns=K2_DEFAULT_NS;
      std::string topicColor=K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      std::string topicDepth=K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
      std::string pubXYZTopic="UER_K2_TOPIC_CLOUDXYZ";
      std::string pubRGBATopic="UER_K2_TOPIC_CLOUDXYZRGBA";

      topicColor="/"+ns+topicColor;
      topicDepth="/"+ns+topicDepth;

      OUT_INFO("topic color: " FG_CYAN<<topicColor<<NO_COLOR);
      OUT_INFO("topic depth: " FG_CYAN<<topicDepth<<NO_COLOR);

      PointRelay relay(topicDepth,topicColor,pubXYZTopic,pubRGBATopic);
      
      OUT_INFO("starting kinect2 relay service...");
      relay.run();
      ros::shutdown();
      return 0;
    }

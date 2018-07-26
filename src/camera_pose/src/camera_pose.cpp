#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <Eigen/Dense>
//#include <types_c.h>


using namespace cv;
/// Global Variables
int tmax = 255;
int bi_gray_max = 255;
int bi_gray_min = 245;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_rgb_;
  image_transport::Publisher image_pub_gray;
  image_transport::Publisher image_pub_bi_gray;
public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_rgb_ = it_.subscribe("/app/camera/rgb/image_raw", 1,
      &ImageConverter::image_processing, this);
    //image_ir_ = it_.subscribe("/app/camera/ir/image_raw", 1,
    //     &ImageConverter::bi_ir_main, this);
    image_pub_gray = it_.advertise("/reading_sensors/gray_img", 1);
    image_pub_bi_gray = it_.advertise("/reading_sensors/binarized_gray_img", 1);
    // cv::namedWindow("gray_img");
    // cv::namedWindow("bi_gray");
    // cv::namedWindow("gauss");
    // cv::namedWindow("bi_ir");
    // cv::namedWindow("seg_hsv");
    // cv::namedWindow("seg_rgb");
    // cv::namedWindow("edge");
    // cv::namedWindow("white");
  }

  ~ImageConverter()
  {
    // cv::destroyWindow("gray_img");
    // cv::destroyWindow("bi_gray");
    // cv::destroyWindow("gauss");
    // cv::destroyWindow("bi_ir");
    // cv::destroyWindow("seg_hsv");
    // cv::destroyWindow("seg_rgb");
    // cv::destroyWindow("edge");
    // cv::destroyWindow("white");
  }

  void image_processing(const sensor_msgs::ImageConstPtr& msg)
  {
    /*Read cv_bridge image*/

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);//TYPE_8UC1
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /*divide in 4 the image and look for pixels*/
    gray(cv_ptr->image);
    Mat bi_gray_img = bi_gray(cv_ptr->image);

  std::vector<cv::Point2f> imagePoints = Generate2DPoints(bi_gray_img);
  std::vector<cv::Point3f> objectPoints = Generate3DPoints();

  std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
  /*fx,0,cx,0,fy,cy,0,0,1*/
  Mat intrinsics = (Mat_<double>(3,3) << 614.1699, 0, 329.9491, 0, 614.9002, 237.2788, 0, 0, 1);

  std::cout << "Intrinsic parameters: \n" << intrinsics << std::endl;

  cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 0.1115;
  distCoeffs.at<double>(1) = -0.1089;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;

  cv::Mat rvec(3,1,cv::DataType<double>::type);
  cv::Mat tvec(3,1,cv::DataType<double>::type);

  cv::solvePnP(objectPoints, imagePoints, intrinsics, distCoeffs, rvec, tvec);

  std::cout << "rvec: \n" << rvec << std::endl;
  std::cout << "tvec: \n" << tvec << std::endl;

  cv::Mat rmat(3,3,cv::DataType<double>::type);
  Rodrigues(rmat, rvec);

  std::cout << "rmat: \n" << rmat << std::endl;

  /*Inverting the matrix [ R | t ]^-1 = [R' | -R'*t]*/
  Mat inv_rmat = rmat.t();
  Mat inv_tvec = -inv_rmat * tvec;

  std::cout << "inv_rmat: \n" << inv_rmat << std::endl;
  std::cout << "inv_tvec: \n" << inv_tvec << std::endl;

  double yaw = atan2(inv_rmat.at<double>(1,0),inv_rmat.at<double>(0,0));
  double pitch = atan2(-inv_rmat.at<double>(2,0),sqrt(pow(inv_rmat.at<double>(2,1),2)+pow(inv_rmat.at<double>(2,2),2)));
  double roll =  atan2(inv_rmat.at<double>(2,1),inv_rmat.at<double>(2,2));

  std::cout << "yaw: \n" << yaw*180/3.1416 << std::endl;
  std::cout << "pitch: \n" << pitch*180/3.1416 << std::endl;
  std::cout << "roll: \n" << roll*180/3.1416 << std::endl;

  float sy = sqrt(rmat.at<double>(0,0) * rmat.at<double>(0,0) +  rmat.at<double>(1,0) * rmat.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(rmat.at<double>(2,1) , rmat.at<double>(2,2));
        y = atan2(-rmat.at<double>(2,0), sy);
        z = atan2(rmat.at<double>(1,0), rmat.at<double>(0,0));
    }
    else
    {
        x = atan2(-rmat.at<double>(1,2), rmat.at<double>(1,1));
        y = atan2(-rmat.at<double>(2,0), sy);
        z = 0;
    }

    std::cout << "x: \n" << x*180/3.1416 << std::endl;
    std::cout << "y: \n" << y*180/3.1416 << std::endl;
    std::cout << "z: \n" << z*180/3.1416 << std::endl;

    /*print transformations*/
    //since all odometry is 6DOF we'll need a quaternion created from yaw

    Eigen::Matrix3d erotmat;
    for (int i = 0; i < rmat.rows; i++)
            for (int j = 0; j < rmat.cols; j++)
            {
              erotmat(i,j) =  rmat.at<double>(i,j);
            }


    Eigen::Quaterniond q(erotmat);
    //geometry_msgs::Quaternion odom_quat = q;
    //std::cout << "w: "<<q.w()<<std::endl;
    tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(inv_tvec.at<double>(0), inv_tvec.at<double>(1), inv_tvec.at<double>(2)) );
    transform.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));
    ros::Rate rate(10.0);
    rate.sleep();
    //send the transform
    //odom_broadcaster.sendTransform(p1);



  // std::vector<cv::Point2f> projectedPoints;
  // cv::projectPoints(objectPoints, rvec, tvec, intrinsics, distCoeffs, projectedPoints);
  //
  // for(unsigned int i = 0; i < projectedPoints.size(); ++i)
  //   {
  //   std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
  //   }

  //  cv::waitKey(1);
  }

  void gray (cv::Mat img)
  {
    Mat gray (img.size(), CV_8UC1);
    cvtColor(img, gray, CV_BGR2GRAY);

    sensor_msgs::ImagePtr cv_ptr;
    //
    cv_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();

    image_pub_gray.publish(cv_ptr);
    //imshow("gray_img", gray);

  }

  cv::Mat bi_gray (cv::Mat img)
  {
    Mat gray (img.size(), CV_8UC1);
    cvtColor(img, gray, CV_BGR2GRAY);
    Mat bn = gray.clone();

    threshold(gray, bn, bi_gray_min, bi_gray_max, THRESH_BINARY);
    //adaptiveThreshold(gray, bn_adap, l_max, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3 , 2);
    sensor_msgs::ImagePtr cv_ptr;
    //
    cv_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", bn).toImageMsg();

    image_pub_bi_gray.publish(cv_ptr);
    return bn;
    //cv::imshow("bi_gray", bn);
  }

  std::vector<cv::Point2f> Generate2DPoints(cv::Mat img)
  {
  std::vector<cv::Point2f> points;

  float x,y;
  /*look for first point*/
  for (int i = 0; i < 150; i++)
          for (int j = 0; j < 320; j++)
          {
              if (img.at<uchar>(i,j) >= 200)
              {
              x = j;
              y = i;
              break;
              }
          }
  points.push_back(cv::Point2f(x,y));

  for (int i = 150; i < 300; i++)
          for (int j = 0; j < 320; j++)
          {
              if (img.at<uchar>(i,j) >= 200)
              {
              x = j;
              y = i;
              break;
              }
          }
  points.push_back(cv::Point2f(x,y));

  for (int i = 300; i < 480; i++)
          for (int j = 0; j < 320; j++)
          {
              if (img.at<uchar>(i,j) >= 200)
              {
              x = j;
              y = i;
              break;
              }
          }
  points.push_back(cv::Point2f(x,y));

  for (int i = 0; i < 150; i++)
          for (int j = 320; j < 640; j++)
          {
              if (img.at<uchar>(i,j) >= 200)
              {
              x = j;
              y = i;
              break;
              }
          }
  points.push_back(cv::Point2f(x,y));

  for (int i = 150; i < 300; i++)
          for (int j = 320; j < 640; j++)
          {
              if (img.at<uchar>(i,j) >= 200)
              {
              x = j;
              y = i;
              break;
              }
          }
  points.push_back(cv::Point2f(x,y));

  for (int i = 300; i < 480; i++)
          for (int j = 320; j < 640; j++)
          {
              if (img.at<uchar>(i,j) >= 200)
              {
              x = j;
              y = i;
              break;
              }
          }
  points.push_back(cv::Point2f(x,y));

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }

  return points;
}


std::vector<cv::Point3f> Generate3DPoints()
{
  std::vector<cv::Point3f> points;


  float x,y,z;

  x=0;y=80;z=0;
  points.push_back(cv::Point3f(x,y,z));

  x=0;y=40;z=0;
  points.push_back(cv::Point3f(x,y,z));

  x=0;y=0;z=0;
  points.push_back(cv::Point3f(x,y,z));

  x=28;y=80;z=0;
  points.push_back(cv::Point3f(x,y,z));

  x=28;y=40;z=0;
  points.push_back(cv::Point3f(x,y,z));

  x=28;y=0;z=0;
  points.push_back(cv::Point3f(x,y,z));

  // x=-.5;y=-.5;z=.5;
  // points.push_back(cv::Point3f(x,y,z));

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }

  return points;
}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

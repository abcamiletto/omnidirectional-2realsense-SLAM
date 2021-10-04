#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace AVATAR 
{
	class ImgProcessing;
}	

class AVATAR::ImgProcessing : public nodelet::Nodelet
{
	protected:
		std::string node_name_;
		std::string topic1_;
		std::string topic2_;
		std::string output;
		bool rotate_;
		bool debug_;
		
		boost::shared_ptr<image_transport::ImageTransport> it_;
		typedef image_transport::SubscriberFilter ImageSubscriber;

		ImageSubscriber camera1_sub_;
		ImageSubscriber camera2_sub_;

		typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image, sensor_msgs::Image
		> MySyncPolicy;

		boost::shared_ptr<message_filters::Synchronizer< MySyncPolicy >> sync;
			
		image_transport::Publisher image_pub_;

		cv::Mat homography_;
		std::string homography_string;

		cv_bridge::CvImagePtr down_ptr_, top_ptr_;

		sensor_msgs::ImageConstPtr output_frame_;

		cv::Rect roi_;

		bool calibration;

		void imageCb(const sensor_msgs::ImageConstPtr& topic1, 
						const sensor_msgs::ImageConstPtr& topic2);
	public:
		~ImgProcessing();
		ImgProcessing();
		virtual void onInit();
};

#endif

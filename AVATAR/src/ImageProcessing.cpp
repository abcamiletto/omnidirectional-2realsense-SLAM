#include <ImageProcessing/ImageProcessing.h>

namespace AVATAR 
{
	ImgProcessing::~ImgProcessing()
	{
		cv::destroyAllWindows();
	}
	
	ImgProcessing::ImgProcessing()
	{
		ROS_INFO_STREAM("COSTRUTTORE");
	}

	void ImgProcessing::onInit()
	{
		node_name_ = Nodelet::getName();
		ROS_INFO_STREAM("\t nodelet name" << node_name_);
		
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle local_nh = getPrivateNodeHandle();

		if (local_nh.getParam("topic1", topic1_))
			ROS_INFO_STREAM("\t topic1: " << topic1_);
		else ROS_WARN("\t 'topic1_' Parameter not set");

		if (local_nh.getParam("topic2", topic2_))
			ROS_INFO_STREAM("\t topic2: " << topic2_);
		else ROS_WARN("\t 'topic2_' Parameter not set");

		if (local_nh.getParam("output", output))
			ROS_INFO_STREAM("\t output: " << output);
		else ROS_WARN("\t 'output' Parameter not set");

		if (local_nh.getParam("rotate", rotate_))
			ROS_INFO_STREAM("\t rotate: " << rotate_);
		else ROS_WARN("\t 'rotate' Parameter not set");

		if (local_nh.getParam("debug", debug_))
			ROS_INFO_STREAM("\t debug: " << debug_);
		else ROS_WARN("\t 'debug' Parameter not set");

		// if (local_nh.getParam("homography", homography_string))
		// 	ROS_INFO_STREAM("\t homography: " << homography_string);
		// else ROS_WARN("\t 'homography' Parameter not set");

		// if (local_nh.getParam("calibration", calibration))
		// 	ROS_INFO_STREAM("\t calibration: " << calibration);
		// else ROS_WARN("\t 'calibration' Parameter not set");

		ros::Duration(5.0).sleep();
		
		it_ = boost::shared_ptr<image_transport::ImageTransport>
								(new image_transport::ImageTransport(nh)); 
		image_transport::TransportHints hints("raw", ros::TransportHints(),local_nh);
		camera1_sub_.subscribe(*(it_.get()), topic1_, 10, hints);
		camera2_sub_.subscribe(*(it_.get()), topic2_, 10, hints);

		sync.reset(new message_filters::Synchronizer< MySyncPolicy >(MySyncPolicy(50), camera1_sub_, camera2_sub_));      
		sync->registerCallback( boost::bind( &ImgProcessing::imageCb, this, _1, _2 ) );

		image_pub_ = it_->advertise(output, 1);
		
		roi_ = cv::Rect();
	
		// if(!calibration)
		// {
		// 	std::vector<double> tokens;
		// 	for (auto i = strtok(&homography_string[0], " "); i != NULL; i = strtok(NULL, " "))
		// 	{
		// 		tokens.push_back(std::stod(i));
		// 		ROS_INFO_STREAM("\n: " << tokens[tokens.size()-1]);
		// 	}
		// 	homography_ = cv::Mat(3,3, CV_64FC1, tokens.data());
		// 	ROS_INFO_STREAM("\n\n\n\n homography: " << homography_);
		// }
	}

	void ImgProcessing::imageCb(const sensor_msgs::ImageConstPtr& topic1, const sensor_msgs::ImageConstPtr& topic2)
	{
		try {
			down_ptr_ = cv_bridge::toCvCopy(topic2, sensor_msgs::image_encodings::BGR8);
			top_ptr_ = cv_bridge::toCvCopy(topic1, sensor_msgs::image_encodings::BGR8);

			if(rotate_)
			{
				cv::rotate(top_ptr_->image, top_ptr_->image, cv::ROTATE_180);
				cv::rotate(down_ptr_->image, down_ptr_->image, cv::ROTATE_180);
			}
		
			cv::Mat padded_top; 
			copyMakeBorder(top_ptr_->image, padded_top, 0, top_ptr_->image.size().width, 0, 0, cv::BORDER_ISOLATED, cv::Scalar::all(0));
			
			if(output == "/right")
			{
				homography_ = (cv::Mat_<double>(3,3) << 1.084104694767265, -0.1106879779407097, -1.721804204936373,
														0.02673201523034853, 0.9674296157543039, 470.2568012096532,
														5.296596552988371e-05, -0.0003936489055290201, 1);
			} else if(output == "/left") {
				homography_ = (cv::Mat_<double>(3,3) << 0.9515973914270953, -0.1551210046931248, 50.42143335681721,
														-0.009577303764245759, 0.8050966606298334, 524.8028690935375,
														1.323483046227488e-05, -0.0001510891842282936, 1);
			}

			if(debug_)
				ROS_INFO_STREAM("homography: " << homography_);
			cv::warpPerspective(down_ptr_->image, padded_top, homography_, padded_top.size(),cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
			
			if(roi_.empty())
			{
				cv::Mat app;
				cv::cvtColor(padded_top.clone(), app, cv::COLOR_BGR2GRAY);
				cv::threshold(app, app, 1, 255, cv::THRESH_BINARY);
				std::vector<std::vector<cv::Point> > contours;
				cv::findContours(app, contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
				cv::drawContours(app, contours, 0, cv::Scalar(0,255,0), 2, cv::LINE_8);
				std::vector<cv::Point> cnt = contours[0];
				cv::Rect roi = cv::boundingRect(cnt);
				padded_top = padded_top(roi);
				roi_ = roi;
			}else
				padded_top = padded_top(roi_);

			top_ptr_->image = padded_top.clone();
			image_pub_.publish(top_ptr_->toImageMsg());
		} catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}
	
} // namespace AVATAR
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(AVATAR::ImgProcessing, nodelet::Nodelet)

#include "colorblob.ih"

Colorblob::Colorblob()
:
	d_visual_state(RGB)
{
	ros::NodeHandle param_node("~");
	int h_upper, h_lower, s_upper, s_lower, v_upper, v_lower;
	
	//Iamge topic
	string image_topic;
	param_node.param<string>("image_topic", image_topic, "/NAO/image_bottom");

	//Visual mode
	param_node.param("visual_mode", d_VISUAL_MODE, true);
	
	//colorblob_bal
	param_node.param("red_hue_upper", h_upper, 0);
	param_node.param("red_saturation_upper", s_upper, 0);
	param_node.param("red_value_upper", v_upper, 0);
	param_node.param("red_hue_lower", h_lower, 0);
	param_node.param("red_saturation_lower", s_lower, 0);
	param_node.param("red_value_lower", v_lower, 0);
	d_objects.push_back({"colorblob_red", 10, cv::Scalar(h_upper,s_upper,v_upper), cv::Scalar(h_lower,s_lower,v_lower)});

	//colorblob_goal
	param_node.param("yellow_hue_upper", h_upper, 0);
	param_node.param("yellow_saturation_upper", s_upper, 0);
	param_node.param("yellow_value_upper", v_upper, 0);
	param_node.param("yellow_hue_lower", h_lower, 0);
	param_node.param("yellow_saturation_lower", s_lower, 0);
	param_node.param("yellow_value_lower", v_lower, 0);
	d_objects.push_back({"colorblob_yellow", 10, cv::Scalar(h_upper,s_upper,v_upper), cv::Scalar(h_lower,s_lower,v_lower)});
	
	//colorblob_goal
	param_node.param("blue_hue_upper", h_upper, 0);
	param_node.param("blue_saturation_upper", s_upper, 0);
	param_node.param("blue_value_upper", v_upper, 0);
	param_node.param("blue_hue_lower", h_lower, 0);
	param_node.param("blue_saturation_lower", s_lower, 0);
	param_node.param("blue_value_lower", v_lower, 0);
	d_objects.push_back({"colorblob_blue", 10, cv::Scalar(h_upper,s_upper,v_upper), cv::Scalar(h_lower,s_lower,v_lower)});
	
	//colorblob_def
	param_node.param("def_hue_upper", h_upper, 0);
	param_node.param("def_saturation_upper", s_upper, 0);
	param_node.param("def_value_upper", v_upper, 0);
	param_node.param("def_hue_lower", h_lower, 0);
	param_node.param("def_saturation_lower", s_lower, 0);
	param_node.param("def_value_lower", v_lower, 0);
	d_objects.push_back({"colorblob_def", 10, cv::Scalar(h_upper,s_upper,v_upper), cv::Scalar(h_lower,s_lower,v_lower)});
	

	d_ball_upper = cv::Scalar(h_upper, s_upper, v_upper);
	d_ball_lower = cv::Scalar(h_lower, s_lower, v_lower);

	//Subscriber
	d_image_sub = d_nh.subscribe(image_topic, 1, &Colorblob::newImageCB, this);

	
	//Services
	d_memory_client = d_nh.serviceClient<alice_msgs::MemorySrv>("memory");

	if (d_VISUAL_MODE)
	{
		cv::namedWindow("Colorblob");
		cv::setMouseCallback("Colorblob", onMouse, this);	
	}	
}

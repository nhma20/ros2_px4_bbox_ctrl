#include <rclcpp/rclcpp.hpp>
#include "vision_msgs/msg/bounding_box2_d.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <math.h>  
#include <limits>
#include <vector>
#include <string>


using namespace std::chrono_literals;

//creates a LidarToMmwave class that subclasses the generic rclcpp::Node base class.
class LidarToMmwave : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		LidarToMmwave() : Node("lidar_to_mmwave_converter") {
			fake_bbox_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>("/bbox", 10);


		auto timer_callback = [this]() -> void {
			vision_msgs::msg::BoundingBox2D msg;
			msg.center.x = 210;
			msg.center.y = 150;
			msg.center.theta = 0;
			msg.size_x = 90;
			msg.size_y = 140;
			fake_bbox_pub_->publish(msg);
			RCLCPP_INFO(this->get_logger(),  "Published fake bbox");
		};

		timer_ = this->create_wall_timer(100ms, timer_callback);

		}

		~LidarToMmwave() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down fake bbox publisher node..");
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr fake_bbox_pub_;
		
};

			
int main(int argc, char *argv[])
{
	std::cout << "Starting fake bbox publisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarToMmwave>());

	rclcpp::shutdown();
	return 0;
}

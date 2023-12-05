#pragma once

#include "../dma_dir/dma.hpp"

#include <cstdlib>
#include <memory>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber
	: public rclcpp::Node
{
	using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
public:
	MinimalSubscriber();

	~MinimalSubscriber();

private:
	/**
	 * @brief Reshapes one dimensional input vector to cv::Mat of size (rows, cols).
	 *
	 * @param inputVector Data to be reshaped.
	 * @param rows Number of rows in the output matrix.
	 * @param cols Number of columns in the output matrix.
	 * @return The reshaped matrix containing data from inputVector.
	 */
	// Function to reconstruct the image from a single-dimensional vector
	cv::Mat reconstructImageFromVector(const std::vector<uint8_t>& flattenedData, int width, int height);

	// Function to flatten the image into a single-dimensional vector
	std::vector<uint8_t> flattenImageToVector(const cv::Mat& image);

	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	void moveMotorTo(int id, int angle);

	// Subscriber to the /image_raw topic that the camera node created.
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_cameraSubscriber;

	// Publisher publishing the motor angle.
	rclcpp::Publisher<Subscription>::SharedPtr m_motorAnglePublisher;

	// Output image that is sent to the publisher_.
	sensor_msgs::msg::Image out_image_;

	// XImge_processor that supports the communication with IP.
	DMA dma;
};


#include "dma_node.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

#define IMG_WIDTH 320.0
#define IMG_HEIGHT 240.0

// This represents the maximal motor rotation.
// 0.29 degrees per 1 impulse.
// Angles 300 - 360 are in invalid range.
#define MOTOR_IMPULZES 1023
#define MOTOR_MAX_RANGE_DEG 300

MinimalSubscriber::MinimalSubscriber()
	: Node("img_subscriber")
{
	// Initialize the XImge_processor the name is obtained by 'cat /sys/class/uio/uiox/name'
	std::cout << "Imge processor initiated\n";

	// Create a publisher that will publish the filtered image with frequency of 10Hz.
	m_motorAnglePublisher = this->create_publisher<SetPosition>("set_position", 10);

	// Create a subscriber to the topic created by the camera node.
	m_cameraSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
		"/image_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
}

MinimalSubscriber::~MinimalSubscriber()
{
	// XImge_processor_Release(&ip_inst);
}

// Function to flatten the image into a single-dimensional vector
std::vector<uint8_t> MinimalSubscriber::flattenImageToVector(const cv::Mat& image)
{
	std::vector<uint8_t> flattenedData;
	if (image.empty()) {
		std::cerr << "Input image is empty." << std::endl;
		return flattenedData;
	}

	// Flatten the image into a single-dimensional vector
	for (int y = 0; y < image.rows; ++y) {
		for (int x = 0; x < image.cols; ++x) {
			flattenedData.push_back(image.at<cv::Vec3b>(y, x)[0]); // Blue channel
			flattenedData.push_back(image.at<cv::Vec3b>(y, x)[1]); // Green channel
			flattenedData.push_back(image.at<cv::Vec3b>(y, x)[2]); // Red channel
		}
	}

	return flattenedData;
}

void MinimalSubscriber::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	// convert ros Image to cv Image.
	std::cout << "encoding: " << msg->encoding << std::endl;
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
	cv::Mat img = cv_ptr->image;

	resize(img, img, cv::Size(IMG_WIDTH, IMG_HEIGHT), cv::INTER_LINEAR);

	std::cout << "Image size: r: " << img.rows << " c: " << img.cols << '\n';

	// Reshape data from 2D representation to 1D vector.
	std::vector<uint8_t> flattenedStdVector = flattenImageToVector(img);
	std::cout << "Flattened size: " << flattenedStdVector.size() << std::endl;

	uint8_t *data = flattenedStdVector.data();
	// filterImage(data, data, 16*16);

	printf("Write data\n");
	auto size = dma.sendData(data, flattenedStdVector.size());
	printf("Written data size: %ud\n", size);
	int output = -1;
	size = dma.getData(&output, 1*sizeof(int));
	printf("Read data size: %ud\n", size);

	switch (outptu) {
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		default:
	}
}

void MinimalSubscriber::moveMotorTo(int id, int angle)
{
	auto msg = SetPosition();
	msg.id = id;
	msg.position = MOTOR_IMPULZES / MOTOR_MAX_RANGE_DEG * angle;

	m_motorAnglePublisher->publish(msg);
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}


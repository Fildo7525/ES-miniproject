#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	// Read the image containing the hexagonal screw
	Mat image = imread("hexagonal_screw_image.jpg"); // Replace with your image file name

	if (image.empty()) {
		cout << "Could not open or find the image." << endl;
		return -1;
	}

	// Convert the image to grayscale
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);

	// Apply GaussianBlur to reduce noise and improve edge detection
	GaussianBlur(gray, gray, Size(5, 5), 0);

	// Use Canny edge detection to find edges
	Mat edges;
	Canny(gray, edges, 50, 150, 3);

	// Find contours in the edge-detected image
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// Iterate through all contours to find the hexagonal shape
	for (size_t i = 0; i < contours.size(); i++) {
		double peri = arcLength(contours[i], true);
		vector<Point> approx;
		approxPolyDP(contours[i], approx, 0.03 * peri, true);

		// Check if the contour has 6 vertices (a hexagon)
		if (approx.size() == 6) {
			// Calculate the orientation of the hexagon
			RotatedRect rect = minAreaRect(contours[i]);
			float angle = rect.angle;
			if (angle < -45) {
				angle += 90;
			}

			// Display the angle of rotation
			cout << "Rotation angle of the hexagonal screw: " << angle << " degrees" << endl;

			// Draw the rotated rectangle
			Point2f vertices[4];
			rect.points(vertices);
			for (int j = 0; j < 4; j++) {
				line(image, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2);
			}

			// Show the detected hexagonal screw
			imshow("Detected Hexagonal Screw", image);
			waitKey(0);
			destroyAllWindows();

			break; // Exit the loop after processing the first detected hexagon
		}
	}

	return 0;
}


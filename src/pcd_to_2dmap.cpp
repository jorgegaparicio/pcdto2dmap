#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

int main(int argc, char** argv)
{
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/grvc/Escritorio/filtered_scans_obstaculos.pcd", *cloud) == -1){
        PCL_ERROR("Couldn't read the PCD file");
        return -1;
    }
    std::cout << "Using default map. To change map, replace the following file, maintaining the name and location:\n /home/grvc/Escritorio/filtered_scans_obstaculos.pcd" << std::endl;

    // Finding map dimensions
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& point : cloud->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
        
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
    }

    float width = max_x - min_x;
    float height = max_y - min_y;
    float depth = max_z - min_z;

    std::cout << "Width: " << width << " meters" << std::endl;
    std::cout << "Height: " << height << " meters" << std::endl;
    std::cout << "Depth: " << depth << " meters" << std::endl;
    std::cout << "xmin: " << min_x << " meters" << std::endl;
    std::cout << "xmax: " << max_x << " meters" << std::endl;
    std::cout << "ymin: " << min_y << " meters" << std::endl;
    std::cout << "ymax: " << max_y << " meters" << std::endl;
    std::cout << "zmin: " << min_z << " meters" << std::endl;
    std::cout << "zmax: " << max_z << " meters" << std::endl;

    // Defining grid parameters
    float grid_resolution = 0.25; // 25cm² per grid cell
    float map_width = width;
    float map_height = height;

    int grid_width = static_cast<int>(map_width / grid_resolution);
    int grid_height = static_cast<int>(map_height / grid_resolution);

    std::cout << "grid w: " << grid_width << " cells" << std::endl;
    std::cout << "grid h: " << grid_height << " cells" << std::endl;

    // Create a 2D grid (vector of vectors) initialized to 0 (free space).
    std::vector<std::vector<int>> obstacle_map(grid_height, std::vector<int>(grid_width, 0));

    // Define height to mark as obstacle
    float height_threshold_min = 0.5; // 0.5 meters height threshold to be considered obstacle
    float height_threshold_max = 5; // above 7.5 meters is considered ceiling

    // Calculate map grid.
    for (const auto& point : cloud->points) {
        if (point.z > height_threshold_min && point.z < height_threshold_max) {
            int x_index = static_cast<int>((point.x + map_width / 2) / grid_resolution);
            int y_index = static_cast<int>((point.y + map_height / 2) / grid_resolution);

            if (x_index >= 0 && x_index < grid_width && y_index >= 0 && y_index < grid_height) {
                obstacle_map[y_index][x_index] = 1; // Mark as obstacle
            }
        }
    }

    // Create an OpenCV matrix to store the image
    cv::Mat image(grid_height, grid_width, CV_8UC1);

    // Convert the binary map to an image
    for (int i = 0; i < grid_height; i++) {
        for (int j = 0; j < grid_width; j++) {
            // Set pixels: 255 for free space (0) and 0 for obstacles (1)
            image.at<uchar>(i, j) = obstacle_map[grid_height-1-i][j] == 0 ? 255 : 0;
        }
    }

    // Optionally, resize the image for better visualization
    // cv::resize(image, image, cv::Size(900,0.832*900), 0, 0, cv::INTER_NEAREST);
    cv::imwrite("obstacle_map.pgm", image);
    std::cout << "Map saved as obstacle_map.pgm" << std::endl;
    // Display the image
    cv::imshow("Obstacle Map", image);
    std::cout << "Displaying map. Press any key to close the window." << std::endl;
    cv::waitKey(0); // Wait for a key press to close the window
    return 0;
}

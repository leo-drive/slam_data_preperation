#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class BagCreatorFromPospac: public rclcpp::Node{
public:
    explicit BagCreatorFromPospac(const std::string & node_name, const rclcpp::NodeOptions & node_options);
private:
    void gnss_reader();    
    void readBag();
    void writeBag();
    std::vector<sensor_msgs::msg::PointCloud2> pc2s;
    std::vector<sensor_msgs::msg::Imu> imu_msgs;
    std::vector<sensor_msgs::msg::NavSatFix> nav_msgs;
    std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> twist_msgs;
    unsigned long int i{0},j{0};
    double roll0,pitch0,yaw0;
    std::string pc_bag_file, output_bag_file, pospac_file;
    std::string imu_frame, navmsg_frame, twist_frame, lidar_frame;
    std::string imu_topic, navmsg_topic, twist_topic, pointcloud_topic;
};
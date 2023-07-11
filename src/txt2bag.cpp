#include "txt2bag.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <chrono>
#include <vector>
#include <fstream>
#include <sstream>

#define NANO pow(10,-9)
#define PRECISION 0.1

// using namespace std::chrono_literals;

double today_seconds_calculator(const sensor_msgs::msg::PointCloud2 msg){
    auto now_sec = std::chrono::seconds(msg.header.stamp.sec);
    auto today_sec = floor<std::chrono::days>(now_sec);
    auto tod_sec = std::chrono::duration_cast<std::chrono::seconds>(now_sec - today_sec);
    auto now_nsec = std::chrono::nanoseconds(msg.header.stamp.nanosec);
    auto today_nsec = floor<std::chrono::days>(now_nsec);
    auto tod_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now_nsec - today_nsec);
    double tod_nsec_rounded=std::round((tod_nsec.count()*NANO)/PRECISION)*PRECISION;
    return (tod_sec.count()+tod_nsec_rounded);
}

BagCreatorFromPospac::BagCreatorFromPospac(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name,node_options),
    pc_bag_file{this->declare_parameter("input_bag_path","data/pointcloud.bag")},
    pospac_file{this->declare_parameter("pospac_file_path","data/postprocessed_gnssins.txt")},
    output_bag_file(this->declare_parameter("output_bag_path","data/pointcloud_pospac.bag")),
    imu_topic{this->declare_parameter("imu_topic","/imu/data")},
    navmsg_topic{this->declare_parameter("navmsg_topic","/navsat/fix")},
    twist_topic{this->declare_parameter("twist_topic","/twist")},
    pointcloud_topic{this->declare_parameter("pontcloud_topic","/points_raw")},
    lidar_frame{this->declare_parameter("lidar_frame","lidar_link")},
    imu_frame{this->declare_parameter("imu_frame","navsat_link")},
    navmsg_frame{this->declare_parameter("navmsg_frame","navsat_link")},
    twist_frame{this->declare_parameter("twist_frame","navsat_link")},
    roll0{this->declare_parameter("roll0",0.0)},
    pitch0{this->declare_parameter("pitch0",0.0)},
    yaw0{this->declare_parameter("yaw0",0.0)}{
    gnss_reader();
    readBag();
}

void BagCreatorFromPospac::readBag(){
    sensor_msgs::msg::PointCloud2 pointcloud;
    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> pc_reader_impl = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    rosbag2_cpp::Reader pc_reader(std::move(pc_reader_impl));
    pc_reader.open(pc_bag_file);

    auto topics_and_types = pc_reader.get_all_topics_and_types();
    for (const auto &topic_and_type: topics_and_types) {
        std::cout << topic_and_type.name << " " << topic_and_type.type << std::endl;
    }

    while(pc_reader.has_next() && rclcpp::ok()){
        auto msg = pc_reader.read_next();
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        rclcpp::SerializedMessage extracted_message(*msg->serialized_data);
        serialization.deserialize_message(&extracted_message, &pointcloud);
        pointcloud.header.frame_id = lidar_frame;
        double time_stamp=today_seconds_calculator(pointcloud);
        double integer;
        double fractional = modf(time_stamp, &integer);
        pointcloud.header.stamp.sec=integer;
        pointcloud.header.stamp.nanosec=fractional*pow(10,9);
        std::cout<<pointcloud.header.stamp.sec<<"."<<pointcloud.header.stamp.nanosec<<std::endl;
        pc2s.push_back(pointcloud);
    }
}

void BagCreatorFromPospac::writeBag(){
    std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_imp = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    rosbag2_cpp::Writer writer_(std::move(writer_imp));
    writer_.open(output_bag_file);

    while((i<pc2s.size() && j<imu_msgs.size()) && rclcpp::ok()){
        if(imu_msgs[j].header.stamp.sec>pc2s[i].header.stamp.sec){
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            rclcpp::SerializedMessage serialized_message;
            serialization.serialize_message(&pc2s[i], &serialized_message);
            writer_.write(serialized_message,pointcloud_topic,"sensor_msgs/msg/PointCloud2",pc2s[i].header.stamp);
            i++;
        }
        else if(imu_msgs[j].header.stamp.sec==pc2s[i].header.stamp.sec && imu_msgs[j].header.stamp.nanosec>pc2s[i].header.stamp.nanosec){
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            rclcpp::SerializedMessage serialized_message;
            serialization.serialize_message(&pc2s[i], &serialized_message);
            writer_.write(serialized_message,pointcloud_topic,"sensor_msgs/msg/PointCloud2",pc2s[i].header.stamp);
            i++;
        }
        else{
            rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
            rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gnss_serialization;
            rclcpp::Serialization<geometry_msgs::msg::TwistWithCovarianceStamped> twist_serialization;
            rclcpp::SerializedMessage imu_serialized_message;
            rclcpp::SerializedMessage gnss_serialized_message;
            rclcpp::SerializedMessage twist_serialized_message;
            gnss_serialization.serialize_message(&nav_msgs[j], &gnss_serialized_message);
            imu_serialization.serialize_message(&imu_msgs[j], &imu_serialized_message);
            twist_serialization.serialize_message(&twist_msgs[j], &twist_serialized_message);
            writer_.write(imu_serialized_message,imu_topic,"sensor_msgs/msg/Imu",imu_msgs[j].header.stamp);
            writer_.write(gnss_serialized_message,navmsg_topic,"sensor_msgs/msg/NavSatFix",nav_msgs[j].header.stamp);
            writer_.write(twist_serialized_message,twist_topic,"geometry_msgs/msg/TwistWithCovarianceStamped",twist_msgs[j].header.stamp);
            j++;
        }
    }
}

void BagCreatorFromPospac::gnss_reader(){
    std::ifstream file{pospac_file};
    std::string line;
    std::string timestamp,lat,lon,alt,roll,pitch,heading,east_vel,north_vel,up_vel,x_ang_rate,y_ang_rate,z_ang_rate,x_acc,y_acc,z_acc,east_rms,north_rms,h_rms,roll_rms,pitcg_rms,heading_rms,unused;
    double d_roll,d_pitch,d_heading;
    sensor_msgs::msg::Imu imu_msg;
    sensor_msgs::msg::NavSatFix nav_msg;
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    double radian=M_PI/180;
    double roll0{-0.777117}, pitch0{-5.432728}, heading0{219.101116};
    tf2::Quaternion q;
    while(std::getline(file,line)){
        std::stringstream iss(line);
        iss>>timestamp>>unused>>unused>>unused>>unused>>lat>>lon>>alt>>roll>>pitch>>heading>>east_vel>>north_vel>>up_vel>>x_ang_rate>>y_ang_rate>>z_ang_rate>>x_acc>>y_acc>>z_acc>>east_rms>>north_rms>>h_rms>>roll_rms>>pitcg_rms>>heading_rms;
        std::stringstream iss2(timestamp);
        std::vector<std::string> sec_nsec;
        std::string buffer;
        while(std::getline(iss2,buffer,'.')){
            sec_nsec.push_back(buffer);
        }
        try{
            imu_msg.header.frame_id=imu_frame;
            nav_msg.header.frame_id=navmsg_frame;
            twist_msg.header.frame_id=twist_frame;
            imu_msg.header.stamp.nanosec=std::stoi(sec_nsec.back())*std::pow(10,3);
            nav_msg.header.stamp.nanosec=std::stoi(sec_nsec.back())*std::pow(10,3);
            twist_msg.header.stamp.nanosec=std::stoi(sec_nsec.back())*std::pow(10,3);
            sec_nsec.pop_back();
            imu_msg.header.stamp.sec=std::stoi(sec_nsec.back());
            nav_msg.header.stamp.sec=std::stoi(sec_nsec.back());
            twist_msg.header.stamp.sec=std::stoi(sec_nsec.back());
            sec_nsec.pop_back();
            nav_msg.latitude=std::stod(lat);
            nav_msg.longitude=std::stod(lon);
            nav_msg.altitude=std::stod(alt);
            nav_msg.position_covariance_type=nav_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
            nav_msg.position_covariance.fill(0.0);
            nav_msg.position_covariance[0]=std::pow(std::stod(north_rms),2);
            nav_msg.position_covariance[4]=std::pow(std::stod(east_rms),2);
            nav_msg.position_covariance[8]=std::pow(std::stod(h_rms),2);
            nav_msgs.push_back(nav_msg);
            twist_msg.twist.twist.linear.x=std::sqrt(std::pow(std::stod(north_vel),2)+std::pow(std::stod(east_vel),2));
            twist_msg.twist.twist.linear.y=0.0;
            twist_msg.twist.twist.linear.z=0.0;
            twist_msg.twist.twist.angular.x=-std::stod(y_ang_rate)*radian;
            twist_msg.twist.twist.angular.y=-std::stod(x_ang_rate)*radian;
            twist_msg.twist.twist.angular.z=-std::stod(z_ang_rate)*radian;
            twist_msg.twist.covariance.fill(0.0);
            twist_msg.twist.covariance[0]  = 0.04;
            twist_msg.twist.covariance[7]  = 10000.0;
            twist_msg.twist.covariance[14] = 10000.0;
            twist_msg.twist.covariance[21] = 0.01;
            twist_msg.twist.covariance[28] = 0.01;
            twist_msg.twist.covariance[35] = 0.01;
            twist_msgs.push_back(twist_msg);
            d_roll=std::stod(roll);
            d_pitch=std::stod(pitch);
            d_heading=std::stod(heading);
            d_roll=d_roll-roll0;
            d_pitch=d_pitch-pitch0;
            d_heading=d_heading-heading0;
            d_roll=(d_roll>180) ? (d_roll-360) : (d_roll);
            d_pitch=(d_pitch>180) ? (d_pitch-360) : (d_pitch);
            d_heading=(d_heading>180) ? (d_heading-360) : (d_heading);
            q.setRPY(-(d_pitch*radian),-(d_roll*radian), -(d_heading*radian));
            imu_msg.angular_velocity.x=-std::stod(y_ang_rate)*radian;
            imu_msg.angular_velocity.y=-std::stod(x_ang_rate)*radian;
            imu_msg.angular_velocity.z=-std::stod(z_ang_rate)*radian;
            imu_msg.linear_acceleration.x=-std::stod(y_acc);
            imu_msg.linear_acceleration.y=-std::stod(x_acc);
            imu_msg.linear_acceleration.z=-(std::stod(z_acc)-9.81);
            tf2::convert(q,imu_msg.orientation);
            imu_msg.orientation_covariance.fill(0.0);
            imu_msg.orientation_covariance[0]=std::pow(std::stod(roll_rms),2);
            imu_msg.orientation_covariance[4]=std::pow(std::stod(pitcg_rms),2);
            imu_msg.orientation_covariance[8]=std::pow(std::stod(heading_rms),2);
            imu_msg.angular_velocity_covariance.fill(0.0);
            imu_msg.angular_velocity_covariance[0]=0.01;
            imu_msg.angular_velocity_covariance[4]=0.01;
            imu_msg.angular_velocity_covariance[8]=0.01;
            imu_msg.linear_acceleration_covariance.fill(0.0);
            imu_msg.linear_acceleration_covariance[0]=0.01;
            imu_msg.linear_acceleration_covariance[4]=0.01;
            imu_msg.linear_acceleration_covariance[8]=0.01;
            // std::cout<<imu_msg.header.stamp.sec<<"."<<imu_msg.header.stamp.nanosec<<std::endl;
            imu_msgs.push_back(imu_msg);
        }
        catch(...){
            continue;
        }
    }
    RCLCPP_WARN(this->get_logger(),"Done!");
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<BagCreatorFromPospac>("BagCreatorFromPospac",options));
    rclcpp::shutdown();
    return 0;
}
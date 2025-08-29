#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <rclcpp/qos.hpp> // 加上這個 include

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher() : Node("map_publisher")
    {	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();  
        // 創建發布者
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapadd", qos);

        // 聲明參數
        this->declare_parameter("map_file", "");
        this->declare_parameter("yaml_file", "");

        // 獲取參數
        std::string map_file = this->get_parameter("map_file").as_string();
        std::string yaml_file = this->get_parameter("yaml_file").as_string();

        if (map_file.empty() || yaml_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide map_file and yaml_file parameters");
            return;
        }

        // 讀取YAML文件
        try {
            YAML::Node config = YAML::LoadFile(yaml_file);
            
            // 創建地圖消息
            auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
            
            // 設置地圖元數據
            map_msg->header.frame_id = "map";
            map_msg->info.resolution = config["resolution"].as<double>();
            
            // 讀取PGM文件
            std::ifstream map_file_stream(map_file, std::ios::binary);
            if (!map_file_stream) {
                RCLCPP_ERROR(this->get_logger(), "Could not open map file: %s", map_file.c_str());
                return;
            }

            // 讀取PGM頭部
            std::string line;
            std::getline(map_file_stream, line); // P5
            std::getline(map_file_stream, line); // 註釋
            std::getline(map_file_stream, line); // 尺寸
            std::stringstream ss(line);
            ss >> map_msg->info.width >> map_msg->info.height;
            std::getline(map_file_stream, line); // 最大值

            // 設置原點
            map_msg->info.origin.position.x = config["origin"][0].as<double>();
            map_msg->info.origin.position.y = config["origin"][1].as<double>();
            map_msg->info.origin.position.z = 0.0;
            map_msg->info.origin.orientation.w = 1.0;

            // 讀取地圖數據
            std::vector<int8_t> map_data(map_msg->info.width * map_msg->info.height);
            map_file_stream.read(reinterpret_cast<char*>(map_data.data()), map_data.size());
            
            // 轉換數據格式
            for (auto& pixel : map_data) {
                if (pixel == 0) {
                    pixel = 100; // 佔用
                } else if (pixel == 254) {
                    pixel = 0;   // 空閒
                } else {
                    pixel = -1;  // 未知
                }
            }

            map_msg->data = map_data;

            // 發布地圖
            publisher_->publish(std::move(map_msg));
            RCLCPP_INFO(this->get_logger(), "Map published successfully");

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
        }
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 

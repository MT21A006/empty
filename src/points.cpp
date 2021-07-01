#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

int i =0;
namespace empty_pcl
{
class AAAA : public rclcpp::Node
{
public:
  explicit AAAA(const rclcpp::NodeOptions & options);

private:
  void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
} 


namespace empty_pcl
{
AAAA::AAAA(const rclcpp::NodeOptions & options)
: Node("empty_pcl", options)
{
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/points", 1,std::bind(&AAAA::pointsCallback, this, std::placeholders::_1));
}
void AAAA::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  i+=1;
  std::cout<<"CALL_BACK : "<<i<<std::endl;
}
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  printf("hello world empty_pcl package\n");
  auto component = std::make_shared<empty_pcl::AAAA>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}

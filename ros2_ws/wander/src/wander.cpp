#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>

using rclcpp::Publisher;
using rclcpp::Subscription;
using rclcpp::TimerBase;

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Twist;
using sensor_msgs::msg::LaserScan;

using std::srand;
using std::bind;
using std::ceil;
using std::min;
using std::make_shared;

using namespace std::literals::chrono_literals;


class Wander : public rclcpp::Node
{
private:
  Subscription<Odometry>::SharedPtr odometrySub;
  Publisher<Twist>::SharedPtr commandPub;
  Subscription<LaserScan>::SharedPtr laserSub;
  TimerBase::SharedPtr timer_;

  double forwardVel;
  double rotateVel;
  double closestRange;

  void commandCallback(const LaserScan::SharedPtr msg)
  {
    int totalValues = static_cast<int>(ceil((msg->angle_max - msg->angle_min) / msg->angle_increment));
    totalValues = min(totalValues, static_cast<int>(msg->ranges.size()));

    for(int i=0; i < totalValues; i++) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Values[" << i << "]: " << msg->ranges[i]);
    }
  }

  void timerCallback() {
    auto msg = Twist();
    msg.linear.x = forwardVel;
    msg.angular.z = rotateVel;
    commandPub->publish(msg);
  }

  void commandOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    double angle = 2 * atan2(q_z, q_w);

    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Odometry -> x: " << msg->pose.pose.position.x << " | y: " << msg->pose.pose.position.y << " | ang: " << angle);
  }

public:
  Wander() : Node("wander_node") {
    forwardVel = 1.0;
    rotateVel = 0.0;
    closestRange = 0.0;
    
    srand(std::time(nullptr));
    commandPub = this->create_publisher<Twist>("cmd_vel", 1);
    laserSub = this->create_subscription<LaserScan>("base_scan", 1, bind(&Wander::commandCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, bind(&Wander::timerCallback, this));
    odometrySub = this->create_subscription<Odometry>("odom", 1, bind(&Wander::commandOdom, this, std::placeholders::_1));
  }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Wander>());
  rclcpp::shutdown();
  return 0;
}

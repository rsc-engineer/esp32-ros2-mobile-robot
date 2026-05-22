#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

static double norm_angle(double a){
  while (a > M_PI)  a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

class CarrotNode : public rclcpp::Node
{
public:
  CarrotNode() : Node("carrot_node")
  {
    // ---- Parámetros (los puedes cambiar con ros2 param) ----
    lookahead_ = this->declare_parameter<double>("lookahead", 0.25);   // metros
    v_cmd_     = this->declare_parameter<double>("v", 0.10);           // m/s
    w_max_     = this->declare_parameter<double>("w_max", 1.5);        // rad/s
    k_w_       = this->declare_parameter<double>("k_w", 2.0);          // ganancia giro
    tol_       = this->declare_parameter<double>("tol", 0.05);         // m

    // Ruta: rectángulo 2x3 losas (0.45m cada losa)
    double LOSA = 0.45;
    double A = 2.0 * LOSA; // 0.90
    double B = 3.0 * LOSA; // 1.35

    path_ = {
      {A, 0.0},
      {A, B},
      {0.0, B},
      {0.0, 0.0},
      {0.0, 0.0} 
    };

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/pose2d", 10,
      std::bind(&CarrotNode::pose_cb, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CarrotNode::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "CarrotNode listo. Esperando /pose2d ...");
  }

private:
  void pose_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg){
    pose_ = *msg;
    have_pose_ = true;
  }

  void control_loop(){
    if(!have_pose_) return;

    if(idx_ >= path_.size()){
      stop();
      RCLCPP_INFO(this->get_logger(), "Ruta terminada.");
      return;
    }

    // objetivo actual (waypoint)
    double gx = path_[idx_].first;
    double gy = path_[idx_].second;

    double dx = gx - pose_.x;
    double dy = gy - pose_.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    // Si llego al waypoint -> siguiente
    if(dist < tol_){
      idx_++;
      stop();
      return;
    }

    double L = std::min(lookahead_, dist);
    double ux = dx / dist;
    double uy = dy / dist;
    double cx = pose_.x + ux * L;
    double cy = pose_.y + uy * L;

    // Control hacia carrot
    double ang_to_carrot = std::atan2(cy - pose_.y, cx - pose_.x);
    double e_th = norm_angle(ang_to_carrot - pose_.theta);

    geometry_msgs::msg::Twist cmd;

    // Avanza siempre hacia delante, pero reduce si estás muy girada
    double v = v_cmd_;
    if(std::fabs(e_th) > 0.8) v = 0.0; // si estás muy mal orientada, primero gira

    double w = k_w_ * e_th;
    if(w >  w_max_) w =  w_max_;
    if(w < -w_max_) w = -w_max_;

    cmd.linear.x  = v;
    cmd.angular.z = w;

    pub_cmd_->publish(cmd);
  }

  void stop(){
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub_cmd_->publish(cmd);
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Estado
  geometry_msgs::msg::Pose2D pose_;
  bool have_pose_ = false;

  std::vector<std::pair<double,double>> path_;
  size_t idx_ = 0;

  // Parámetros
  double lookahead_, v_cmd_, w_max_, k_w_, tol_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarrotNode>());
  rclcpp::shutdown();
  return 0;
}


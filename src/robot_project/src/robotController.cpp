#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>

class RobotController : public rclcpp::Node
{
public:
    RobotController()
        : Node("robot_controller"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // Crear un suscriptor para el tópico cmd_vel
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobotController::cmd_vel_callback, this, std::placeholders::_1));

        // Crear un broadcaster para las transformaciones
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Iniciar el temporizador para calcular actualizaciones de posición
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotController::update_position, this));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Guardar las velocidades lineales y angulares
        linear_velocity_ = msg->linear.x;  // Velocidad lineal en m/s
        angular_velocity_ = msg->angular.z; // Velocidad angular en rad/s
    }

    void update_position()
    {
        // Calcular el tiempo transcurrido desde la última actualización
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Actualizar posición y orientación usando el modelo cinemático
        x_ += linear_velocity_ * cos(theta_) * dt;
        y_ += linear_velocity_ * sin(theta_) * dt;
        theta_ += angular_velocity_ * dt;

        // Crear y publicar la transformación
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "world"; // Frame de referencia
        transform.child_frame_id = "base_link"; // Frame del robot

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, sin(theta_ / 2), cos(theta_ / 2)));

        tf_broadcaster_->sendTransform(transform);
    }

    // Variables internas para posición y orientación
    double x_, y_, theta_; // Posición y orientación del robot
    double linear_velocity_ = 0.0; // Velocidad lineal actual
    double angular_velocity_ = 0.0; // Velocidad angular actual

    rclcpp::Time last_time_ = this->get_clock()->now();

    // Suscriptor y broadcaster
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Temporizador
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

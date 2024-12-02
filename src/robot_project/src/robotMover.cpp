#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotMover : public rclcpp::Node
{
public:
    RobotMover()
        : Node("robot_mover")
    {
        // Crea un publicador para el tema /cmd_vel (mensajes Twist)
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Crea un temporizador para publicar cada 0.1 segundos
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotMover::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Crea un mensaje Twist
        auto twist_msg = geometry_msgs::msg::Twist();

        // Define la velocidad lineal en x y angular en z
        twist_msg.linear.x = 1.1;  // Velocidad lineal en m/s
        twist_msg.angular.z = 0.3; // Velocidad angular en rad/s

        // Publica el mensaje
        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(),
                    "Publicando velocidad lineal x: %.2f y velocidad angular z: %.2f",
                    twist_msg.linear.x, twist_msg.angular.z);
    }

    // Declaración del publicador y temporizador
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // Inicializa ROS2
    rclcpp::init(argc, argv);

    // Crea el nodo y ejecuta el bucle principal
    auto node = std::make_shared<RobotMover>();
    rclcpp::spin(node);

    // Apaga ROS2
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelRelay : public rclcpp::Node
{
public:
    CmdVelRelay()
        : Node("cmd_vel_turtle")
    {
        // Crea un suscriptor para recibir datos de /cmd_vel
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelRelay::cmd_vel_callback, this, std::placeholders::_1));

        // Crea un publicador para enviar datos a turtle1/cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo inicializado para reenviar velocidades de /cmd_vel a turtle1/cmd_vel");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Publica el mensaje recibido directamente en turtle1/cmd_vel
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(),
                    "Reenviando - Velocidad lineal x: %.2f, angular z: %.2f",
                    msg->linear.x, msg->angular.z);
    }

    // Declaración del suscriptor y publicador
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    // Inicializa ROS2
    rclcpp::init(argc, argv);

    // Crea el nodo y ejecuta el bucle principal
    auto node = std::make_shared<CmdVelRelay>();
    rclcpp::spin(node);

    // Apaga ROS2
    rclcpp::shutdown();
    return 0;
}

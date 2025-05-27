#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals; // for 2s duration literal
using rclcpp::Subscription;
using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::State;
using std_msgs::msg::String;
using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class SimpleLifecycleNode : public LifecycleNode 
{
public:
    explicit SimpleLifecycleNode(const std::string &node_name, bool intra_process_comms = false)
    : LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}

    CallbackReturn on_configure(const State &) override
    {
        sub_ = create_subscription<String>(
            "chatter", 10, std::bind(&SimpleLifecycleNode::msg_callback, this, _1));
        RCLCPP_INFO(get_logger(), "Lifecycle node on_configure is called");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const State & state) override
    {
        LifecycleNode::on_activate(state);  // call base class method
        RCLCPP_INFO(get_logger(), "Lifecycle node on_activate is called");
        std::this_thread::sleep_for(2s);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const State & state) override
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "Lifecycle node on_deactivate is called");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const State &) override
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "Lifecycle node on_cleanup is called");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const State &) override
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "Lifecycle node on_shutdown is called");
        return CallbackReturn::SUCCESS;
    }

private:
    Subscription<String>::SharedPtr sub_;

    void msg_callback(const String::SharedPtr msg)
    {
        auto state = get_current_state();
        if (state.label() == "active")
        {
            RCLCPP_INFO(get_logger(), "Lifecycle node heard: %s", msg->data.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto simple_lifecycle_node = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(simple_lifecycle_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
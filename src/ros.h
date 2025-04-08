#pragma once

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <functional>
#include <string>
#include <vector>

// Prevent RCCHECK redefinition
#ifndef RCCHECK
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#endif

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

// Define a helper macro for message type support
#define UROS_GET_MSG_TYPE_SUPPORT(pkg, subfolder, name) \
  ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, subfolder, name)

namespace uros
{

enum class State
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

// Forward declarations
class Node;
class TimerBase;
class PublisherBase;

/**
 * @brief Support class that manages ROS 2 resources for microROS.
 *
 * This class handles initialization and management of ROS 2 entities including
 * the context, allocator, support structure and executor. It serves as the central
 * point for managing nodes and providing access to shared ROS 2 resources.
 */
class Support
{
public:
  Support();
  ~Support();

  void initialize();
  void spin();
  State get_state() const;

  // Register a node with this support instance
  void register_node(Node* node);
  // Access to context for timer initialization
  rcl_context_t* get_context();
  // Access to executor for adding timers
  rclc_executor_t* get_executor();

private:
  bool initialize_entities();
  void cleanup_entities();

  State state_;
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rclc_executor_t executor_;
  std::vector<Node*> nodes_;

  friend class Node;
};

/**
 * @brief Base class for timer implementations in the ROS 2 system.
 *
 * TimerBase provides a foundation for creating periodic timers that execute callbacks
 * at specified intervals. This is an abstract class that requires derived classes to
 * implement the initialization logic and callback functionality.
 *
 * @note Timer objects must be initialized before use and should be cleaned up when no longer
 * needed.
 */
class TimerBase
{
public:
  TimerBase(Node& node, uint32_t period_ms);
  virtual ~TimerBase();

  virtual bool initialize() = 0;
  virtual void callback() = 0;
  bool is_initialized() const
  {
    return initialized_;
  }

  // Cleanup resources
  void cleanup();

  // Store the timer instance pointer separately
  static void* get_timer_user_ptr(const rcl_timer_t* timer);
  static void set_timer_user_ptr(rcl_timer_t* timer, void* user_ptr);

protected:
  Node& node_;
  uint32_t period_ms_;
  bool initialized_;
  rcl_timer_t timer_;
  rcl_clock_t* clock_ = nullptr;  // Store the clock pointer

  friend class Node;
  friend class Support;
};

/**
 * @brief Base class for ROS2 publishers.
 *
 * This abstract class provides the common functionality and interface for all publishers
 * in the system. It manages the underlying rcl_publisher_t resource and provides
 * initialization, cleanup, and status checking methods.
 *
 * Derived classes must implement the initialize() method to configure their specific
 * message type and publisher settings.
 */
class PublisherBase
{
public:
  PublisherBase(Node& node, const char* topic_name);
  virtual ~PublisherBase();

  virtual bool initialize() = 0;
  bool is_initialized() const
  {
    return initialized_;
  }

  // Cleanup resources
  void cleanup();

protected:
  Node& node_;
  std::string topic_name_;
  bool initialized_;
  rcl_publisher_t publisher_;

  friend class Node;
  friend class Support;
};

/**
 * @brief A ROS2 Node implementation.
 *
 * This class represents a ROS2 node, which is a fundamental element in the ROS2 architecture.
 * It provides functionality for initialization, cleanup, and managing node entities such as
 * timers and publishers.
 */
class Node
{
public:
  Node(Support& support, const char* name, const char* namespace_ = "");
  ~Node();

  rcl_node_t* get_handle();
  bool is_initialized() const
  {
    return initialized_;
  }

  // Initialize node and its entities
  bool initialize();

  // Add a timer to this node
  void add_timer(TimerBase* timer);

  // Initialize timers if node is ready
  bool initialize_timers();

  // Add a publisher to this node
  void add_publisher(PublisherBase* publisher);

  // Initialize publishers if node is ready
  bool initialize_publishers();

  // Cleanup node resources
  void cleanup();

  // Access to support
  Support& get_support()
  {
    return support_;
  }

private:
  Support& support_;
  rcl_node_t node_;
  std::string node_name_;
  std::string node_namespace_;
  bool initialized_;
  std::vector<TimerBase*> timers_;
  std::vector<PublisherBase*> publishers_;

  friend class Support;
  friend class TimerBase;
};

/**
 * @brief A timer class that executes a user-defined callback at regular intervals.
 *
 * Timer extends TimerBase and provides a convenient way to schedule periodic tasks
 * with a std::function callback. The timer is associated with a Node and executes
 * the callback at the specified interval.
 */
class Timer : public TimerBase
{
public:
  using CallbackType = std::function<void()>;

  Timer(Node& node, uint32_t period_ms, CallbackType callback);
  ~Timer() override;

  bool initialize() override;
  void callback() override;

private:
  CallbackType callback_;
};

template <typename MessageT>
/**
 * @brief A templated ROS 2 publisher class to publish specific message types.
 *
 * The Publisher class provides functionality to create and manage a ROS 2 publisher
 * for a specific message type. It handles message initialization, publication,
 * and proper cleanup of resources.
 *
 * @tparam MessageT The ROS 2 message type that this publisher will publish.
 *
 * @note This class inherits from PublisherBase and implements its virtual functions.
 * @note The publisher is not initialized upon construction. You must call initialize()
 *       before publishing messages.
 */
class Publisher : public PublisherBase
{
public:
  Publisher(Node& node,
            const char* topic_name,
            const rosidl_message_type_support_t* type_support,
            const rmw_qos_profile_t& qos_profile = rmw_qos_profile_default)
    : PublisherBase(node, topic_name), type_support_(type_support), qos_profile_(qos_profile)
  {
    publisher_ = rcl_get_zero_initialized_publisher();
    // Initialize message with defaults
    memset(&message_, 0, sizeof(MessageT));
  }

  ~Publisher() override
  {
    cleanup();
  }

  bool initialize() override
  {
    if (!node_.is_initialized() || initialized_)
    {
      return false;
    }

    // Create publisher options with QoS
    rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
    pub_opts.qos = qos_profile_;

    // Initialize publisher using the provided type support
    rcl_ret_t ret = rcl_publisher_init(
      &publisher_, node_.get_handle(), type_support_, topic_name_.c_str(), &pub_opts);

    initialized_ = (ret == RCL_RET_OK);
    return initialized_;
  }

  // Get reference to message for filling
  MessageT& get_message()
  {
    return message_;
  }

  // Set message data directly
  void set_message(const MessageT& msg)
  {
    message_ = msg;
  }

  // Publish message
  bool publish()
  {
    if (!initialized_)
    {
      return false;
    }

    rcl_ret_t ret = rcl_publish(&publisher_, &message_, nullptr);
    return (ret == RCL_RET_OK);
  }

private:
  MessageT message_;
  const rosidl_message_type_support_t* type_support_;
  rmw_qos_profile_t qos_profile_;
};

}  // namespace uros

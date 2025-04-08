#include "ros.h"

#include <Arduino.h>
#include <rcutils/allocator.h>

#include <unordered_map>
static std::unordered_map<const rcl_timer_t*, void*> timer_user_data;

namespace uros
{

Support::Support() : state_(State::WAITING_AGENT)
{
  allocator_ = rcl_get_default_allocator();
}

Support::~Support()
{
  cleanup_entities();
}

rcl_context_t* Support::get_context()
{
  return &support_.context;
}

rclc_executor_t* Support::get_executor()
{
  return &executor_;
}

bool Support::initialize_entities()
{
  allocator_ = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));

  executor_ = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 10, &allocator_));

  for (auto* node : nodes_)
  {
    if (!node->initialize())
    {
      return false;
    }
  }

  return true;
}

void Support::cleanup_entities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor_);

  for (auto* node : nodes_)
  {
    if (node->initialized_)
    {
      node->cleanup();
    }
  }

  rclc_support_fini(&support_);
}

void Support::initialize()
{
  state_ = State::WAITING_AGENT;
}

void Support::spin()
{
  switch (state_)
  {
    case State::WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(1000, 1))
                                    ? State::AGENT_AVAILABLE
                                    : State::WAITING_AGENT;);
      break;
    case State::AGENT_AVAILABLE:
      state_ = (true == initialize_entities()) ? State::AGENT_CONNECTED : State::WAITING_AGENT;
      if (state_ == State::WAITING_AGENT)
      {
        cleanup_entities();
      };

      rmw_uros_sync_session(1000);  // ms

      break;
    case State::AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(1000, 3))
                                    ? State::AGENT_CONNECTED
                                    : State::AGENT_DISCONNECTED;);
      if (state_ == State::AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
      }
      break;
    case State::AGENT_DISCONNECTED:
      cleanup_entities();
      state_ = State::WAITING_AGENT;
      break;
    default:
      break;
  }
}

State Support::get_state() const
{
  return state_;
}

void Support::register_node(Node* node)
{
  nodes_.push_back(node);
}

// Helper functions for timer user data
void* TimerBase::get_timer_user_ptr(const rcl_timer_t* timer)
{
  auto it = timer_user_data.find(timer);
  if (it != timer_user_data.end())
  {
    return it->second;
  }
  return nullptr;
}

void TimerBase::set_timer_user_ptr(rcl_timer_t* timer, void* user_ptr)
{
  timer_user_data[timer] = user_ptr;
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  TimerBase* timer_obj = static_cast<TimerBase*>(TimerBase::get_timer_user_ptr(timer));
  if (timer_obj != nullptr)
  {
    timer_obj->callback();
  }
}

void TimerBase::cleanup()
{
  if (initialized_)
  {
    rcl_ret_t ret = rcl_timer_fini(&timer_);
    (void)ret;  // Suppress warning

    if (clock_ != nullptr)
    {
      rcl_ret_t clock_ret = rcl_clock_fini(clock_);
      (void)clock_ret;  // Suppress warning
      delete clock_;
      clock_ = nullptr;
    }

    timer_user_data.erase(&timer_);
    initialized_ = false;
  }
}

TimerBase::TimerBase(Node& node, uint32_t period_ms)
  : node_(node), period_ms_(period_ms), initialized_(false), clock_(nullptr)
{
  timer_ = rcl_get_zero_initialized_timer();
  node.add_timer(this);
}

TimerBase::~TimerBase()
{
  cleanup();
}

Timer::Timer(Node& node, uint32_t period_ms, CallbackType callback)
  : TimerBase(node, period_ms), callback_(callback)
{
}

Timer::~Timer()
{
  // Base class handles timer cleanup
}

bool Timer::initialize()
{
  if (!node_.is_initialized() || initialized_)
  {
    return false;
  }

  rcl_context_t* context = node_.get_support().get_context();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_clock_type_t clock_type = RCL_STEADY_TIME;
  clock_ = new rcl_clock_t;  // Store in the base class member
  rcl_ret_t ret = rcl_clock_init(clock_type, clock_, &allocator);
  if (ret != RCL_RET_OK)
  {
    delete clock_;
    clock_ = nullptr;
    return false;
  }

  ret = rcl_timer_init2(
    &timer_, clock_, context, RCL_MS_TO_NS(period_ms_), timer_callback, allocator, true);

  if (ret != RCL_RET_OK)
  {
    rcl_ret_t fini_ret = rcl_clock_fini(clock_);
    (void)fini_ret;  // Suppress warning
    delete clock_;
    clock_ = nullptr;
    return false;
  }

  set_timer_user_ptr(&timer_, this);

  ret = rclc_executor_add_timer(node_.get_support().get_executor(), &timer_);
  initialized_ = (ret == RCL_RET_OK);

  return initialized_;
}

void Timer::callback()
{
  if (callback_)
  {
    callback_();
  }
}

void PublisherBase::cleanup()
{
  if (initialized_)
  {
    rcl_ret_t ret = rcl_publisher_fini(&publisher_, node_.get_handle());
    (void)ret;  // Suppress warning
    initialized_ = false;
  }
}

PublisherBase::PublisherBase(Node& node, const char* topic_name)
  : node_(node), topic_name_(topic_name), initialized_(false)
{
  publisher_ = rcl_get_zero_initialized_publisher();
  node.add_publisher(this);
}

PublisherBase::~PublisherBase()
{
  cleanup();
}

Node::Node(Support& support, const char* name, const char* namespace_)
  : support_(support), node_name_(name), node_namespace_(namespace_), initialized_(false)
{
  node_ = rcl_get_zero_initialized_node();
  support.register_node(this);
}

Node::~Node()
{
  cleanup();

  for (auto* timer : timers_)
  {
    delete timer;
  }
  timers_.clear();

  for (auto* publisher : publishers_)
  {
    delete publisher;
  }
  publishers_.clear();
}

void Node::cleanup()
{
  if (initialized_)
  {
    for (auto* publisher : publishers_)
    {
      publisher->cleanup();
    }

    for (auto* timer : timers_)
    {
      timer->cleanup();
    }

    rcl_ret_t ret = rcl_node_fini(&node_);
    (void)ret;  // Suppress warning
    initialized_ = false;
  }
}

rcl_node_t* Node::get_handle()
{
  return &node_;
}

void Node::add_timer(TimerBase* timer)
{
  timers_.push_back(timer);

  if (initialized_)
  {
    timer->initialize();
  }
}

bool Node::initialize_timers()
{
  if (!initialized_)
  {
    return false;
  }

  bool success = true;
  for (auto* timer : timers_)
  {
    success &= timer->initialize();
  }

  return success;
}

void Node::add_publisher(PublisherBase* publisher)
{
  publishers_.push_back(publisher);

  if (initialized_)
  {
    publisher->initialize();
  }
}

bool Node::initialize_publishers()
{
  if (!initialized_)
  {
    return false;
  }

  bool success = true;
  for (auto* publisher : publishers_)
  {
    success &= publisher->initialize();
  }

  return success;
}

bool Node::initialize()
{
  if (initialized_)
  {
    return true;  // Already initialized
  }

  rcl_node_t* handle = get_handle();
  rcl_ret_t ret =
    rclc_node_init_default(handle, node_name_.c_str(), node_namespace_.c_str(), &support_.support_);

  if (ret != RCL_RET_OK)
  {
    return false;
  }

  initialized_ = true;

  bool timers_ok = initialize_timers();
  bool publishers_ok = initialize_publishers();

  return timers_ok && publishers_ok;
}

}  // namespace uros

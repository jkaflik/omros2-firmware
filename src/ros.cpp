#include "ros.h"

#include <Arduino.h>

// Define the state variable here
states state;

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "openmower_mainboard", "", &support));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  return true;
}

void destroy_entities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void initialize()
{
  state = WAITING_AGENT;
}

void spin()
{
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(
        500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT)
      {
        destroy_entities();
      };

      rmw_uros_sync_session(1000);  // ms

      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 3))
                                   ? AGENT_CONNECTED
                                   : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}
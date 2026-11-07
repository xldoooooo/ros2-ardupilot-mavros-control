/** @file publish_delay.cpp
 * @brief One-shot DDS boundary delays for localhost native-node regression tests only.
 */
#include <rcl/rcl.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <thread>
#include <unistd.h>

/** Record an injection boundary for independent ordering assertions. */
void mark(const char * root, const char * suffix)
{
  const std::string path = std::string(root) + suffix;
  if (auto * file = std::fopen(path.c_str(), "w")) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    std::fprintf(file, "%lld\n", static_cast<long long>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()));
    std::fclose(file);
  }
}

/** Delay only a selected fictitious publisher in an explicitly isolated test domain. */
extern "C" rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher, const void * message,
  rmw_publisher_allocation_t * allocation)
{
  using Publish = rcl_ret_t (*)(const rcl_publisher_t *, const void *, rmw_publisher_allocation_t *);
  using Topic = const char * (*)(const rcl_publisher_t *);
  static auto original = reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rcl_publish"));
  static auto topic_name = reinterpret_cast<Topic>(dlsym(RTLD_NEXT, "rcl_publisher_get_topic_name"));
  static std::atomic<bool> injected{false};
  const char * trigger = std::getenv("ONBOARD_TEST_TRIGGER");
  const char * selected = std::getenv("ONBOARD_TEST_TOPIC");
  const char * isolation = std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  const char * topic = topic_name(publisher);
  const bool permitted = topic &&
    (std::strcmp(topic, "/avoidance_executor_double_mavros/setpoint_raw/attitude") == 0 ||
    std::strcmp(topic, "/mavros/local_position/pose") == 0 ||
    std::strcmp(topic, "/avoidance_executor_double_onboard/tracking_sample") == 0 ||
    std::strcmp(topic, "/avoidance_executor_double_onboard/status") == 0);
  if (trigger && selected && isolation && domain && permitted &&
    std::strcmp(isolation, "LOCALHOST") == 0 && std::atoi(domain) >= 150 &&
    std::strcmp(topic, selected) == 0 && access(trigger, F_OK) == 0 &&
    !injected.exchange(true, std::memory_order_relaxed))
  {
    mark(trigger, ".entered");
    const auto delay = std::chrono::milliseconds(std::atoi(std::getenv("ONBOARD_TEST_DELAY_MS")));
    const char * mode = std::getenv("ONBOARD_TEST_DELAY_MODE");
    if (mode && std::strcmp(mode, "busy") == 0) {
      const auto end = std::chrono::steady_clock::now() + delay;
      while (std::chrono::steady_clock::now() < end) {}
    } else {
      std::this_thread::sleep_for(delay);
    }
    const auto result = original(publisher, message, allocation);
    mark(trigger, ".returned");
    return result;
  }
  if (trigger && topic &&
    std::strcmp(topic, "/avoidance_executor_double_mavros/setpoint_raw/attitude") == 0 &&
    access((std::string(trigger) + ".land_sent").c_str(), F_OK) == 0)
  {
    mark(trigger, ".late_attitude");
  }
  return original(publisher, message, allocation);
}

/** Observe the native SetMode send boundary, avoiding cross-topic receiver scheduling ambiguity. */
extern "C" rcl_ret_t rcl_send_request(
  const rcl_client_t * client, const void * request, int64_t * sequence_number)
{
  using Send = rcl_ret_t (*)(const rcl_client_t *, const void *, int64_t *);
  using Service = const char * (*)(const rcl_client_t *);
  static auto original = reinterpret_cast<Send>(dlsym(RTLD_NEXT, "rcl_send_request"));
  static auto service_name = reinterpret_cast<Service>(dlsym(RTLD_NEXT, "rcl_client_get_service_name"));
  const char * trigger = std::getenv("ONBOARD_TEST_TRIGGER");
  const char * service = service_name(client);
  if (trigger && service &&
    std::strcmp(service, "/avoidance_executor_double_mavros/set_mode") == 0 &&
    access((std::string(trigger) + ".entered").c_str(), F_OK) == 0)
  {
    // This fixture is already GUIDED/armed; its only mode request after injection is LAND.
    mark(trigger, ".land_sent");
  }
  return original(client, request, sequence_number);
}

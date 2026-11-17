// Exercise actual ROS action state transitions without any robot or controller.
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

using namespace std::chrono_literals;
using Action = control_msgs::action::FollowJointTrajectory;
using RTGoal = realtime_tools::RealtimeServerGoalHandle<Action>;
using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
#ifdef TEST_LIBRARY_CALLBACK
// Verify the callback exported by each built library, not a test-local copy.
extern template void realtime_tools::RealtimeServerGoalHandle<Action>::runNonRealtime();
#endif

template<class Future>
bool wait(rclcpp::Executor & executor, Future & future, std::chrono::milliseconds timeout = 1000ms)
{
  auto end = std::chrono::steady_clock::now() + timeout;
  while (future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < end) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  return future.wait_for(0ms) == std::future_status::ready;
}

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv)
{
  // Refuse the production domain even if launched incorrectly.
  require(std::getenv("ROS_DOMAIN_ID") && std::string(std::getenv("ROS_DOMAIN_ID")) == "183",
          "Run only in isolated ROS_DOMAIN_ID=183");
  const bool unpatched = argc > 1 && std::string(argv[1]) == "--verify-unpatched";
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("axisforge_goal_race_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::shared_ptr<RTGoal> rt;
  int scenario = 0;
  auto success = std::make_shared<Action::Result>();
  auto failure = std::make_shared<Action::Result>();
  failure->error_code = Action::Result::PATH_TOLERANCE_VIOLATED;
  auto server = rclcpp_action::create_server<Action>(node, "isolated_goal_completion",
    [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [&](auto) {
      // Deterministically recreate the RT-complete / NRT-result gap.
      if (scenario == 2) rt->setSucceeded(success);
      if (scenario == 3) rt->setAborted(failure);
      rt->setCanceled(success);
      if (scenario == 4) rt->setSucceeded(success);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](std::shared_ptr<ServerGoal> handle) {
      rt = std::make_shared<RTGoal>(handle);
      rt->execute();
    });
  auto client = rclcpp_action::create_client<Action>(node, "isolated_goal_completion");
  require(client->wait_for_action_server(3s), "Action server unavailable");
  int checks = 0;
  for (int cycle = 0; cycle < (unpatched ? 1 : 30); ++cycle) {
    for (scenario = unpatched ? 2 : 0; scenario < (unpatched ? 3 : 6); ++scenario) {
      auto sent = client->async_send_goal(Action::Goal());
      require(wait(executor, sent), "Goal acceptance timeout");
      auto goal = sent.get();
      require(goal != nullptr && rt != nullptr, "Goal rejected");
      auto result = client->async_get_result(goal);
      if (scenario == 0) rt->setSucceeded(success);
      else if (scenario == 5) rt->setAborted(failure);
      else {
        auto cancelled = client->async_cancel_goal(goal);
        require(wait(executor, cancelled), "Cancellation response timeout");
        require(cancelled.get()->return_code == 0 && rt->gh_->is_canceling(), "Cancellation not accepted");
      }
      rt->runNonRealtime();
      if (unpatched) {
        require(!wait(executor, result, 150ms) && rt->gh_->is_canceling(), "Expected race did not reproduce");
        // Clean up this isolated test goal; this is never a hardware endpoint.
        rt->gh_->abort(failure);
        require(wait(executor, result), "Test cleanup failed");
        std::cout << "REPRODUCED: queued success stranded in CANCELING\n";
        continue;
      }
      require(wait(executor, result), "Terminal result timeout");
      auto response = result.get();
      auto expected = (scenario == 0 || scenario == 2) ? rclcpp_action::ResultCode::SUCCEEDED :
        ((scenario == 3 || scenario == 5) ? rclcpp_action::ResultCode::ABORTED : rclcpp_action::ResultCode::CANCELED);
      require(response.code == expected, "Incorrect terminal outcome");
      require(response.result->error_code == ((scenario == 3 || scenario == 5) ? -4 : 0), "Result payload changed");
      require(!rt->gh_->is_active(), "Terminal goal still active");
      rt->runNonRealtime(); // A second timer tick must be harmless.
      ++checks;
    }
  }
  std::cout << "Passed " << checks << " completion/cancellation checks\n";
  rclcpp::shutdown();
}

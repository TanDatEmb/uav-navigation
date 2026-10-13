// The lio process (SYSTEM_DESIGN §3.5, §6.4; S1b runtime design): one LioNode on one SingleThreadedExecutor,
// whose thread is the ingest thread.
//
// Signals: rclcpp::init installs rclcpp's SIGINT/SIGTERM handling (SignalHandlerOptions::All), which shuts the
// context down. The context first runs its pre-shutdown callbacks, so LioNode::shutdown() runs while the context
// is still valid: the node stops taking callbacks, joins the backend and drains the recorder, and the final
// records (LioNodeSummary, IngestSummary) still reach /events as well as the JSONL file. The executor's spin()
// then returns. shutdown() is idempotent, so the call after spin() only matters when spin() ended otherwise.
#include <cstdlib>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uavnav/lio_node/lio_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<uavnav::lio_node::LioNode> node;
  try {
    node = std::make_shared<uavnav::lio_node::LioNode>(rclcpp::NodeOptions());
  } catch (const uavnav::lio_node::LioNodeStartFailed&) {
    rclcpp::shutdown();  // the node logged its one RCLCPP_FATAL
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("lio"), "lio refuses to start: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const rclcpp::Context::SharedPtr context = node->get_node_base_interface()->get_context();
  uavnav::lio_node::LioNode* const raw = node.get();
  const auto hook = context->add_pre_shutdown_callback([raw] { raw->shutdown(); });

  int rc = EXIT_SUCCESS;
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    try {
      executor.spin();
    } catch (const std::exception& e) {  // a callback threw: the node emitted LioNodeFailed
      RCLCPP_FATAL(node->get_logger(), "lio executor failed: %s", e.what());
      rc = EXIT_FAILURE;
    } catch (...) {
      RCLCPP_FATAL(node->get_logger(), "lio executor failed: unknown exception");
      rc = EXIT_FAILURE;
    }
  }
  node->shutdown();  // the executor has stopped
  context->remove_pre_shutdown_callback(hook);
  node.reset();
  rclcpp::shutdown();
  return rc;
}

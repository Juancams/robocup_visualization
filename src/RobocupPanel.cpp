// Copyright 2024 Juan Carlos Manzanares Serrano
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robocup_visualization/RobocupPanel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace robocup_visualization
{

RobocupPanel::RobocupPanel(QWidget * parent)
: Panel(parent)
{
  node_ = rclcpp::Node::make_shared("robocup_panel");

  spin_thread_ = std::thread(
    [this]() {
      rclcpp::spin(node_);
    });
}

RobocupPanel::~RobocupPanel()
{
  spin_thread_.join();
}

void
RobocupPanel::onInitialize()
{
}

}  // namespace robocup_visualization

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(robocup_visualization::RobocupPanel, rviz_common::Panel)

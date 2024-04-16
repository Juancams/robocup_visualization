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

#include "robocup_visualization/RobocupRobotState.hpp"

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

RobocupRobotState::RobocupRobotState(QWidget * parent)
: Panel(parent)
{
  node_ = rclcpp::Node::make_shared("robocup_robot_state");

  tab_widget_ = new QTabWidget();
  layout_ = new QVBoxLayout();

  QWidget * panel_widget = new QWidget();
  QVBoxLayout * panel_layout = new QVBoxLayout(panel_widget);

  image_label_ = new QLabel();
  listen_image_path_ = ament_index_cpp::get_package_share_directory("robocup_visualization") +
    "/images/listen.png";
  speak_image_path_ = ament_index_cpp::get_package_share_directory("robocup_visualization") +
    "/images/speak.png";
  think_image_path_ = ament_index_cpp::get_package_share_directory("robocup_visualization") +
    "/images/think.png";
  image_label_->setAlignment(Qt::AlignCenter);
  panel_layout->addWidget(image_label_);

  panel_widget->setLayout(panel_layout);
  layout_->addWidget(panel_widget);

  setLayout(layout_);

  listen_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    "/dialog_action", 10,
    std::bind(&RobocupRobotState::listenCallback, this, std::placeholders::_1));

  spin_thread_ = std::thread(
    [this]() {
      rclcpp::spin(node_);
    });
}

RobocupRobotState::~RobocupRobotState()
{
  spin_thread_.join();
}

void
RobocupRobotState::onInitialize()
{
}

void RobocupRobotState::listenCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  if (msg->data == 0) {
    image_label_->setPixmap(QPixmap(listen_image_path_.c_str()));
  } else if (msg->data == 1) {
    image_label_->setPixmap(QPixmap(speak_image_path_.c_str()));
  } else if (msg->data == 2) {
    image_label_->setPixmap(QPixmap(think_image_path_.c_str()));
  } else {
    image_label_->clear();
  }
}

}  // namespace robocup_visualization

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(robocup_visualization::RobocupRobotState, rviz_common::Panel)

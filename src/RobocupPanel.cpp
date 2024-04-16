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

  tab_widget_ = new QTabWidget();
  layout_ = new QVBoxLayout();

  QWidget * panel_widget = new QWidget();
  QVBoxLayout * panel_layout = new QVBoxLayout(panel_widget);

  message_history_ = new QTextEdit();
  message_history_->setReadOnly(true);
  message_history_->setStyleSheet("font-size: 25pt;");
  message_history_->setWordWrapMode(QTextOption::WordWrap);
  scrollBar_ = message_history_->verticalScrollBar();
  panel_layout->addWidget(message_history_);

  panel_widget->setLayout(panel_layout);
  layout_->addWidget(panel_widget);

  setLayout(layout_);

  listen_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/whisper/text", 10, std::bind(&RobocupPanel::listenCallback, this, std::placeholders::_1));

  say_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/say_text", 10, std::bind(&RobocupPanel::sayCallback, this, std::placeholders::_1));

  timer_ = new QTimer(this);
  timer_->setInterval(100);

  connect(timer_, &QTimer::timeout, this, &RobocupPanel::scrollToBottom);

  timer_->start();

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

void RobocupPanel::listenCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string message = "Person: " + msg->data;
  message_history_->append("<font color=\"red\">" + QString::fromStdString(message));
}

void RobocupPanel::sayCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string message = "Robot: " + msg->data;
  message_history_->append("<font color=\"blue\">" + QString::fromStdString(message));

}

void RobocupPanel::scrollToBottom()
{
  scrollBar_->setValue(scrollBar_->maximum());
}

}  // namespace robocup_visualization

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(robocup_visualization::RobocupPanel, rviz_common::Panel)

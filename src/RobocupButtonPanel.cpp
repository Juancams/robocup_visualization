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

#include "robocup_visualization/RobocupButtonPanel.hpp"

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

RobocupButtonPanel::RobocupButtonPanel(QWidget * parent)
: Panel(parent)
{
  node_ = rclcpp::Node::make_shared("robocup_panel");
  pub_ = node_->create_publisher<std_msgs::msg::Empty>("/continue", 10);

  tab_widget_ = new QTabWidget();
  layout_ = new QVBoxLayout();

  QWidget * panel_widget = new QWidget();
  QVBoxLayout * panel_layout = new QVBoxLayout(panel_widget);
  
  QFont button_font;
  button_font.setPointSize(48);

  button_ = new QPushButton("CONTINUE", panel_widget);
  button_->setFont(button_font);
  button_->setMinimumSize(200, 200);
  button_->setStyleSheet("color: red;");

  connect(button_, &QPushButton::clicked, this, [this]() {
    std_msgs::msg::Empty msg;
    pub_->publish(msg);
  });

  panel_layout->addWidget(button_);

  panel_widget->setLayout(panel_layout);
  layout_->addWidget(panel_widget);

  setLayout(layout_);

  spin_thread_ = std::thread(
    [this]() {
      rclcpp::spin(node_);
    });
}

RobocupButtonPanel::~RobocupButtonPanel()
{
  spin_thread_.join();
}

void
RobocupButtonPanel::onInitialize()
{
}

}  // namespace robocup_visualization

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(robocup_visualization::RobocupButtonPanel, rviz_common::Panel)

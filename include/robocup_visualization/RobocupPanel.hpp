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

#ifndef ROBOCUP_VISUALIZATION__ROBOCUPPANEL_HPP_
#define ROBOCUP_VISUALIZATION__ROBOCUPPANEL_HPP_

#include <QtWidgets>
#include <QLabel>
#include <QLineEdit>
#include <QBasicTimer>
#include <QTreeWidget>
#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QThread>
#include <QApplication>
#include <QMessageBox>
#include <QTabWidget>
#include <QScrollBar>

#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"


class QPushButton;

namespace robocup_visualization
{

class RobocupPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RobocupPanel(QWidget * parent = 0);
  virtual ~RobocupPanel();

  void onInitialize() override;

protected:
  QVBoxLayout * layout_;
  QTabWidget * tab_widget_;
  QTextEdit * message_history_;
  QScrollBar * scrollBar_;
  QTimer * timer_;

private:
  void listenCallback(const std_msgs::msg::String::SharedPtr msg);
  void sayCallback(const std_msgs::msg::String::SharedPtr msg);
  void scrollToBottom();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listen_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr say_sub_;

  std::thread spin_thread_;
};

}  // namespace robocup_visualization

#endif  //  ROBOCUP_VISUALIZATION__ROBOCUPPANEL_HPP_

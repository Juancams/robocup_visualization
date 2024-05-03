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

#ifndef ROBOCUP_VISUALIZATION__ROBOCUPOBJECTDETECTED_HPP_
#define ROBOCUP_VISUALIZATION__ROBOCUPOBJECTDETECTED_HPP_

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

#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/int8.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  


class QPushButton;

namespace robocup_visualization
{

class RobocupObjectDetected : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RobocupObjectDetected(QWidget * parent = 0);
  virtual ~RobocupObjectDetected();

  void onInitialize() override;

protected:
  QVBoxLayout * layout_;
  QTabWidget * tab_widget_;
  QLabel * image_label_;
  
private:
  void detectionCallback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detections_sub_;

  std::thread spin_thread_;
};

}  // namespace robocup_visualization

#endif  //  ROBOCUP_VISUALIZATION__ROBOCUPOBJECTDETECTED_HPP_

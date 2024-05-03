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

#include "robocup_visualization/RobocupObjectDetected.hpp"

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

RobocupObjectDetected::RobocupObjectDetected(QWidget * parent)
: Panel(parent)
{
  node_ = rclcpp::Node::make_shared("robocup_object_detected");

  detections_sub_ = node_->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/perception_system/detections", 10,
    std::bind(&RobocupObjectDetected::detectionCallback, this, std::placeholders::_1));

  image_label_ = new QLabel();
  image_label_->setAlignment(Qt::AlignCenter);
  layout_ = new QVBoxLayout();
  layout_->addWidget(image_label_);
  setLayout(layout_);

  // Iniciar el hilo para rclcpp::spin
  spin_thread_ = std::thread([this]() {
    rclcpp::spin(node_);
  });
}

RobocupObjectDetected::~RobocupObjectDetected()
{
  spin_thread_.join();
}

void
RobocupObjectDetected::onInitialize()
{
}

void
RobocupObjectDetected::detectionCallback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
{
  const auto& source_img = msg->source_img;

  if (source_img.encoding.empty()) {
    std::cerr << "Error: Image encoding is empty" << std::endl;
    return;
  }

  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(
      source_img, source_img.encoding);

    QImage qimage(
      cv_image->image.data, 
      cv_image->image.cols,
      cv_image->image.rows,
      static_cast<int>(cv_image->image.step),
      QImage::Format_BGR888);

    image_label_->setPixmap(QPixmap::fromImage(qimage));

  } catch (const cv_bridge::Exception& e) {
    std::cerr << "Error converting image: " << e.what() << std::endl;
  }
}

}  // namespace robocup_visualization

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(robocup_visualization::RobocupObjectDetected, rviz_common::Panel)

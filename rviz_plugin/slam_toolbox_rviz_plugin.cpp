/*
 * slam_toolbox
 * Copyright (c) 2018, Simbe Robotics, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

// Header
#include "rviz_plugin/slam_toolbox_rviz_plugin.hpp"
// ROS
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
// QT
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>
// STL
#include <memory>


namespace slam_toolbox
{

/*****************************************************************************/
SlamToolboxPlugin::SlamToolboxPlugin(QWidget * parent)
: Panel(parent),
  _match_type(PROCESS_FIRST_NODE_CMT)
/*****************************************************************************/
{
  ros_node_ = std::make_shared<rclcpp::Node>("SlamToolboxPlugin");

  bool paused_measure = false, interactive = false;
  paused_measure = ros_node_->declare_parameter(
    "/slam_toolbox/paused_new_measurements", paused_measure);
  interactive = ros_node_->declare_parameter(
    "/slam_toolbox/interactive_mode", interactive);
    
  _initialposeSub =
    ros_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&SlamToolboxPlugin::InitialPoseCallback, this, std::placeholders::_1));

  _serialize =
    ros_node_->create_client<slam_toolbox::srv::SerializePoseGraph>(
    "/slam_toolbox/serialize_map");
  _load_map =
    ros_node_->create_client<slam_toolbox::srv::DeserializePoseGraph>(
    "/slam_toolbox/deserialize_map");
  _clearChanges = ros_node_->create_client<slam_toolbox::srv::Clear>(
    "/slam_toolbox/clear_changes");
  _saveChanges = ros_node_->create_client<slam_toolbox::srv::LoopClosure>(
    "/slam_toolbox/manual_loop_closure");
  _saveMap = ros_node_->create_client<slam_toolbox::srv::SaveMap>(
    "/slam_toolbox/save_map");
  _clearQueue = ros_node_->create_client<slam_toolbox::srv::ClearQueue>(
    "/slam_toolbox/clear_queue");
  _interactive =
    ros_node_->create_client<slam_toolbox::srv::ToggleInteractive>(
    "/slam_toolbox/toggle_interactive_mode");
  _pause_measurements = ros_node_->create_client<slam_toolbox::srv::Pause>(
    "/slam_toolbox/pause_new_measurements");
  _load_submap_for_merging =
    ros_node_->create_client<slam_toolbox::srv::AddSubmap>(
    "/slam_toolbox/add_submap");
  _merge = ros_node_->create_client<slam_toolbox::srv::MergeMaps>(
    "/slam_toolbox/merge_submaps");

  _vbox = new QVBoxLayout();
  _hbox1 = new QHBoxLayout();
  _hbox2 = new QHBoxLayout();
  _hbox3 = new QHBoxLayout();
  _hbox4 = new QHBoxLayout();
  _hbox5 = new QHBoxLayout();
  _hbox6 = new QHBoxLayout();
  _hbox7 = new QHBoxLayout();
  _hbox8 = new QHBoxLayout();
  _hbox9 = new QHBoxLayout();
  _hbox10 = new QHBoxLayout();

  QFrame * _line = new QFrame();
  _line->setFrameShape(QFrame::HLine);
  _line->setFrameShadow(QFrame::Sunken);

  _button1 = new QPushButton(this);
  _button1->setText("Clear Changes");
  connect(_button1, SIGNAL(clicked()), this, SLOT(ClearChanges()));
  _button2 = new QPushButton(this);
  _button2->setText("Save Changes");
  connect(_button2, SIGNAL(clicked()), this, SLOT(SaveChanges()));
  _button3 = new QPushButton(this);
  _button3->setText("Save Map");
  connect(_button3, SIGNAL(clicked()), this, SLOT(SaveMap()));
  _button4 = new QPushButton(this);
  _button4->setText("Clear Measurement Queue");
  connect(_button4, SIGNAL(clicked()), this, SLOT(ClearQueue()));
  _button5 = new QPushButton(this);
  _button5->setText("Add Submap");
  connect(_button5, SIGNAL(clicked()), this, SLOT(LoadSubmap()));
  _button6 = new QPushButton(this);
  _button6->setText("Generate Map");
  connect(_button6, SIGNAL(clicked()), this, SLOT(GenerateMap()));
  _button7 = new QPushButton(this);
  _button7->setText("Serialize Map");
  connect(_button7, SIGNAL(clicked()), this, SLOT(SerializeMap()));
  _button8 = new QPushButton(this);
  _button8->setText("Deserialize Map");
  connect(_button8, SIGNAL(clicked()), this, SLOT(DeserializeMap()));

  _label1 = new QLabel(this);
  _label1->setText("Interactive Mode");
  _label2 = new QLabel(this);
  _label2->setText("Accept New Scans");
  _label4 = new QLabel(this);
  _label4->setText("Merge Map Tool");
  _label4->setAlignment(Qt::AlignCenter);
  _label5 = new QLabel(this);
  _label5->setText("Create Map Tool");
  _label5->setAlignment(Qt::AlignCenter);
  _label6 = new QLabel(this);
  _label6->setText("X");
  _label6->setAlignment(Qt::AlignCenter);
  _label7 = new QLabel(this);
  _label7->setText("Y");
  _label7->setAlignment(Qt::AlignCenter);
  _label8 = new QLabel(this);
  _label8->setText("θ");
  _label8->setAlignment(Qt::AlignCenter);

  _check1 = new QCheckBox();
  _check1->setChecked(interactive);
  connect(_check1, SIGNAL(stateChanged(int)), this, SLOT(InteractiveCb(int)));
  _check2 = new QCheckBox();
  _check2->setChecked(!paused_measure);
  connect(_check2, SIGNAL(stateChanged(int)), this,
    SLOT(PauseMeasurementsCb(int)));
  _radio1 = new QRadioButton(tr("Start At Dock"));
  _radio1->setChecked(true);
  _radio2 = new QRadioButton(tr("Start At Pose Est."));
  _radio3 = new QRadioButton(tr("Start At Curr. Odom"));
  _radio4 = new QRadioButton(tr("Localize"));

  connect(_radio1, SIGNAL(clicked()), this, SLOT(FirstNodeMatchCb()));
  connect(_radio2, SIGNAL(clicked()), this, SLOT(PoseEstMatchCb()));
  connect(_radio3, SIGNAL(clicked()), this, SLOT(CurEstMatchCb()));
  connect(_radio4, SIGNAL(clicked()), this, SLOT(LocalizeCb()));

  _line1 = new QLineEdit();
  _line2 = new QLineEdit();
  _line3 = new QLineEdit();
  _line4 = new QLineEdit();
  _line5 = new QLineEdit();
  _line6 = new QLineEdit();
  _line7 = new QLineEdit();

  _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button5->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button6->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button7->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button8->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _check1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _check2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line5->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line6->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line7->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  _hbox1->addWidget(_check1);
  _hbox1->addWidget(_label1);
  _hbox1->addWidget(_check2);
  _hbox1->addWidget(_label2);

  _hbox2->addWidget(_button1);
  _hbox2->addWidget(_button2);

  _hbox3->addWidget(_button3);
  _hbox3->addWidget(_line1);

  _hbox4->addWidget(_button4);

  _hbox5->addWidget(_button5);
  _hbox5->addWidget(_line2);

  _hbox6->addWidget(_button6);

  _hbox7->addWidget(_button7);
  _hbox7->addWidget(_line3);

  _hbox8->addWidget(_button8);
  _hbox8->addWidget(_line4);

  _hbox9->addWidget(_radio1);
  _hbox9->addWidget(_radio2);
  _hbox9->addWidget(_radio3);
  _hbox9->addWidget(_radio4);
  _hbox9->addStretch(1);

  _hbox10->addWidget(_label6);
  _hbox10->addWidget(_line5);
  _hbox10->addWidget(_label7);
  _hbox10->addWidget(_line6);
  _hbox10->addWidget(_label8);
  _hbox10->addWidget(_line7);

  _vbox->addWidget(_label5);
  _vbox->addLayout(_hbox1);
  _vbox->addLayout(_hbox2);
  _vbox->addLayout(_hbox3);
  _vbox->addLayout(_hbox7);
  _vbox->addLayout(_hbox8);
  _vbox->addLayout(_hbox9);
  _vbox->addLayout(_hbox10);
  _vbox->addLayout(_hbox4);
  _vbox->addWidget(_line);
  _vbox->addWidget(_label4);
  _vbox->addLayout(_hbox5);
  _vbox->addLayout(_hbox6);

  setLayout(_vbox);

  _thread =
    std::make_unique<std::thread>(
    &SlamToolboxPlugin::updateCheckStateIfExternalChange, this);
}

/*****************************************************************************/
SlamToolboxPlugin::~SlamToolboxPlugin()
/*****************************************************************************/
{
  _thread->join();
  _thread.reset();
}
  
/*****************************************************************************/
void SlamToolboxPlugin::InitialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
/*****************************************************************************/
{
  _match_type = PROCESS_NEAR_REGION_CMT;
  RCLCPP_INFO(
    ros_node_->get_logger(),
    "Setting initial pose from rviz; you can now deserialize a map given that pose.");
  _radio2->setChecked(true);
  _line5->setText(QString::number(msg->pose.pose.position.x, 'f', 2));
  _line6->setText(QString::number(msg->pose.pose.position.y, 'f', 2));
  tf2::Quaternion quat_tf;
  tf2::convert(msg->pose.pose.orientation , quat_tf);
  tf2::Matrix3x3 m(quat_tf);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  _line7->setText(QString::number(yaw, 'f', 2));
}

/*****************************************************************************/
void SlamToolboxPlugin::SerializeMap()
/*****************************************************************************/
{
  auto request =
    std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
  request->filename = _line3->text().toStdString();
  auto result_future = _serialize->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(ros_node_->get_logger(),
      "SlamToolbox: Failed to serialize"
      " pose graph to file, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::DeserializeMap()
/*****************************************************************************/
{
  typedef slam_toolbox::srv::DeserializePoseGraph::Request procType;

  auto request =
    std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
  request->filename = _line4->text().toStdString();
  if (_match_type == PROCESS_FIRST_NODE_CMT) {
    request->match_type = procType::START_AT_FIRST_NODE;
  } else if (_match_type == PROCESS_NEAR_REGION_CMT) {
    try
    {
      request->match_type = procType::START_AT_GIVEN_POSE;
      request->initial_pose.x = std::stod(_line5->text().toStdString());
      request->initial_pose.y = std::stod(_line6->text().toStdString());
      request->initial_pose.theta = std::stod(_line7->text().toStdString());
    }
    catch (const std::invalid_argument& ia)
    {
      RCLCPP_WARN(ros_node_->get_logger(), "Initial pose invalid.");
      return;
    }
  } else if (_match_type == LOCALIZE_CMT) {
    try
    {
      request->match_type = procType::LOCALIZE_AT_POSE;
      request->initial_pose.x = std::stod(_line5->text().toStdString());
      request->initial_pose.y = std::stod(_line6->text().toStdString());
      request->initial_pose.theta = std::stod(_line7->text().toStdString());
    }
    catch (const std::invalid_argument& ia)
    {
      RCLCPP_WARN(ros_node_->get_logger(), "Initial pose invalid.");
      return;
    }
  } else {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "No match type selected, cannot send request.");
    return;
  }

  auto result_future = _load_map->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "SlamToolbox: Failed to deserialize mapper object "
      "from file, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::LoadSubmap()
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::AddSubmap::Request>();
  request->filename = _line2->text().toStdString();
  auto result_future = _load_submap_for_merging->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "MergeMaps: Failed to load pose graph from file, is service running?");
  }
}
/*****************************************************************************/
void SlamToolboxPlugin::GenerateMap()
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::MergeMaps::Request>();
  auto result_future = _merge->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "MergeMaps: Failed to merge maps, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::ClearChanges()
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::Clear::Request>();
  auto result_future = _clearChanges->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "SlamToolbox: Failed to clear changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveChanges()
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::LoopClosure::Request>();
  auto result_future = _saveChanges->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "SlamToolbox: Failed to save changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveMap()
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
  request->name.data = _line1->text().toStdString();
  auto result_future = _saveMap->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "SlamToolbox: Failed to save map as %s, is service running?",
      request->name.data.c_str());
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::ClearQueue()
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::ClearQueue::Request>();
  auto result_future = _clearQueue->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "Failed to clear queue, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::InteractiveCb(int state)
/*****************************************************************************/
{
  auto request =
    std::make_shared<slam_toolbox::srv::ToggleInteractive::Request>();
  auto result_future = _interactive->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "SlamToolbox: Failed to toggle interactive mode, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PauseMeasurementsCb(int state)
/*****************************************************************************/
{
  auto request = std::make_shared<slam_toolbox::srv::Pause::Request>();
  auto result_future = _pause_measurements->async_send_request(request);

  if (rclcpp::spin_until_future_complete(ros_node_, result_future,
    std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "SlamToolbox: Failed to toggle pause measurements, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::FirstNodeMatchCb()
/*****************************************************************************/
{
  if (_radio1->isChecked() == Qt::Unchecked) {
    return;
  } else {
    _match_type = PROCESS_FIRST_NODE_CMT;
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Processing at first node selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PoseEstMatchCb()
/*****************************************************************************/
{
  if (_radio2->isChecked() == Qt::Unchecked) {
    return;
  } else {
    _match_type = PROCESS_NEAR_REGION_CMT;
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Processing at current pose estimate selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::CurEstMatchCb()
/*****************************************************************************/
{
  if (_radio3->isChecked() == Qt::Unchecked) {
    return;
  } else {
    _match_type = PROCESS_CMT;
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Processing at current odometry selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::LocalizeCb()
/*****************************************************************************/
{
  if (_radio4->isChecked() == Qt::Unchecked) {
    return;
  } else {
    _match_type = LOCALIZE_CMT;
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Processing localization selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::updateCheckStateIfExternalChange()
/*****************************************************************************/
{
  rclcpp::Rate r(1);
  bool paused_measure = false, interactive = false;
  auto node = std::make_shared<rclcpp::Node>("SlamToolboxStateUpdateNode");
  auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(node, "slam_toolbox");

  while (rclcpp::ok()) {
    auto parameters = parameters_client->get_parameters(
      {"paused_new_measurements", "interactive_mode"});
    paused_measure = parameters[0].as_bool();
    interactive = parameters[1].as_bool();

    bool oldState = _check1->blockSignals(true);
    _check1->setChecked(interactive);
    _check1->blockSignals(oldState);

    oldState = _check2->blockSignals(true);
    _check2->setChecked(!paused_measure);
    _check2->blockSignals(oldState);

    r.sleep();
  }
}

}  // namespace slam_toolbox

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(slam_toolbox::SlamToolboxPlugin, rviz_common::Panel)

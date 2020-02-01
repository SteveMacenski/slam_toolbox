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
#include "slam_toolbox_rviz_plugin.h"
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slam_toolbox::SlamToolboxPlugin, rviz::Panel)

namespace slam_toolbox
{

/*****************************************************************************/
SlamToolboxPlugin::SlamToolboxPlugin(QWidget* parent):
    rviz::Panel(parent),
    _thread(NULL),
    _match_type(PROCESS_FIRST_NODE_CMT)
/*****************************************************************************/    
{
  ros::NodeHandle nh;
  bool paused_measure = false, interactive = false;
  nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
  nh.getParam("/slam_toolbox/interactive_mode", interactive);
  _serialize = nh.serviceClient<slam_toolbox::SerializePoseGraph>("/slam_toolbox/serialize_map");
  _load_map = nh.serviceClient<slam_toolbox::DeserializePoseGraph>("/slam_toolbox/deserialize_map");
  _clearChanges = nh.serviceClient<slam_toolbox::Clear>("/slam_toolbox/clear_changes");
  _saveChanges = nh.serviceClient<slam_toolbox::LoopClosure>("/slam_toolbox/manual_loop_closure");
  _saveMap = nh.serviceClient<slam_toolbox::SaveMap>("/slam_toolbox/save_map");
  _clearQueue = nh.serviceClient<slam_toolbox::ClearQueue>("/slam_toolbox/clear_queue");
  _interactive = nh.serviceClient<slam_toolbox::ToggleInteractive>("/slam_toolbox/toggle_interactive_mode");
  _pause_measurements = nh.serviceClient<slam_toolbox::Pause>("/slam_toolbox/pause_new_measurements");
  _load_submap_for_merging = nh.serviceClient<slam_toolbox::AddSubmap>("/map_merging/add_submap");
  _merge = nh.serviceClient<slam_toolbox::MergeMaps>("/map_merging/merge_submaps");

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

  QFrame* _line = new QFrame();
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
  connect(_check2, SIGNAL(stateChanged(int)), this, SLOT(PauseMeasurementsCb(int)));
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

  _thread = new std::thread(&SlamToolboxPlugin::updateCheckStateIfExternalChange, this);
}

/*****************************************************************************/
SlamToolboxPlugin::~SlamToolboxPlugin()
/*****************************************************************************/
{
  if (_thread)
  {
    delete _thread;
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SerializeMap()
/*****************************************************************************/
{
  slam_toolbox::SerializePoseGraph msg;
  msg.request.filename = _line3->text().toStdString();
  if (!_serialize.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to serialize pose graph to file, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::DeserializeMap()
/*****************************************************************************/
{
  typedef slam_toolbox::DeserializePoseGraph::Request procType;

  slam_toolbox::DeserializePoseGraph msg;
  msg.request.filename = _line4->text().toStdString();
  if (_match_type == PROCESS_FIRST_NODE_CMT)
  {
    msg.request.match_type = procType::START_AT_FIRST_NODE;
  }
  else if (_match_type == PROCESS_NEAR_REGION_CMT)
  {
    msg.request.match_type = procType::START_AT_GIVEN_POSE;
    msg.request.initial_pose.x = std::stod(_line5->text().toStdString());
    msg.request.initial_pose.y = std::stod(_line6->text().toStdString());
    msg.request.initial_pose.theta = std::stod(_line7->text().toStdString());
  }
  else if (_match_type == LOCALIZE_CMT)
  {
    msg.request.match_type = procType::LOCALIZE_AT_POSE;
    msg.request.initial_pose.x = std::stod(_line5->text().toStdString());
    msg.request.initial_pose.y = std::stod(_line6->text().toStdString());
    msg.request.initial_pose.theta = std::stod(_line7->text().toStdString());
  }
  else
  {
    ROS_WARN("No match type selected, cannot send request.");
    return;
  }
  if (!_load_map.call(msg))
  {
     ROS_WARN("SlamToolbox: Failed to deserialize mapper object "
      "from file, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::LoadSubmap()
/*****************************************************************************/
{
  slam_toolbox::AddSubmap msg;
  msg.request.filename = _line2->text().toStdString();
  if (!_load_submap_for_merging.call(msg))
  {
    ROS_WARN("MergeMaps: Failed to load pose graph from file, is service running?");
  }
}
/*****************************************************************************/
void SlamToolboxPlugin::GenerateMap()
/*****************************************************************************/
{
  slam_toolbox::MergeMaps msg;
  if (!_merge.call(msg))
  {
    ROS_WARN("MergeMaps: Failed to merge maps, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::ClearChanges()
/*****************************************************************************/
{
  slam_toolbox::Clear msg;
  if (!_clearChanges.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to clear changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveChanges()
/*****************************************************************************/
{
  slam_toolbox::LoopClosure msg;

  if (!_saveChanges.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to save changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveMap()
/*****************************************************************************/
{
  slam_toolbox::SaveMap msg;
  msg.request.name.data = _line1->text().toStdString();
  if (!_saveMap.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to save map as %s, is service running?",
              msg.request.name.data.c_str());
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::ClearQueue()
/*****************************************************************************/
{
  slam_toolbox::ClearQueue msg;
  if (!_clearQueue.call(msg))
  {
    ROS_WARN("Failed to clear queue, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::InteractiveCb(int state)
/*****************************************************************************/
{
  slam_toolbox::ToggleInteractive msg;
  if (!_interactive.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to toggle interactive mode, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PauseMeasurementsCb(int state)
/*****************************************************************************/
{
  slam_toolbox::Pause msg;
  if (!_pause_measurements.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to toggle pause measurements, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::FirstNodeMatchCb()
/*****************************************************************************/
{
  if (_radio1->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = PROCESS_FIRST_NODE_CMT;
    ROS_INFO("Processing at first node selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PoseEstMatchCb()
/*****************************************************************************/
{
  if (_radio2->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = PROCESS_NEAR_REGION_CMT;
    ROS_INFO("Processing at current pose estimate selected.");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::CurEstMatchCb()
/*****************************************************************************/
{
  if (_radio3->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = PROCESS_CMT;
    ROS_INFO("Processing at current odometry selected.");
  }
}


/*****************************************************************************/
void SlamToolboxPlugin::LocalizeCb()
/*****************************************************************************/
{
  if (_radio4->isChecked() == Qt::Unchecked)
  {
    return;
  }
  else
  {
    _match_type = LOCALIZE_CMT;
    ROS_INFO("Processing localization selected.");
  }
}


/*****************************************************************************/
void SlamToolboxPlugin::updateCheckStateIfExternalChange()
/*****************************************************************************/
{
  ros::Rate r(1); //1 hz
  ros::NodeHandle nh;
  bool paused_measure = false, interactive = false;
  while (ros::ok())
  {
    nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
    nh.getParam("/slam_toolbox/interactive_mode", interactive);

    bool oldState = _check1->blockSignals(true);
    _check1->setChecked(interactive);
    _check1->blockSignals(oldState);

    oldState = _check2->blockSignals(true);
    _check2->setChecked(!paused_measure);
    _check2->blockSignals(oldState);

    r.sleep();
  }
}

} // end namespace

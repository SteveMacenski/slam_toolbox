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
#include "slam_toolbox_rviz_plugin.hpp"
// QT
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
// ROS
#include <pluginlib/class_list_macros.h>

namespace slam_toolbox
{

/*****************************************************************************/
SlamToolboxPlugin::SlamToolboxPlugin(QWidget* parent):
    rviz::Panel(parent),
    _thread(NULL)
/*****************************************************************************/    
{
  ros::NodeHandle nh;
  bool paused_measure = false, paused_process = false, interactive = false;
  nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
  nh.getParam("/slam_toolbox/paused_processing", paused_process);
  nh.getParam("/slam_toolbox/interactive_mode", interactive);

  _clearChanges = nh.serviceClient<slam_toolbox::Clear>("/slam_toolbox/clear_changes");
  _saveChanges = nh.serviceClient<slam_toolbox::LoopClosure>("/slam_toolbox/manual_loop_closure");
  _saveMap = nh.serviceClient<slam_toolbox::SaveMap>("/slam_toolbox/save_map");
  _clearQueue = nh.serviceClient<slam_toolbox::ClearQueue>("/slam_toolbox/clear_queue");
  _interactive = nh.serviceClient<slam_toolbox::ToggleInteractive>("/slam_toolbox/toggle_interactive_mode");
  _pause_processing = nh.serviceClient<slam_toolbox::Pause>("/slam_toolbox/pause_processing");
  _pause_measurements = nh.serviceClient<slam_toolbox::Pause>("/slam_toolbox/pause_new_measurements");

  _vbox = new QVBoxLayout();
  _hbox1 = new QHBoxLayout();
  _hbox2 = new QHBoxLayout();
  _hbox3 = new QHBoxLayout();
  _hbox4 = new QHBoxLayout();

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

  _label1 = new QLabel(this);
  _label1->setText("Interactive");
  _label2 = new QLabel(this);
  _label2->setText("Allow New Scans");
  _label3 = new QLabel(this);
  _label3->setText("Process Scans");

  _check1 = new QCheckBox();
  _check1->setChecked(interactive);
  connect(_check1, SIGNAL(stateChanged(int)), this, SLOT(InteractiveCb(int)));
  _check2 = new QCheckBox();
  _check2->setChecked(!paused_measure);
  connect(_check2, SIGNAL(stateChanged(int)), this, SLOT(PauseMeasurementsCb(int)));
  _check3 = new QCheckBox();
  _check3->setChecked(!paused_process);
  connect(_check3, SIGNAL(stateChanged(int)), this, SLOT(PauseProcessingCb(int)));

  _line1 = new QLineEdit();

  _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _check1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _check2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _check3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  _hbox1->addWidget(_check1);
  _hbox1->addWidget(_label1);
  _hbox1->addWidget(_check2);
  _hbox1->addWidget(_label2);
  _hbox1->addWidget(_check3);
  _hbox1->addWidget(_label3);

  _hbox2->addWidget(_button1);
  _hbox2->addWidget(_button2);

  _hbox3->addWidget(_button3);
  _hbox3->addWidget(_line1);

  _hbox4->addWidget(_button4);

  _vbox->addLayout(_hbox1);
  _vbox->addLayout(_hbox2);
  _vbox->addLayout(_hbox3);
  _vbox->addLayout(_hbox4);

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
void SlamToolboxPlugin::ClearChanges()
/*****************************************************************************/
{
  slam_toolbox::Clear msg;
  if (!_clearChanges.call(msg))
  {
    ROS_WARN("Failed to clear changes, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::SaveChanges()
/*****************************************************************************/
{
  slam_toolbox::LoopClosure msg;

  if (!_saveChanges.call(msg))
  {
    ROS_WARN("Failed to save changes, is service running?");
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
    ROS_WARN("Failed to save map as %s, is service running?", 
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
    ROS_WARN("Failed to toggle interactive mode, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PauseProcessingCb(int state)
/*****************************************************************************/
{
  slam_toolbox::Pause msg;
  if (!_pause_processing.call(msg))
  {
    ROS_WARN("Failed to toggle pause processing, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::PauseMeasurementsCb(int state)
/*****************************************************************************/
{
  slam_toolbox::Pause msg;
  if (!_pause_measurements.call(msg))
  {
    ROS_WARN("Failed to toggle pause measurements, is service running?");
  }
}

/*****************************************************************************/
void SlamToolboxPlugin::updateCheckStateIfExternalChange()
/*****************************************************************************/
{
  ros::Rate r(1); //1 hz
  ros::NodeHandle nh;
  bool paused_measure = false, paused_process = false, interactive = false;
  while (ros::ok())
  {
    nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
    nh.getParam("/slam_toolbox/paused_processing", paused_process);
    nh.getParam("/slam_toolbox/interactive_mode", interactive);

    bool oldState = _check1->blockSignals(true);
    _check1->setChecked(interactive);
    _check1->blockSignals(oldState);

    oldState = _check2->blockSignals(true);
    _check2->setChecked(!paused_measure);
    _check2->blockSignals(oldState);

    oldState = _check3->blockSignals(true);
    _check3->setChecked(!paused_process);
    _check3->blockSignals(oldState);

    r.sleep();
  }
}

} // end namespace

PLUGINLIB_EXPORT_CLASS(slam_toolbox::SlamToolboxPlugin, rviz::Panel)

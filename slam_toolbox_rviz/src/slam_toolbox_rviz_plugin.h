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

#ifndef SLAM_TOOLBOX_PANEL_H
#define SLAM_TOOLBOX_PANEL_H

// ROS
#include <ros/ros.h>
#include <rviz/panel.h>
// STL
#include <stdlib.h>
#include <stdio.h>
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
#include <QRadioButton>

#include <thread>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "slam_toolbox_msgs/AddSubmap.h"
#include "slam_toolbox_msgs/Clear.h"
#include "slam_toolbox_msgs/ClearQueue.h"
#include "slam_toolbox_msgs/DeserializePoseGraph.h"
#include "slam_toolbox_msgs/LoopClosure.h"
#include "slam_toolbox_msgs/MergeMaps.h"
#include "slam_toolbox_msgs/Pause.h"
#include "slam_toolbox_msgs/SaveMap.h"
#include "slam_toolbox_msgs/SerializePoseGraph.h"
#include "slam_toolbox_msgs/ToggleInteractive.h"

class QLineEdit;
class QSpinBox;
class QComboBox;

#include <rviz/panel.h>

namespace slam_toolbox
{

enum ContinueMappingType
{
  PROCESS_CMT = 0,
  PROCESS_FIRST_NODE_CMT = 1,
  PROCESS_NEAR_REGION_CMT = 2,
  LOCALIZE_CMT = 3
};

class SlamToolboxPlugin : public rviz::Panel
{
  Q_OBJECT

public:
  SlamToolboxPlugin(QWidget* parent = 0);
  ~SlamToolboxPlugin();

public Q_SLOTS:
protected Q_SLOTS:
  void ClearChanges();
  void SaveChanges();
  void SaveMap();
  void ClearQueue();
  void InteractiveCb(int state);
  void PauseMeasurementsCb(int state);
  void FirstNodeMatchCb();
  void PoseEstMatchCb();
  void CurEstMatchCb();
  void LocalizeCb();
  void LoadSubmap();
  void GenerateMap();
  void SerializeMap();
  void DeserializeMap();

  void updateCheckStateIfExternalChange();

protected:
  QVBoxLayout* _vbox;
  QHBoxLayout* _hbox1;
  QHBoxLayout* _hbox2;
  QHBoxLayout* _hbox3;
  QHBoxLayout* _hbox4;
  QHBoxLayout* _hbox5;
  QHBoxLayout* _hbox6;
  QHBoxLayout* _hbox7;
  QHBoxLayout* _hbox8;
  QHBoxLayout* _hbox9;
  QHBoxLayout* _hbox10;

  QPushButton* _button1;
  QPushButton* _button2;
  QPushButton* _button3;
  QPushButton* _button4;
  QPushButton* _button5;
  QPushButton* _button6;
  QPushButton* _button7;
  QPushButton* _button8;

  QLineEdit* _line1;
  QLineEdit* _line2;
  QLineEdit* _line3;
  QLineEdit* _line4;
  QLineEdit* _line5;
  QLineEdit* _line6;
  QLineEdit* _line7;

  QCheckBox* _check1;
  QCheckBox* _check2;

  QRadioButton* _radio1;
  QRadioButton* _radio2;
  QRadioButton* _radio3;
  QRadioButton* _radio4;

  QLabel* _label1;
  QLabel* _label2;
  QLabel* _label4;
  QLabel* _label5;
  QLabel* _label6;
  QLabel* _label7;
  QLabel* _label8;

  QFrame* _line;

  ros::ServiceClient _clearChanges, _saveChanges, _saveMap, _clearQueue, _interactive, _pause_measurements, _load_submap_for_merging, _merge, _serialize, _load_map;
  ros::Subscriber _initialposeSub;

  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

  std::thread* _thread;

  ContinueMappingType _match_type;
};

} // end namespace

#endif

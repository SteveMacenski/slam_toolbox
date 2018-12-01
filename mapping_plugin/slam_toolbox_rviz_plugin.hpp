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

#include <thread>

// msgs
#include "slam_toolbox/Pause.h"
#include "slam_toolbox/ClearQueue.h"
#include "slam_toolbox/ToggleInteractive.h"
#include "slam_toolbox/Clear.h"
#include "slam_toolbox/SaveMap.h"
#include "slam_toolbox/LoopClosure.h"
#include "slam_toolbox/MergeMaps.h"
#include "slam_toolbox/AddSubmap.h"
#include "slam_toolbox/SerializePoseGraph.h"

class QLineEdit;
class QSpinBox;
class QComboBox;

#include <rviz/panel.h>

namespace slam_toolbox
{

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
  void PauseProcessingCb(int state);
  void PauseMeasurementsCb(int state);
  void LoadPoseGraph();
  void GenerateMap();
  void SerializeMap();
  void LoadMap();

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

  QCheckBox* _check1;
  QCheckBox* _check2;
  QCheckBox* _check3;

  QLabel* _label1;
  QLabel* _label2;
  QLabel* _label3;
  QLabel* _label4;
  QLabel* _label5;

  QFrame* _line;

  ros::ServiceClient _clearChanges, _saveChanges, _saveMap, _clearQueue, _interactive, _pause_processing, _pause_measurements, _load_submap, _merge, _serialize, _load_map;

  std::thread* _thread;
};

} // end namespace

#endif
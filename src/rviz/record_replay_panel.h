/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_RECORD_REPLAY_PANEL_H
#define RVIZ_RECORD_REPLAY_PANEL_H

#include <rviz/panel.h>
#include <ros/ros.h>

#include <qstring.h>

class QLineEdit;
class QComboBox;
class QCheckBox;
class QPushButton;
class QHBoxLayout;
class QWidget;
class QTimer;
class QStringListModel;
class QLabel;
class QGroupBox;
class QRadioButton;
class QCheckList;
class QSlider;

namespace rviz
{
class VisualizationManager;
class Display;

enum RecordReplayControlMode
{
  RECORD_MODE = 0,
  REPLAY_MODE = 1
};

enum RecordingControlRequest
{
  START_RECORDING = 0,
  STOP_RECORDING = 1
};

enum ReplayControlRequest
{
  START_REPLAY = 0,
  PAUSE_REPLAY = 1,
  STOP_REPLAY = 2,
};

/**
 * \class RecordReplayPanel
 *
 */
class RecordReplayPanel : public Panel
{
  Q_OBJECT
public:
  RecordReplayPanel(QWidget* parent = nullptr);

  void onInitialize() override;

protected Q_SLOTS:
  void recordModeToggled(bool checked);
  void replayModeToggled(bool checked);

  void startToggled(bool checked);
  void pauseToggled(bool checked);
  void stopToggled(bool checked);
  void recordToggled(bool checked);
  void recordReplayNodeIndexChanged(int index);
  void onRecordReplayNodesUpdateTimer();
  void useDefaultTopicsToggled(int state);

  /** Read time values from VisualizationManager and update displays. */
  void update();

  void onDisplayAdded(rviz::Display* display);
  void onDisplayRemoved(rviz::Display* display);

  void onTimeSignal(rviz::Display* display, ros::Time time);

  void load(const Config& config) override;
  void save(Config config) const override;

protected:
  bool dispatchRecordingControl(const RecordingControlRequest, QString &status_message);
  bool dispatchReplayControl(const ReplayControlRequest, QString& status_message);

  bool getRecorderOrPlayerNodeServices(bool player_services);
  bool updateTopicList();
  void updateRecorderPlayerNodesList(bool force_update = false);
  void updateRosbagsForReplayList();

  QLabel* control_label_;
  QPushButton* start_button_;
  QPushButton* pause_button_;
  QPushButton* stop_button_;
  QPushButton* record_button_;
  QLabel* recorder_player_nodes_label_;
  QComboBox* recorder_nodes_combobox_;
  QStringListModel* recorder_nodes_combobox_model_;
  QGroupBox* control_mode_group_box_;
  QGroupBox* replay_control_group_box_;
  QRadioButton* replay_mode_button_;
  QRadioButton* record_mode_button_;
  QLabel* use_default_topics_label_;
  QCheckBox* use_default_topics_checkbox_;
  QLabel* topic_list_label_;
  QCheckList* topic_selection_list_;

  QSlider* replay_msg_selection_slider_;
  QComboBox* replay_rosbag_folders_combobox_;
  QCheckList* replay_rosbag_files_list_;

  QTimer* recorder_player_nodes_update_timer_;

  ros::NodeHandlePtr nh_;

  std::vector<std::string> all_node_names_;
  std::vector<std::string> recorder_player_node_names_;

  std::vector<std::string> topic_list_;

  std::string bag_file_base_name_;
  int bag_file_compression_type_;
  int bag_file_naming_mode_;
  std::string bag_file_size_limit_;

  std::string current_recorder_node_name_, current_player_node_name_;
  std::string current_recorder_node_control_srv_uri_;

  RecordReplayControlMode control_mode_;

  unsigned long replay_slider_message_count_;

  unsigned int node_list_update_interval_;
};

} // namespace rviz

#endif


/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QButtonGroup>
#include <QCheckBox>
#include <QSlider>
#include <QComboBox>
#include <QDebug>
#include <QTimer>
#include <QStringListModel>
#include <QGroupBox>
#include <QRadioButton>

#include "q_checkable_combobox.h"

#include <boost/algorithm/string.hpp>

#include "visualization_manager.h"
#include "frame_manager.h"
#include <rviz/load_resource.h>

#include "display_group.h"

#include "record_replay_panel.h"

#include <carecules_rosbag_msgs/CCRosbagRecord.h>
#include <carecules_rosbag_msgs/CCRosbagPlay.h>

namespace rviz
{
  RecordReplayPanel::RecordReplayPanel(QWidget* parent) : Panel(parent)
  {
    control_mode_ = rviz::RECORD_MODE;
    // Placeholder value for replay control slider
    // TODO: Adapt from rosbag replay setup response of selected player node
    replay_slider_message_count_ = 100000;

    // TODO: Expose via inputs
    bag_file_base_name_ = "ros_bag";
    bag_file_compression_type_ = 0; // rosbag::CompressonType::Uncompressed
    bag_file_naming_mode_ = 1; // BagWriter::Naming::AppendTimestamp
    bag_file_size_limit_ = "100G";

    nh_.reset(new ros::NodeHandle("cc_record_replay_control"));

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Creating control mode widgets.");
    control_mode_group_box_ = new QGroupBox("Control mode");
    replay_mode_button_ = new QRadioButton(tr("&Replay"));
    record_mode_button_ = new QRadioButton(tr("R&ecord"));

    // Activate record mode by default
    record_mode_button_->setChecked(true);
    replay_mode_button_->setChecked(false);

    QVBoxLayout* outer_control_mode_layout = new QVBoxLayout();
    QHBoxLayout* inner_control_mode_layout = new QHBoxLayout();
    inner_control_mode_layout->addWidget(record_mode_button_);
    inner_control_mode_layout->addWidget(replay_mode_button_);

    outer_control_mode_layout->addLayout(inner_control_mode_layout);

    replay_control_group_box_= new QGroupBox("ROS bag replay control");
    QVBoxLayout* slider_layout = new QVBoxLayout();
    replay_msg_selection_slider_ = new QSlider();
    replay_msg_selection_slider_->setMaximum(replay_slider_message_count_);
    replay_msg_selection_slider_->setOrientation(Qt::Orientation::Horizontal);
    slider_layout->addWidget(replay_msg_selection_slider_);

    QHBoxLayout* rosbag_list_layout = new QHBoxLayout();
    replay_rosbag_folders_combobox_ = new QComboBox();
    replay_rosbag_files_list_ = new QCheckList();
    rosbag_list_layout->addWidget(replay_rosbag_folders_combobox_);
    rosbag_list_layout->addWidget(replay_rosbag_files_list_);
    slider_layout->addLayout(rosbag_list_layout);

    control_mode_group_box_->setLayout(outer_control_mode_layout);
    replay_control_group_box_->setLayout(slider_layout);

    control_label_ = new QLabel("Control");

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Creating start/stop/pause/record buttons.");
    QIcon start_icon = loadPixmap("package://rviz/icons/play.svg");
    start_button_ = new QPushButton(start_icon, "Start");
    start_button_->setToolTip("Start ROSBag replay.");
    start_button_->setCheckable(true);

    QIcon pause_icon = loadPixmap("package://rviz/icons/pause.svg");
    pause_button_ = new QPushButton(pause_icon, "Pause");
    pause_button_->setToolTip("Pause ROSBag replay (if active).");
    pause_button_->setCheckable(true);

    QIcon stop_icon = loadPixmap("package://rviz/icons/stop.svg");
    stop_button_ = new QPushButton(stop_icon, "Stop");
    stop_button_->setToolTip("Stop ROSBag replay (if active).");
    stop_button_->setCheckable(true);

    QIcon record_icon = loadPixmap("package://rviz/icons/record.svg");
    record_button_ = new QPushButton(record_icon, "Record");
    record_button_->setToolTip("Start ROSBag data recording.");
    record_button_->setCheckable(true);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Creating node selection combobox.");
    recorder_nodes_combobox_ = new QComboBox();
    recorder_nodes_combobox_model_ = new QStringListModel();
    recorder_nodes_combobox_->setModel(recorder_nodes_combobox_model_);

    QVBoxLayout* control_mode_layout = new QVBoxLayout();
    control_mode_layout->addWidget(control_mode_group_box_);
    control_mode_layout->addWidget(replay_control_group_box_);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Creating layout for topics + recording/replay options.");
    QVBoxLayout* nodes_topics_layout = new QVBoxLayout();
    QHBoxLayout* node_selection_layout = new QHBoxLayout();
    recorder_player_nodes_label_ = new QLabel("Recorder nodes");
    node_selection_layout->addWidget(recorder_player_nodes_label_);
    node_selection_layout->addWidget(recorder_nodes_combobox_);
    nodes_topics_layout->addLayout(node_selection_layout);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Assembling layouts: Record/replay buttons.");
    QHBoxLayout* record_replay_buttons_layout = new QHBoxLayout();
    record_replay_buttons_layout->addWidget(control_label_);
    record_replay_buttons_layout->addWidget(start_button_);
    record_replay_buttons_layout->addWidget(pause_button_);
    record_replay_buttons_layout->addWidget(record_button_);
    record_replay_buttons_layout->addWidget(stop_button_);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Layouting default topics checkbox.");
    QHBoxLayout* default_topics_layout = new QHBoxLayout();
    use_default_topics_label_ = new QLabel("Use default topics");
    use_default_topics_checkbox_ = new QCheckBox();
    use_default_topics_checkbox_->setChecked(true);

    default_topics_layout->addWidget(use_default_topics_label_);
    default_topics_layout->addWidget(use_default_topics_checkbox_);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Creating topic selection list.");
    topic_selection_list_ = new QCheckList();

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Layouting topic selection.");
    QHBoxLayout* topic_selection_layout = new QHBoxLayout();
    topic_list_label_ = new QLabel("Topics to record/replay");
    topic_selection_layout->addWidget(topic_list_label_);
    topic_selection_layout->addWidget(topic_selection_list_);

    if (use_default_topics_checkbox_->isChecked())
    {
      topic_selection_list_->setEnabled(false);
    }

    if (record_mode_button_->isChecked())
    {
      recordModeToggled(true);
      replayModeToggled(false);
    }
    else if (replay_mode_button_->isChecked())
    {
      recordModeToggled(false);
      replayModeToggled(true);
    }

    QVBoxLayout* secondary_layout = new QVBoxLayout();
    secondary_layout->addLayout(nodes_topics_layout);
    secondary_layout->addLayout(default_topics_layout);
    secondary_layout->addLayout(topic_selection_layout);
    secondary_layout->addLayout(record_replay_buttons_layout);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Assembling layouts, main layout");
    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addLayout(control_mode_layout);
    main_layout->addLayout(secondary_layout);

    connect(replay_mode_button_, SIGNAL(toggled(bool)), this, SLOT(replayModeToggled(bool)));
    connect(record_mode_button_, SIGNAL(toggled(bool)), this, SLOT(recordModeToggled(bool)));

    connect(start_button_, SIGNAL(toggled(bool)), this, SLOT(startToggled(bool)));
    connect(pause_button_, SIGNAL(toggled(bool)), this, SLOT(pauseToggled(bool)));
    connect(record_button_, SIGNAL(toggled(bool)), this, SLOT(recordToggled(bool)));
    connect(stop_button_, SIGNAL(toggled(bool)), this, SLOT(stopToggled(bool)));
    connect(recorder_nodes_combobox_, SIGNAL(currentIndexChanged(int)), this, SLOT(recordReplayNodeIndexChanged(int)));
    connect(use_default_topics_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(useDefaultTopicsToggled(int)));

    recorder_player_nodes_update_timer_ = new QTimer(this);
    connect(recorder_player_nodes_update_timer_, SIGNAL(timeout()), this, SLOT(onRecordReplayNodesUpdateTimer()));
  }

  void RecordReplayPanel::onInitialize()
  {
    connect(vis_manager_, SIGNAL(preUpdate()), this, SLOT(update()));

    DisplayGroup* display_group = vis_manager_->getRootDisplayGroup();
    onDisplayAdded(display_group);

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "******************************************************************");
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "onInitialize() -- starting update timer for ROS node list updates.");
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "******************************************************************");

    recorder_player_nodes_update_timer_->start(5000);
  }

  void RecordReplayPanel::load(const Config& config)
  {
    Panel::load(config);
  }

  void RecordReplayPanel::save(Config config) const
  {
    Panel::save(config);
  }

  void RecordReplayPanel::onDisplayAdded(Display* display)
  {
    DisplayGroup* display_group = qobject_cast<DisplayGroup*>(display);
    if (display_group)
    {
      connect(display_group, SIGNAL(displayAdded(rviz::Display*)), this,
              SLOT(onDisplayAdded(rviz::Display*)));
      connect(display_group, SIGNAL(displayRemoved(rviz::Display*)), this,
              SLOT(onDisplayRemoved(rviz::Display*)));

      for (int i = 0; i < display_group->numDisplays(); i++)
      {
        rviz::Display* display = display_group->getDisplayAt(i);
        onDisplayAdded(display);
      }
    }
    else
    {
      connect(display, SIGNAL(timeSignal(rviz::Display*, ros::Time)), this,
              SLOT(onTimeSignal(rviz::Display*, ros::Time)));
    }
  }

  void RecordReplayPanel::onDisplayRemoved(Display* display)
  {
    /*QString name = display->getName();
    int index = sync_source_selector_->findData(QVariant((qulonglong)display));
    if (index >= 0)
    {
      sync_source_selector_->removeItem(index);
    }*/
  }

  void RecordReplayPanel::onTimeSignal(Display* display, ros::Time time)
  {
    /*QString name = display->getName();
    int index = sync_source_selector_->findData(QVariant((qulonglong)display));

    // if we loaded the sync source name from the config, we need to
    // switch to it as soon as we get a signal
    if (index < 0 && name == config_sync_source_)
    {
      sync_source_selector_->addItem(name, QVariant((qulonglong)display));
      index = sync_source_selector_->findData(QVariant((qulonglong)display));
      sync_source_selector_->setCurrentIndex(index);
      config_sync_source_.clear();
    }

    if (index < 0)
    {
      sync_source_selector_->addItem(name, QVariant((qulonglong)display));
    }
    else
    {
      sync_source_selector_->setItemText(index, name);
      if (sync_source_selector_->currentIndex() == index)
      {
        vis_manager_->getFrameManager()->syncTime(time);
      }
    }*/
  }

  void RecordReplayPanel::update()
  {

  }

  void RecordReplayPanel::recordModeToggled(bool checked)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Record mode radio button toggled: " << (int) checked);
    if (checked)
    {
      control_mode_ = rviz::RECORD_MODE;

      recorder_player_nodes_label_->setText("Recorder nodes");
      topic_list_label_->setText("Topics to record");

      start_button_->setCheckable(false);
      start_button_->setEnabled(false);
      pause_button_->setCheckable(false);
      pause_button_->setEnabled(false);

      record_button_->setEnabled(true);
      record_button_->setCheckable(true);

      stop_button_->setEnabled(true);
      stop_button_->setCheckable(true);

      updateRecorderPlayerNodesList(true);

      replay_msg_selection_slider_->setEnabled(false);
      replay_rosbag_folders_combobox_->setEnabled(false);
      replay_rosbag_files_list_->setEnabled(false);
    }
  }

  void RecordReplayPanel::replayModeToggled(bool checked)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Replay mode radio button toggled: " << (int) checked);
    if (checked)
    {
      control_mode_ = rviz::REPLAY_MODE;

      recorder_player_nodes_label_->setText("Player nodes");
      topic_list_label_->setText("Topics to replay");

      start_button_->setCheckable(true);
      start_button_->setEnabled(true);
      pause_button_->setCheckable(true);
      pause_button_->setEnabled(true);

      record_button_->setEnabled(false);
      record_button_->setCheckable(false);

      stop_button_->setEnabled(true);
      stop_button_->setCheckable(true);

      updateRecorderPlayerNodesList(true);

      replay_msg_selection_slider_->setEnabled(true);
      replay_rosbag_folders_combobox_->setEnabled(true);
      replay_rosbag_files_list_->setEnabled(true);
    }
  }

  void RecordReplayPanel::startToggled(bool checked)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "startToggled(" << (int) checked << ") -- current_recorder_node_name_: " << current_recorder_node_name_);
    if (checked)
    {
      record_button_->setChecked(false);
      record_button_->setEnabled(false);

      pause_button_->setEnabled(true);
      pause_button_->setChecked(false);
      stop_button_->setEnabled(true);
      stop_button_->setCheckable(true);
      stop_button_->setChecked(false);

      start_button_->setCheckable(false);
      pause_button_->setCheckable(true);

      //qDebug() << "Start rosbag replay.";
    }
  }

  void RecordReplayPanel::pauseToggled(bool checked)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "pauseToggled(" << (int) checked << ")" << " -- current_recorder_node_name_: " << current_recorder_node_name_);
    if (checked)
    {
      record_button_->setChecked(false);
      record_button_->setEnabled(false);

      start_button_->setChecked(false);
      start_button_->setCheckable(true);

      stop_button_->setChecked(false);
      stop_button_->setCheckable(true);

      pause_button_->setCheckable(false);
      start_button_->setCheckable(true);

      if (control_mode_ == rviz::RECORD_MODE)
      {

      }
      else if (control_mode_ == rviz::REPLAY_MODE)
      {

      }
    }
  }

  void RecordReplayPanel::stopToggled(bool checked)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "stopToggled(" << (int) checked << ") -- current_recorder_node_name_: " << current_recorder_node_name_);
    if (checked)
    {
      record_button_->setChecked(false);
      record_button_->setCheckable(true);
      record_button_->setEnabled(true);

      start_button_->setChecked(false);
      start_button_->setEnabled(true);

      pause_button_->setChecked(false);
      pause_button_->setEnabled(true);

      stop_button_->setCheckable(false);
      start_button_->setCheckable(true);
      pause_button_->setCheckable(true);

      //qDebug() << "Stop rosbag replay or recording.";
      QString status_message;
      if (control_mode_ == rviz::RECORD_MODE)
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Dispatching recording control command (stop recording).");
        if (dispatchRecordingControl(rviz::STOP_RECORDING, status_message))
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Successfully dispatched command to stop recording:" << status_message.toStdString());
        }
      }
      else if (control_mode_ == rviz::REPLAY_MODE)
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Stop active replay process (if any).");
      }
    }
  }

  void RecordReplayPanel::recordToggled(bool checked)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "recordToggled(" << checked << ") -- current_recorder_node_name_: " << current_recorder_node_name_);
    if (checked)
    {
      start_button_->setChecked(false);
      start_button_->setEnabled(false);

      pause_button_->setChecked(false);
      pause_button_->setEnabled(false);

      stop_button_->setEnabled(true);
      stop_button_->setCheckable(true);
      stop_button_->setChecked(false);

      record_button_->setCheckable(false);

      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Begin rosbag recording.");
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "control_mode_ = " << (int) control_mode_ << "; rviz::RECORD_MODE = " << (int) rviz::RECORD_MODE << ", rviz::REPLAY_MODE = " << (int) rviz::REPLAY_MODE);

      QString status_message;
      if (control_mode_ == rviz::RECORD_MODE)
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Dispatching recording control command (start recording).");
        if (dispatchRecordingControl(rviz::START_RECORDING, status_message))
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Successfully dispatched command to start recording: " << status_message.toStdString());
        }
      }
    }
  }

  void RecordReplayPanel::recordReplayNodeIndexChanged(int index)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "recorderNodeIndexChanged(" << index << ")");
    if (index >= 0)
    {
      QString recorder_node_name = recorder_nodes_combobox_->currentText();
      if (!recorder_node_name.isEmpty())
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "New node for rosbag recording/replay control selected: " << recorder_node_name.toStdString());
        current_recorder_node_name_ = recorder_node_name.toStdString();
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Now calling getRecorderOrPlayerNodeServices()");
        bool player_services = (control_mode_ == rviz::REPLAY_MODE);
        if (getRecorderOrPlayerNodeServices(player_services))
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Successfully retrieved recorder node services for selected node.");
          updateTopicList();
        }
      }
    }
    else
    {
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Empty or invalid record/replay node selection in recorderNodeIndexChanged(): index " << index);
      current_recorder_node_name_ = "";
    }
  }

  void RecordReplayPanel::useDefaultTopicsToggled(int state)
  {
    //qDebug() << "useDefaultTopicsToggled(" << state << ") -- Qt::Checked = " << Qt::CheckState::Checked;
    if (state == 2) // Qt::Checked?
    {
      topic_selection_list_->setEnabled(false);
      topic_selection_list_->setEditable(false);
    }
    else if (state == 0)
    {
      topic_selection_list_->setEnabled(true);
      topic_selection_list_->setEditable(true);
    }
  }

  void RecordReplayPanel::updateRecorderPlayerNodesList(bool force_update)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "updateRecorderPlayerNodesList()");

    ros::V_string node_names;
    if (ros::master::getNodes(node_names))
    {
      std::vector<std::string> all_node_names_tmp;
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Currently active ROS nodes: " << node_names.size());
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Previously known ROS nodes: " << all_node_names_.size());

      bool initial_node_list_buildup = all_node_names_.empty();

      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "initial_node_list_buildup = " << (int) initial_node_list_buildup);

      if (!initial_node_list_buildup)
      {
        for (size_t k = 0; k < all_node_names_.size(); k++)
        {
          if (std::find(node_names.begin(), node_names.end(), all_node_names_[k]) != node_names.end())
          {
            all_node_names_tmp.emplace_back(all_node_names_[k]);
          }
        }
      }
      else
      {
        for (size_t k = 0; k < node_names.size(); k++)
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Initially adding new node: " << node_names[k]);
          all_node_names_.emplace_back(node_names[k]);
          all_node_names_tmp.emplace_back(node_names[k]);
        }
      }

      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "all_node_names_tmp.size() = " << all_node_names_tmp.size() << ", all_node_names_.size() = " << all_node_names_.size());

      bool removed_existing_recorder_node = false;
      bool found_new_recorder_node = false;

      for (size_t k = 0; k < all_node_names_.size(); k++)
      {
        if (std::find(all_node_names_tmp.begin(), all_node_names_tmp.end(), all_node_names_[k]) == all_node_names_tmp.end())
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Node no longer active: " << all_node_names_[k]);
          if (boost::algorithm::ends_with(all_node_names_[k], "_data_recorder") || boost::algorithm::ends_with(all_node_names_[k], "_data_player_worker"))
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Removing data recorder node that is no longer active: " << all_node_names_[k]);
            recorder_player_node_names_.erase(std::remove(recorder_player_node_names_.begin(), recorder_player_node_names_.end(), all_node_names_[k]), recorder_player_node_names_.end());

            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Recorder node names list size now: " << recorder_player_node_names_.size() << " nodes.");
            for (size_t k = 0; k < recorder_player_node_names_.size(); k++)
            {
              ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", " * " << recorder_player_node_names_[k]);
            }
            removed_existing_recorder_node = true;
          }
        }
      }

      for (size_t k = 0; k < node_names.size(); k++)
      {
        if (std::find(all_node_names_.begin(), all_node_names_.end(), node_names[k]) == all_node_names_.end())
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "New node found: " << node_names[k].c_str());
          all_node_names_tmp.emplace_back(node_names[k]);
        }
        if (boost::algorithm::ends_with(node_names[k], "_data_recorder") || boost::algorithm::ends_with(all_node_names_[k], "_data_player_worker"))
        {
          if (std::find(recorder_player_node_names_.begin(), recorder_player_node_names_.end(), node_names[k]) == recorder_player_node_names_.end())
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Found new data recorder node: " << node_names[k]);
            recorder_player_node_names_.emplace_back(node_names[k]);
            found_new_recorder_node = true;
          }
        }
      }

      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "*******************************************************************************************");
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "found_new_recorder_node = " << (int) found_new_recorder_node << ", removed_existing_recorder_node = " << (int) removed_existing_recorder_node);
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "*******************************************************************************************");

      all_node_names_ = all_node_names_tmp;

      if (found_new_recorder_node || removed_existing_recorder_node || force_update)
      {
        QStringList current_cb_items = recorder_nodes_combobox_model_->stringList();
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Current ComboBox items: " << current_cb_items.size() << " items.");
        if (current_cb_items.size() != (int) recorder_player_node_names_.size())
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Number of registered recorder nodes changed, update of node list in ComboBox is required.");
          QString selected_node_name = recorder_nodes_combobox_->currentText();
          int selected_node_index = current_cb_items.indexOf(selected_node_name);
          if (current_cb_items.contains(selected_node_name))
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Currently selected recorder node is still in updated node list at index: " << selected_node_index);
          }
          else
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "No node selected yet, or previously selected node is no longer in updated node list. Picking first entry.");
            selected_node_index = 0;
          }

          bool recorder_node_list_changed = false;

          QStringList updated_cb_items;
          for (int k = 0; k < current_cb_items.size(); k++)
          {
            if (std::find(recorder_player_node_names_.begin(), recorder_player_node_names_.end(), current_cb_items[k].toStdString()) != recorder_player_node_names_.end())
            {
              updated_cb_items.append(current_cb_items[k]);
            }
          }

          for (size_t k = 0; k < recorder_player_node_names_.size(); k++)
          {
            QString recorder_node_name = QString::fromStdString(recorder_player_node_names_[k]);
            if (!updated_cb_items.contains(recorder_node_name))
            {
              updated_cb_items.append(recorder_node_name);
              recorder_node_list_changed = true;
            }
          }

          if (recorder_node_list_changed)
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "==========================================================");
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Recorder/player node list changed, updating ComboBox data.");
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "==========================================================");

            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "New ComboBox items: " << updated_cb_items.size());
            recorder_nodes_combobox_->clear();

            if (control_mode_ == rviz::RECORD_MODE)
            {
              ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Only adding recorder node names to selection list.");
              QStringList recorder_node_names;
              for (int k = 0; k < updated_cb_items.size(); k++)
              {
                if (boost::algorithm::ends_with(updated_cb_items[k].toStdString(), "_data_recorder"))
                {
                  recorder_node_names.append(updated_cb_items[k]);
                }
              }
              recorder_nodes_combobox_model_->setStringList(recorder_node_names);
            }
            else if (control_mode_ == rviz::REPLAY_MODE)
            {
              ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Only adding player node names to selection list.");
              QStringList player_node_names;
              for (int k = 0; k < updated_cb_items.size(); k++)
              {
                if (boost::algorithm::ends_with(updated_cb_items[k].toStdString(), "_data_player_worker"))
                {
                  player_node_names.append(updated_cb_items[k]);
                }
              }
              recorder_nodes_combobox_model_->setStringList(player_node_names);
            }
          }

          recorder_nodes_combobox_->setCurrentIndex(selected_node_index);
        }
      }
      else
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Recorder/player node list is unchanged, not updating ComboBox.");
      }
    }
  }

  void RecordReplayPanel::updateRosbagsForReplayList()
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "updateRosbagsForReplayList()");
    if (control_mode_ == rviz::REPLAY_MODE)
    {
      if (!recorder_nodes_combobox_->currentText().isEmpty())
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "In REPLAY_MODE, querying available rosbag sets from selected player node: " << recorder_nodes_combobox_->currentText().toStdString());

        replay_rosbag_folders_combobox_->setEnabled(true);
        replay_rosbag_files_list_->setEnabled(true);
      }
      else
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "In REPLAY_MODE, but no player node is selected or no player nodes are active. Disabling and clearing replay control GUI elements.");

        replay_rosbag_folders_combobox_->clear();
        replay_rosbag_folders_combobox_->setEnabled(false);
        replay_rosbag_files_list_->clear();
        replay_rosbag_files_list_->setEnabled(false);
      }
    }
    else
    {
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "In RECORD_MODE, disabling and clearing rosbag replay GUI controls.");

      replay_rosbag_folders_combobox_->clear();
      replay_rosbag_folders_combobox_->setEnabled(false);
      replay_rosbag_files_list_->clear();
      replay_rosbag_files_list_->setEnabled(false);

    }
  }

  void RecordReplayPanel::onRecordReplayNodesUpdateTimer()
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "onRecordReplayNodesUpdateTimer()");
    updateRecorderPlayerNodesList(false);
  }

  bool RecordReplayPanel::getRecorderOrPlayerNodeServices(bool player_services)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "In getRecorderOrPlayerNodeServices()");
    XmlRpc::XmlRpcValue req = current_recorder_node_name_;
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;

    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Calling getSystemState operation on ROS master to retrieve list of available ROS services.");
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Current recorder control node name: " << current_recorder_node_name_);

    auto last_slash_idx = current_recorder_node_name_.find_last_of("/");

    if (last_slash_idx != std::string::npos)
    {
      std::string robot_node_name = current_recorder_node_name_.substr(1, last_slash_idx - 1);
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Robot node name: " << robot_node_name);

      ros::master::execute("getSystemState", req, res, pay, true);

      if (res.size() > 0)
      {
        //qDebug() << "Result size = " << res.size();
        /*for (int k = 0; k < res.size(); k++)
        {
          //qDebug() << " * Record " << k << ": " << res[k].toXml().c_str();
        }*/

        std::vector<std::string> ros_services;
        for(int x = 0; x < res[2][2].size(); x++)
        {
          std::string gh = res[2][2][x][0].toXml().c_str();
          // Remove <value>/</value> tags
          gh.erase(gh.begin(), gh.begin() + 7);
          gh.erase(gh.end() - 8, gh.end());
          ros_services.emplace_back(gh);
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "ROS service record: " << gh);
        }

        std::sort(ros_services.begin(), ros_services.end());

        bool current_recorder_node_control_srv_found = false;
        std::string current_recorder_node_control_srv_uri;
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Active ROS services: " << ros_services.size());
        for (int k = 0; k < ros_services.size(); k++)
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", " * Service " << k << ": " << ros_services[k]);
          if (boost::algorithm::starts_with(ros_services[k], "/cc_data_recorders/" + robot_node_name))
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Found recording control service: " << ros_services[k]);
            current_recorder_node_control_srv_found = true;
            current_recorder_node_control_srv_uri = ros_services[k];
            break;
          }
        }

        if (current_recorder_node_control_srv_found)
        {
          current_recorder_node_control_srv_uri_ = current_recorder_node_control_srv_uri;
        }
        else
        {
          current_recorder_node_control_srv_uri_.clear();
          ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "Did not find recorder node control service for recorder node " << robot_node_name << "!");
          return false;
        }
      }
      else
      {
        ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "getSystemState() did not return any information!");
        return false;
      }
      return true;
    }

    if (player_services)
      ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "Failed to query player node services for currently selected player node " << current_player_node_name_);
    else
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Failed to query recorder node services for currently selected recorder node " << current_recorder_node_name_);

    return false;
  }

  bool RecordReplayPanel::updateTopicList()
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "updateTopicList()");

    XmlRpc::XmlRpcValue params("ros_topic_list");
    XmlRpc::XmlRpcValue results;
    XmlRpc::XmlRpcValue r;

    bool initial_topic_list_buildup = topic_list_.empty();
    std::vector<std::string> topic_list_tmp;

    if (ros::master::execute("getTopicTypes", params, results, r, false) == true)
    {
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Retrieved list of topics from roscore.");
      if (results.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        int32_t i = 2;
        if (results[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          for (int32_t j = 0; j < results[i].size(); ++j)
          {
            if (results[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
              if (results[i][j].size() == 2)
              {
                if (results[i][j][0].getType() == XmlRpc::XmlRpcValue::TypeString
                    && results[i][j][1].getType() == XmlRpc::XmlRpcValue::TypeString)
                {
                  std::string topic = static_cast<std::string>(results[i][j][0]);
                  std::string type = static_cast<std::string>(results[i][j][1]);
                  ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", " * Topic : " << topic << " -> " << type);

                  if (initial_topic_list_buildup)
                  {
                    topic_list_.emplace_back(topic);
                    topic_list_tmp.emplace_back(topic);
                  }
                  else
                  {
                    if (std::find(topic_list_.begin(), topic_list_.end(), topic) == topic_list_.end())
                    {
                      topic_list_tmp.emplace_back(topic);
                    }
                  }
                }
              }
            }
          }
        }

        for (size_t k = 0; k < topic_list_.size(); k++)
        {
          if (std::find(topic_list_tmp.begin(), topic_list_tmp.end(), topic_list_[k]) != topic_list_.end())
          {
            topic_list_tmp.emplace_back(topic_list_[k]);
          }
        }

        if (initial_topic_list_buildup || topic_list_.size() != topic_list_tmp.size())
        {
          topic_selection_list_->clear();
          for (size_t k = 0; k < topic_list_.size(); k++)
          {
            ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", " * Adding topic " << k << ": " << topic_list_[k]);
            topic_selection_list_->addCheckItem(QString::fromStdString(topic_list_[k]), QVariant(QString::fromStdString(topic_list_[k])), Qt::CheckState::Checked);
          }
        }

        return true;
      }
    }
    return false;
  }

  bool RecordReplayPanel::dispatchRecordingControl(const RecordingControlRequest control_request, QString& status_message)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "dispatchRecordingControl() called: control_request = " << (int) control_request);

    if (control_mode_ != rviz::RECORD_MODE)
    {
      ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "Replay mode is active, not dispatching record control request.");
      return false;
    }

    carecules_rosbag_msgs::CCRosbagRecordRequest record_request;
    carecules_rosbag_msgs::CCRosbagRecordResponse record_response;

    if (use_default_topics_checkbox_->checkState() == Qt::Checked)
    {
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Instructing recorder node to use its default topic list.");

      record_request.useDefaultTopics = true;
    }
    else
    {
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Providing user-defined topic list for recording.");

      QStandardItemModel* topic_selection_model = qobject_cast<QStandardItemModel*>(topic_selection_list_->model());
      record_request.useDefaultTopics = false;
      for (int i = 0; i < topic_selection_model->rowCount(); i++)
      {
        if (topic_selection_model->item(i)->checkState() == Qt::Checked)
        {
          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", " * Adding topic to list of topics to record: " << topic_selection_model->item(i)->text().toStdString());

          record_request.topicsToRecord.emplace_back(topic_selection_model->item(i)->text().toStdString());
        }
      }
    }

    record_request.bag_file_base_name = bag_file_base_name_;
    record_request.bag_file_compression_type = bag_file_compression_type_;
    record_request.bag_file_naming_mode = bag_file_naming_mode_;
    record_request.bag_file_size_limit = bag_file_size_limit_;

    if (control_request == rviz::START_RECORDING)
    {
      record_request.startRecording = true;
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Start recording.");
    }
    else if (control_request == rviz::STOP_RECORDING)
    {
      record_request.startRecording = false;
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Stop recording.");
    }

    ros::ServiceClient recording_service_client = nh_->serviceClient<carecules_rosbag_msgs::CCRosbagRecord>(current_recorder_node_control_srv_uri_);
    if (recording_service_client.waitForExistence(ros::Duration(0.05)))
    {
      ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Recorder node control is available: " << current_recorder_node_control_srv_uri_);
      if (recording_service_client.call(record_request, record_response))
      {
        ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "Service call to recorder node control service successful.");
        if (record_response.recordingActive && control_request == rviz::START_RECORDING)
        { 
          status_message = QString("Recording started successfully: ") + QString::fromStdString(record_response.recordingStatus);

          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", status_message.toStdString());

          return true;
        }
        else if (!record_response.recordingActive && control_request == rviz::START_RECORDING)
        {
          status_message = QString("Recording failed to start: ") + QString::fromStdString(record_response.recordingStatus);

          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", status_message.toStdString());

          return false;
        }
        else if (!record_response.recordingActive && control_request == rviz::STOP_RECORDING)
        {
          status_message = QString("Recording stopped successfully: ") + QString::fromStdString(record_response.recordingStatus);

          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", status_message.toStdString());

          return true;
        }
        else if (record_response.recordingActive && control_request == rviz::STOP_RECORDING)
        {
          status_message = QString("Recording failed to stop: ") + QString::fromStdString(record_response.recordingStatus);

          ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", status_message.toStdString());

          return false;
        }
      }
      else
      {
        ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "Failed to call service '" << current_recorder_node_control_srv_uri_ << "' when dispatching recording control request!");
      }
    }
    else
    {
      ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "Failed to call service '" << current_recorder_node_control_srv_uri_ << "' when dispatching recording control request. Service does not exist!");
    }

    return false;
  }

  bool RecordReplayPanel::dispatchReplayControl(const ReplayControlRequest control_request, QString &status_message)
  {
    ROS_INFO_STREAM_NAMED("rviz_record_replay_panel", "dispatchReplayControl() called.");

    if (control_mode_ != rviz::REPLAY_MODE)
    {
      ROS_WARN_STREAM_NAMED("rviz_record_replay_panel", "Record mode is active, not dispatching replay control request.");
      return false;
    }

    return true;
  }

} // namespace rviz

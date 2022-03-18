
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
    // TODO: Expose via inputs
    bag_file_base_name_ = "ros_bag";
    bag_file_compression_type_ = 0; // rosbag::CompressonType::Uncompressed
    bag_file_naming_mode_ = 1; // BagWriter::Naming::AppendTimestamp
    bag_file_size_limit_ = "100G";

    nh_.reset(new ros::NodeHandle("cc_record_replay_control"));

    qInfo() << "Creating control mode widgets";
    control_mode_group_box_ = new QGroupBox("Control mode");
    replay_mode_button_ = new QRadioButton(tr("&Replay"));
    record_mode_button_ = new QRadioButton(tr("R&ecord"));
    record_mode_button_->setChecked(true);
    replay_mode_button_->setChecked(false);

    QHBoxLayout* inner_control_mode_layout = new QHBoxLayout();
    inner_control_mode_layout->addWidget(record_mode_button_);
    inner_control_mode_layout->addWidget(replay_mode_button_);
    control_mode_group_box_->setLayout(inner_control_mode_layout);

    control_label_ = new QLabel();
    control_label_->setText("Control");

    qInfo() << "Creating start/stop/record buttons";
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

    qInfo() << "Creating node selection combobox";
    recorder_nodes_combobox_ = new QComboBox();
    recorder_nodes_combobox_model_ = new QStringListModel();
    recorder_nodes_combobox_->setModel(recorder_nodes_combobox_model_);

    QHBoxLayout* control_mode_layout = new QHBoxLayout();
    control_mode_layout ->addWidget(control_mode_group_box_);

    qInfo() << "Creating layout for topics + recording options";
    QVBoxLayout* nodes_topics_layout = new QVBoxLayout();
    QHBoxLayout* node_selection_layout = new QHBoxLayout();
    recorder_nodes_label_ = new QLabel("Record/replay nodes");
    node_selection_layout->addWidget(recorder_nodes_label_);
    node_selection_layout->addWidget(recorder_nodes_combobox_);
    nodes_topics_layout->addLayout(node_selection_layout);

    qInfo() << "Assembling layouts: Record/replay buttons";
    QHBoxLayout* record_replay_buttons_layout = new QHBoxLayout();
    record_replay_buttons_layout->addWidget(control_label_);
    record_replay_buttons_layout->addWidget(start_button_);
    record_replay_buttons_layout->addWidget(pause_button_);
    record_replay_buttons_layout->addWidget(record_button_);
    record_replay_buttons_layout->addWidget(stop_button_);

    qInfo() << "Layouting default topics checkbox";
    QHBoxLayout* default_topics_layout = new QHBoxLayout();
    use_default_topics_label_ = new QLabel("Use default topics");
    use_default_topics_checkbox_ = new QCheckBox();
    use_default_topics_checkbox_->setChecked(true);

    default_topics_layout->addWidget(use_default_topics_label_);
    default_topics_layout->addWidget(use_default_topics_checkbox_);

    qInfo() << "Creating topic selection list";
    topic_selection_list_ = new QCheckList();
    //topic_selection_list_->setEnabled(false);
    //topic_selection_list_->setEditable(false);

    qInfo() << "Layouting topic selection.";
    QHBoxLayout* topic_selection_layout = new QHBoxLayout();
    topic_list_label_ = new QLabel("Topics to record/replay");
    topic_selection_layout->addWidget(topic_list_label_);
    topic_selection_layout->addWidget(topic_selection_list_);

    QVBoxLayout* secondary_layout = new QVBoxLayout();
    secondary_layout->addLayout(nodes_topics_layout);
    secondary_layout->addLayout(default_topics_layout);
    secondary_layout->addLayout(topic_selection_layout);
    secondary_layout->addLayout(record_replay_buttons_layout);

    qInfo() << "Assembling layouts, main layout";
    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addLayout(control_mode_layout);
    main_layout->addLayout(secondary_layout);

    connect(replay_mode_button_, SIGNAL(toggled(bool)), this, SLOT(replayModeToggled(bool)));
    connect(record_mode_button_, SIGNAL(toggled(bool)), this, SLOT(recordModeToggled(bool)));

    connect(start_button_, SIGNAL(toggled(bool)), this, SLOT(startToggled(bool)));
    connect(pause_button_, SIGNAL(toggled(bool)), this, SLOT(pauseToggled(bool)));
    connect(record_button_, SIGNAL(toggled(bool)), this, SLOT(recordToggled(bool)));
    connect(stop_button_, SIGNAL(toggled(bool)), this, SLOT(stopToggled(bool)));
    connect(recorder_nodes_combobox_, SIGNAL(currentIndexChanged(int)), this, SLOT(recorderNodeIndexChanged(int)));
    connect(use_default_topics_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(useDefaultTopicsToggled(int)));

    record_nodes_update_timer_ = new QTimer(this);
    connect(record_nodes_update_timer_, SIGNAL(timeout()), this, SLOT(onRecordNodesUpdateTimer()));
  }

  void RecordReplayPanel::onInitialize()
  {
    connect(vis_manager_, SIGNAL(preUpdate()), this, SLOT(update()));

    DisplayGroup* display_group = vis_manager_->getRootDisplayGroup();
    onDisplayAdded(display_group);

    record_nodes_update_timer_->start(5000);
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
    qInfo() << "Record mode radio button toggled: " << checked;
    if (checked)
    {
      control_mode_ = rviz::RECORD_MODE;

      start_button_->setCheckable(false);
      start_button_->setEnabled(false);
      pause_button_->setCheckable(false);
      pause_button_->setEnabled(false);

      record_button_->setEnabled(true);
      record_button_->setCheckable(true);

      stop_button_->setEnabled(true);
      stop_button_->setCheckable(true);
    }
  }
  void RecordReplayPanel::replayModeToggled(bool checked)
  {
    qInfo() << "Replay mode radio button toggled: " << checked;
    if (checked)
    {
      control_mode_ = rviz::REPLAY_MODE;

      start_button_->setCheckable(true);
      start_button_->setEnabled(true);
      pause_button_->setCheckable(true);
      pause_button_->setEnabled(true);

      record_button_->setEnabled(false);
      record_button_->setCheckable(false);

      stop_button_->setEnabled(true);
      stop_button_->setCheckable(true);
    }
  }

  void RecordReplayPanel::startToggled(bool checked)
  {
    qInfo() << "startToggled(" << checked << ") -- current_recorder_node_name_: " << current_recorder_node_name_.c_str();
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

      qInfo() << "Start rosbag replay.";
    }
  }

  void RecordReplayPanel::pauseToggled(bool checked)
  {
    qInfo() << "pauseToggled(" << checked << ")";
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

      qInfo() << "Pause rosbag replay. -- current_recorder_node_name_: " << current_recorder_node_name_.c_str();
    }
  }

  void RecordReplayPanel::stopToggled(bool checked)
  {
    qInfo() << "stopToggled(" << checked << ") -- current_recorder_node_name_: " << current_recorder_node_name_.c_str();
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

      qInfo() << "Stop rosbag replay or recording.";
      QString status_message;
      if (control_mode_ == rviz::RECORD_MODE)
      {
        qInfo() << "Dispatching recording control command (stop recording).";
        if (dispatchRecordingControl(rviz::STOP_RECORDING, status_message))
        {
          qInfo() << "Successfully dispatched command to stop recording.";
        }
      }
      else if (control_mode_ == rviz::REPLAY_MODE)
      {
        qInfo() << "Stop active replay process (if any).";
      }
    }
  }

  void RecordReplayPanel::recordToggled(bool checked)
  {
    qInfo() << "recordToggled(" << checked << ") -- current_recorder_node_name_: " << current_recorder_node_name_.c_str();
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

      qInfo() << "Begin rosbag recording";
      qInfo() << "control_mode_ = " << control_mode_ << "; rviz::RECORD_MODE = " << rviz::RECORD_MODE << ", rviz::REPLAY_MODE = " << rviz::REPLAY_MODE;

      QString status_message;
      if (control_mode_ == rviz::RECORD_MODE)
      {
        qInfo() << "Dispatching recording control command (start recording).";
        if (dispatchRecordingControl(rviz::START_RECORDING, status_message))
        {
          qInfo() << "Successfully dispatched command to start recording.";
        }
      }
    }
  }

  void RecordReplayPanel::recorderNodeIndexChanged(int index)
  {
    qInfo() << "recorderNodeIndexChanged(" << index << ")";
    if (index >= 0)
    {
      QString recorder_node_name = recorder_nodes_combobox_->currentText();
      if (!recorder_node_name.isEmpty())
      {
        qInfo() << "New recorder node for rosbag recording control selected: " << recorder_node_name;
        current_recorder_node_name_ = recorder_node_name.toStdString();
        qInfo() << "Now calling getRecorderNodeServices()";
        if (getRecorderNodeServices())
        {
          qInfo() << "Successfully retrieved recorder node services for selected node.";
          updateTopicList();
        }
      }
    }
    else
    {
      current_recorder_node_name_ = "";
    }
  }

  void RecordReplayPanel::useDefaultTopicsToggled(int state)
  {
    qInfo() << "useDefaultTopicsToggled(" << state << ") -- Qt::Checked = " << Qt::CheckState::Checked;
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


  void RecordReplayPanel::onRecordNodesUpdateTimer()
  {
    qInfo() << "onRecordNodesUpdateTimer()";
    ros::V_string node_names;
    if (ros::master::getNodes(node_names))
    {
      std::vector<std::string> all_node_names_tmp;
      qInfo() << "Currently active ROS nodes: " << node_names.size();
      qInfo() << "Previously known ROS nodes: " << all_node_names_.size();

      bool initial_node_list_buildup = all_node_names_.empty();

      qInfo() << "initial_node_list_buildup = " << (int) initial_node_list_buildup;

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
          qInfo() << "Initially adding new node: " << node_names[k].c_str();
          all_node_names_.emplace_back(node_names[k]);
          all_node_names_tmp.emplace_back(node_names[k]);
        }
      }

      qInfo() << "all_node_names_tmp.size() = " << all_node_names_tmp.size() << ", all_node_names_.size() = " << all_node_names_.size();

      bool removed_existing_recorder_node = false;
      bool found_new_recorder_node = false;

      for (size_t k = 0; k < all_node_names_.size(); k++)
      {
        if (std::find(all_node_names_tmp.begin(), all_node_names_tmp.end(), all_node_names_[k]) == all_node_names_tmp.end())
        {
          qInfo() << "Node no longer active: " << all_node_names_[k].c_str();
          if (boost::algorithm::ends_with(all_node_names_[k], "_data_recorder"))
          {
            qInfo() << "Removing data recorder node that is no longer active: " << all_node_names_[k].c_str();
            recorder_node_names_.erase(std::remove(recorder_node_names_.begin(), recorder_node_names_.end(), all_node_names_[k]), recorder_node_names_.end());

            qInfo() << "Recorder node names list size now: " << recorder_node_names_.size() << " nodes.";
            for (size_t k = 0; k < recorder_node_names_.size(); k++)
            {
              qInfo() << " * " << recorder_node_names_[k].c_str();
            }
            removed_existing_recorder_node = true;
          }
        }
      }

      for (size_t k = 0; k < node_names.size(); k++)
      {
        if (std::find(all_node_names_.begin(), all_node_names_.end(), node_names[k]) == all_node_names_.end())
        {
          qInfo() << "New node found: " << node_names[k].c_str();
          all_node_names_tmp.emplace_back(node_names[k]);
        }
        if (boost::algorithm::ends_with(node_names[k], "_data_recorder"))
        {
          if (std::find(recorder_node_names_.begin(), recorder_node_names_.end(), node_names[k]) == recorder_node_names_.end())
          {
            qInfo() << "Found new data recorder node: " << node_names[k].c_str();
            recorder_node_names_.emplace_back(node_names[k]);
            found_new_recorder_node = true;
          }
        }
      }

      qInfo() << "*******************************************************************************************";
      qInfo() << "found_new_recorder_node = " << (int) found_new_recorder_node << ", removed_existing_recorder_node = " << (int) removed_existing_recorder_node;
      qInfo() << "*******************************************************************************************";

      all_node_names_ = all_node_names_tmp;

      if (found_new_recorder_node || removed_existing_recorder_node)
      {
        QStringList current_cb_items = recorder_nodes_combobox_model_->stringList();
        qInfo() << "Current ComboBox items: " << current_cb_items;
        if (current_cb_items.size() != recorder_node_names_.size())
        {
          qInfo() << "Number of registered recorder nodes changed, update of node list in ComboBox is required.";
          QString selected_node_name = recorder_nodes_combobox_->currentText();
          int selected_node_index = current_cb_items.indexOf(selected_node_name);
          if (current_cb_items.contains(selected_node_name))
          {
            qInfo() << "Currently selected recorder node is still in updated node list at index: " << selected_node_index;
          }
          else
          {
            qInfo() << "No node selected yet, or previously selected node is no longer in updated node list. Picking first entry.";
            selected_node_index = 0;
          }

          bool recorder_node_list_changed = false;

          QStringList updated_cb_items;
          for (int k = 0; k < current_cb_items.size(); k++)
          {
            if (std::find(recorder_node_names_.begin(), recorder_node_names_.end(), current_cb_items[k].toStdString()) != recorder_node_names_.end())
            {
              updated_cb_items.append(current_cb_items[k]);
            }
          }

          for (size_t k = 0; k < recorder_node_names_.size(); k++)
          {
            QString recorder_node_name = QString::fromStdString(recorder_node_names_[k]);
            if (!updated_cb_items.contains(recorder_node_name))
            {
              updated_cb_items.append(recorder_node_name);
              recorder_node_list_changed = true;
            }
          }

          qInfo() << "===================================================";
          qInfo() << "Recorder node list changed, updating ComboBox data.";
          qInfo() << "===================================================";

          qInfo() << "New ComboBox items: " << updated_cb_items;
          recorder_nodes_combobox_->clear();
          recorder_nodes_combobox_model_->setStringList(updated_cb_items);

          recorder_nodes_combobox_->setCurrentIndex(selected_node_index);
        }
      }
      else
      {
        qInfo() << "Recorder node list is unchanged, not updating ComboBox.";
      }
    }
  }

  bool RecordReplayPanel::getRecorderNodeServices()
  {
    qInfo() << "In getRecorderNodeServices()";
    XmlRpc::XmlRpcValue req = current_recorder_node_name_;
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;

    qInfo() << "Calling getSystemState operation on ROS master to retrieve list of available ROS services.";
    qInfo() << "Current recorder control node name: " << current_recorder_node_name_.c_str();

    auto last_slash_idx = current_recorder_node_name_.find_last_of("/");

    if (last_slash_idx != std::string::npos)
    {
      std::string robot_node_name = current_recorder_node_name_.substr(1, last_slash_idx - 1);
      qInfo() << "Robot node name: " << robot_node_name.c_str();

      ros::master::execute("getSystemState", req, res, pay, true);

      if (res.size() > 0)
      {
        qInfo() << "Result size = " << res.size();
        /*for (int k = 0; k < res.size(); k++)
        {
          qInfo() << " * Record " << k << ": " << res[k].toXml().c_str();
        }*/

        std::vector<std::string> ros_services;
        for(int x = 0; x < res[2][2].size(); x++)
        {
          std::string gh = res[2][2][x][0].toXml().c_str();
          // Remove <value>/</value> tags
          gh.erase(gh.begin(), gh.begin() + 7);
          gh.erase(gh.end() - 8, gh.end());
          ros_services.emplace_back(gh);
          // qInfo() << "ROS service record: " << gh.c_str();
        }

        std::sort(ros_services.begin(), ros_services.end());

        bool current_recorder_node_control_srv_found = false;
        std::string current_recorder_node_control_srv_uri;
        qInfo() << "Active ROS services: " << ros_services.size();
        for (int k = 0; k < ros_services.size(); k++)
        {
          qInfo() << " * Service " << k << ": " << ros_services[k].c_str();
          if (boost::algorithm::starts_with(ros_services[k], "/cc_data_recorders/" + robot_node_name))
          {
            qInfo() << "Found recording control service: " << ros_services[k].c_str();
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
          qWarning() << "Did not find recorder node control service for recorder node " << robot_node_name.c_str() << "!";
          return false;
        }
      }
      else
      {
        qWarning() << "getSystemState did not return any data!";
        return false;
      }
      return true;
    }

    return false;
  }

  bool RecordReplayPanel::updateTopicList()
  {
    XmlRpc::XmlRpcValue params("ros_topic_list");
    XmlRpc::XmlRpcValue results;
    XmlRpc::XmlRpcValue r;

    bool initial_topic_list_buildup = topic_list_.empty();
    std::vector<std::string> topic_list_tmp;

    if (ros::master::execute("getTopicTypes", params, results, r, false) == true)
    {
      qInfo() << "Retrieved list of topics from roscore.";
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
                  qInfo() << "Topic : " << topic.c_str() << " -> " << type.c_str();

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
            qInfo() << "Adding topic " << k << ": " << QString::fromStdString(topic_list_[k]);
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
    qInfo() << "dispatchRecordingControl() called.";

    if (control_mode_ != rviz::RECORD_MODE)
    {
      qWarning() << "Replay mode is active, not dispatching record control request.";
      return false;
    }
    carecules_rosbag_msgs::CCRosbagRecordRequest record_request;
    carecules_rosbag_msgs::CCRosbagRecordResponse record_response;

    if (use_default_topics_checkbox_->checkState() == Qt::Checked)
    {
      qInfo() << "Instructing recorder node to use its default topic list.";
      record_request.useDefaultTopics = true;
    }
    else
    {
      qInfo() << "Providing user-defined topic list for recording.";
      QStandardItemModel* topic_selection_model = qobject_cast<QStandardItemModel*>(topic_selection_list_->model());
      record_request.useDefaultTopics = false;
      for (int i = 0; i < topic_selection_model->rowCount(); i++)
      {
        if (topic_selection_model->item(i)->checkState() == Qt::Checked)
        {
          qInfo() << "Adding topic to list of topics to record: " << topic_selection_model->item(i)->text();
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
      qInfo() << "Start recording.";
    }
    else if (control_request == rviz::STOP_RECORDING)
    {
      record_request.startRecording = false;
      qInfo() << "Stop recording.";
    }

    ros::ServiceClient recording_service_client = nh_->serviceClient<carecules_rosbag_msgs::CCRosbagRecord>(current_recorder_node_control_srv_uri_);
    if (recording_service_client.waitForExistence(ros::Duration(0.05)))
    {
      qInfo() << "Recorder node control is available: " << current_recorder_node_control_srv_uri_.c_str();
      if (recording_service_client.call(record_request, record_response))
      {
        qInfo() << "Service call to recorder node control service successful.";
        if (record_response.recordingActive && control_request == rviz::START_RECORDING)
        {
          status_message = QString("Recording started successfully: ") + QString::fromStdString(record_response.recordingStatus);
          return true;
        }
        else if (!record_response.recordingActive && control_request == rviz::START_RECORDING)
        {
          status_message = QString("Recording failed to start: ") + QString::fromStdString(record_response.recordingStatus);
          return false;
        }
        else if (!record_response.recordingActive && control_request == rviz::STOP_RECORDING)
        {
          status_message = QString("Recording stopped successfully: ") + QString::fromStdString(record_response.recordingStatus);
          return true;
        }
        else if (record_response.recordingActive && control_request == rviz::STOP_RECORDING)
        {
          status_message = QString("Recording failed to stop: ") + QString::fromStdString(record_response.recordingStatus);
          return false;
        }
      }
    }

    return false;
  }

  bool RecordReplayPanel::dispatchReplayControl(const ReplayControlRequest control_request, QString &status_message)
  {
    qInfo() << "dispatchReplayControl() called.";

    if (control_mode_ != rviz::REPLAY_MODE)
    {
      qWarning() << "Record mode is active, not dispatching replay control request.";
      return false;
    }

    return true;
  }

} // namespace rviz

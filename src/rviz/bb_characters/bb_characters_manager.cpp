#include "bb_characters_manager.h"

namespace bb_characters_rviz
{
  CharactersManager::CharactersManager(QObject *parent) : QObject(parent)
  {

  }

  bool CharactersManager::initialize()
  {
    ROS_INFO_STREAM_NAMED("character_manager", "initialize()");
    // Populate default characters list
    getAvailableModels();

    if (character_visual_infos_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("character_manager", "Could not read default characters list from bb_characters_rviz package!");
      return false;
    }

    return true;
  }

  const CharactersManager::CharacterVisualInfosMap& CharactersManager::getCharacterVisualInfos()
  {
    return character_visual_infos_;
  }

  void CharactersManager::getAvailableModels(const std::string& package_name)
  {
    ROS_INFO_STREAM_NAMED("character_manager", "getAvailableModels(" << package_name << ")");

    std::string package_dir = ros::package::getPath(package_name); // TODO: Make configurable
    ROS_INFO_STREAM_NAMED("character_manager", "Package directory: " << package_dir);
    fs::path package_path(package_dir);
    if (fs::exists(package_path))
    {
      ROS_INFO_STREAM_NAMED("character_manager", "Package directory " << package_dir << " exists, checking for 'media' subdirectory.");
      fs::path models_path = package_path.append("media");
      if (fs::exists(models_path))
      {
        ROS_INFO_STREAM_NAMED("character_manager", "'media' subdirectory exists.");
        fs::directory_iterator model_dir_it{models_path};
        while (model_dir_it != fs::directory_iterator{})
        {
          if (fs::is_directory(*model_dir_it))
          {
            ROS_INFO_STREAM_NAMED("characters_panel", "Subdirectory in 'media': " << model_dir_it->path().string());
            fs::path model_info_yaml_path(model_dir_it->path());
            model_info_yaml_path.append("model_info.yaml");
            if (fs::exists(model_info_yaml_path))
            {
              ROS_INFO_STREAM_NAMED("character_manager", "'model_info.yaml' file found in subdirectory '" << model_dir_it->path().string() << "', adding model.");
              std::string model_folder_name = model_dir_it->path().leaf().string();
              ROS_INFO_STREAM_NAMED("character_manager", "model_folder_name: " << model_folder_name);
              parseModelInfoYamlFile(model_info_yaml_path.string(), model_dir_it->path().string(), model_folder_name);
            }
            else
            {
              ROS_WARN_STREAM_NAMED("character_manager", "No 'model_info.yaml' file found in subdirectory '" << model_dir_it->path().string() << "', ignoring it.");
            }
          }

          model_dir_it++;
        }
      }
      else
      {
        ROS_WARN_STREAM_NAMED("character_manager", "'media' subdirectory does NOT exist!");
      }
    }
    else
    {
      ROS_WARN_STREAM_NAMED("character_manager", "Package directory '" << package_dir << "' does NOT exist!");
    }

    ROS_INFO_STREAM_NAMED("character_manager", "================================================");
    ROS_INFO_STREAM_NAMED("character_manager", "character_visual_infos_.size() = " << character_visual_infos_.size());
    ROS_INFO_STREAM_NAMED("character_manager", "================================================");
  }

  bool CharactersManager::parseModelInfoYamlFile(const std::string& yaml_file_path, const std::string& model_path, const std::string& model_folder_name)
  {
    try
    {
      YAML::Node root_node = YAML::LoadFile(yaml_file_path);
      ROS_INFO_STREAM_NAMED("character_manager", "root_node size = " << root_node.size());

      if (root_node["model_info"])
      {
        std::string model_name, model_author;
        if (root_node["model_info"]["model_name"])
        {
          model_name = root_node["model_info"]["model_name"].as<std::string>();
          ROS_INFO_STREAM_NAMED("character_manager", "model_name: " << model_name);
        }
        if (root_node["model_info"]["model_author"])
        {
          model_author = root_node["model_info"]["model_author"].as<std::string>();
          ROS_INFO_STREAM_NAMED("character_manager", "model_author: " << model_author);
        }

        if (!model_name.empty())
        {
          auto character_visual_info = std::make_shared<CharacterVisualInfo>(model_name, model_folder_name, model_author);
          character_visual_info->parseCharacterInfo(root_node, model_path);
          character_visual_infos_[model_name] = character_visual_info;
        }
        else
        {
          ROS_WARN_STREAM_NAMED("character_manager", "No model_structure element found, can not continue parsing.");
          return false;
        }
      }
    }
    catch (YAML::BadFile& ex)
    {
      ROS_WARN_STREAM_NAMED("character_manager", "Failed to open YAML file " << yaml_file_path << " for reading: " << ex.what());
      return false;
    }
    catch (YAML::ParserException& ex)
    {
      ROS_WARN_STREAM_NAMED("character_manager", "Failed to parse YAML file " << yaml_file_path << ": " << ex.what());
      return false;
    }
    return true;
  }
}

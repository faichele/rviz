#include "bb_character_info.h"

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <angles/angles.h>

namespace bb_characters_rviz
{
  CharacterElement::CharacterElement(const std::string& part_name, const std::string& mesh_file):
    part_name_(part_name), mesh_file_(mesh_file), visible_(false)
  {

  }

  CharacterElement::CharacterElement(const CharacterElement& other)
  {
    if (this != &other)
    {
      part_name_ = other.part_name_;
      mesh_file_ = other.mesh_file_;
      part_parent_ = other.part_parent_;
      visible_ = other.visible_;

      for (auto attach_it = other.attachments_.begin(); attach_it != other.attachments_.end(); attach_it++)
      {
        attachments_.emplace_back(*attach_it);
      }

      for (auto rotation = other.rotations_per_instance_.begin(); rotation != other.rotations_per_instance_.end(); rotation++)
      {
        rotations_per_instance_[rotation->first] = rotation->second;
      }

      for (auto translation = other.translations_per_instance_.begin(); translation != other.translations_per_instance_.end(); translation++)
      {
        translations_per_instance_[translation->first] = translation->second;
      }
    }
  }

  CharacterElement& CharacterElement::operator=(const CharacterElement& other)
  {
    if (this != &other)
    {
      part_name_ = other.part_name_;
      mesh_file_ = other.mesh_file_;
      part_parent_ = other.part_parent_;
      visible_ = other.visible_;

      for (auto attach_it = other.attachments_.begin(); attach_it != other.attachments_.end(); attach_it++)
      {
        attachments_.emplace_back(*attach_it);
      }

      for (auto rotation = other.rotations_per_instance_.begin(); rotation != other.rotations_per_instance_.end(); rotation++)
      {
        rotations_per_instance_[rotation->first] = rotation->second;
      }

      for (auto translation = other.translations_per_instance_.begin(); translation != other.translations_per_instance_.end(); translation++)
      {
        translations_per_instance_[translation->first] = translation->second;
      }
    }
    return *this;
  }

  CharacterVisualInfo::CharacterVisualInfo(const std::string& model_name, const std::string &model_folder, const std::string model_author):
    model_name_(model_name), model_author_(model_author)
  {

  }

  CharacterVisualInfo::CharacterVisualInfo(const CharacterVisualInfo& other)
  {
    if (this != &other)
    {
      model_folder_ = other.model_folder_;
      model_root_folder_ = other.model_root_folder_;
      model_author_ = other.model_author_;
      model_name_ = other.model_name_;
      for (auto mesh_file = other.character_elements_.begin(); mesh_file != other.character_elements_.end(); mesh_file++)
      {
        character_elements_[mesh_file->first] = mesh_file->second;
      }
      for (auto material_script = other.material_scripts_.begin(); material_script != other.material_scripts_.end(); material_script++)
      {
        material_scripts_.emplace_back(*material_script);
      }

      scaling_.setX(other.scaling_.x());
      scaling_.setY(other.scaling_.y());
      scaling_.setZ(other.scaling_.z());

      offsets_.setX(other.offsets_.x());
      offsets_.setY(other.offsets_.y());
      offsets_.setZ(other.offsets_.z());

      translation_.setX(other.translation_.x());
      translation_.setY(other.translation_.y());
      translation_.setZ(other.translation_.z());

      for (auto rotation = other.rotations_.begin(); rotation != other.rotations_.end(); rotation++)
      {
        rotations_[rotation->first] = rotation->second;
      }
    }
  }

  CharacterVisualInfo& CharacterVisualInfo::operator=(const CharacterVisualInfo& other)
  {
    if (this != &other)
    {
      model_folder_ = other.model_folder_;
      model_root_folder_ = other.model_root_folder_;
      model_author_ = other.model_author_;
      model_name_ = other.model_name_;
      for (auto mesh_file = other.character_elements_.begin(); mesh_file != other.character_elements_.end(); mesh_file++)
      {
        character_elements_[mesh_file->first] = mesh_file->second;
      }
      for (auto material_script = other.material_scripts_.begin(); material_script != other.material_scripts_.end(); material_script++)
      {
        material_scripts_.emplace_back(*material_script);
      }

      scaling_.setX(other.scaling_.x());
      scaling_.setY(other.scaling_.y());
      scaling_.setZ(other.scaling_.z());

      offsets_.setX(other.offsets_.x());
      offsets_.setY(other.offsets_.y());
      offsets_.setZ(other.offsets_.z());

      translation_.setX(other.translation_.x());
      translation_.setY(other.translation_.y());
      translation_.setZ(other.translation_.z());

      for (auto rotation = other.rotations_.begin(); rotation != other.rotations_.end(); rotation++)
      {
        rotations_[rotation->first] = rotation->second;
      }
    }
    return *this;
  }

  bool CharacterVisualInfo::parseCharacterInfo(const YAML::Node& yaml_node, const std::string& model_path)
  {
    ROS_INFO_STREAM_NAMED("character_visual_info", "parseCharacterInfo(" << model_name_ << ")");
    if (yaml_node["model_info"] &&
        yaml_node["model_info"]["model_structure"])
    {
      ROS_INFO_STREAM_NAMED("character_visual_info", "Found 'model_info' and 'model_structure' YAML elements.");
      if (yaml_node["model_info"]["model_structure"]["model_parts"])
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "Found 'model_parts' element.");
        YAML::Node model_parts = yaml_node["model_info"]["model_structure"]["model_parts"];

        model_root_folder_ = model_path;

        for (auto part_it = model_parts.begin(); part_it != model_parts.end(); part_it++)
        {
          auto part_key = part_it->first;
          auto part_attribs = part_it->second;

          ROS_INFO_STREAM_NAMED("character_visual_info", " * model_part: " << part_key << " of type " << part_attribs.Type());

          if (part_attribs.IsMap())
          {
            if (part_attribs["name"] && part_attribs["mesh_file"])
            {
              std::string part_name = part_attribs["name"].as<std::string>();
              std::string part_mesh_file = part_attribs["mesh_file"].as<std::string>();
              ROS_INFO_STREAM_NAMED("character_visual_info", "   part_name: " << part_name);
              ROS_INFO_STREAM_NAMED("character_visual_info", "   mesh_file: " << part_mesh_file);

              fs::path model_dir(model_path);
              model_dir / model_name_;
              std::string mesh_file_path;
              ROS_INFO_STREAM_NAMED("character_visual_info", "   Checking if part_mesh_file exists: " << model_dir << "/" << part_mesh_file);
              if (fs::exists(model_dir / part_mesh_file))
              {
                ROS_INFO_STREAM_NAMED("character_visual_info", "   part_mesh_file exists: " << model_dir << "/" << part_mesh_file);
                mesh_file_path = model_dir.string() + "/" + part_mesh_file;
              }
              else
              {
                ROS_WARN_STREAM_NAMED("character_visual_info", "   part_mesh_file NOT FOUND: " << model_dir << "/" << part_mesh_file);
              }

              auto model_part = std::make_shared<bb_characters_rviz::CharacterElement>(part_name, mesh_file_path);

              if (part_attribs["parent"])
              {
                std::string part_parent = part_attribs["parent"].as<std::string>();
                model_part->part_parent_ = part_parent;

                ROS_INFO_STREAM_NAMED("character_visual_info", "   part_parent: " << part_parent);
              }

              if (part_attribs["visible"])
              {
                try
                {
                  ROS_INFO_STREAM_NAMED("character_visual_info", "Found 'visible' element: " << part_attribs["visible"].as<int>());
                  model_part->visible_ = (part_attribs["visible"].as<int>() == 1 ? true : false);
                }
                catch (YAML::TypedBadConversion<int>& ex)
                {
                  ROS_WARN_STREAM_NAMED("character_visual_info", "Failed to read 'visible' element as int value: " << ex.what());
                  ROS_WARN_STREAM_NAMED("character_visual_info", "Clear text value is: " << part_attribs["visible"].as<std::string>());
                  model_part->visible_ = true;
                }
              }

              if (part_attribs["attachments"])
              {
                YAML::Node attachments_node = part_attribs["attachments"];
                ROS_INFO_STREAM_NAMED("character_visual_info", "Found 'attachments' element: " << attachments_node.size() << " attachments specified.");

                for (auto attach_it = attachments_node.begin(); attach_it != attachments_node.end(); attach_it++)
                {
                  auto key = attach_it->first.as<std::string>();
                  auto value = attach_it->second;
                  // ROS_INFO_STREAM_NAMED("character_visual_info", " * Attachment: " << key);
                  // ROS_INFO_STREAM_NAMED("character_visual_info", "   attachment sub-elements: " << value);

                  std::string attached_inst_name(part_name + ";;;" + key);
                  model_part->attachments_.emplace_back(attached_inst_name);

                  if (value["translation"])
                  {
                    // ROS_INFO_STREAM_NAMED("character_visual_info", "   translation element present: " << value["translation"]);
                    parseTranslation(value, part_name, key, false, model_part);
                  }
                  else
                  {
                    std::string translation_inst_name(part_name + ";;;" + key);
                    model_part->translations_per_instance_[translation_inst_name] = tf2::Vector3(0, 0, 0);
                  }

                  if (value["rotations"])
                  {
                    // ROS_INFO_STREAM_NAMED("character_visual_info", "   rotations element present: " << value["rotations"]);
                    parseRotations(value, part_name, key, false, model_part);
                  }
                }
              }

              character_elements_[part_name] = model_part;
            }
          }
          else
          {
            ROS_WARN_STREAM_NAMED("character_visual_info", "YAML structure not of expected type Map! Can not continue parsing.");
          }
        }
      }

      if (yaml_node["model_info"]["material_scripts"])
      {
        YAML::Node material_scripts_node = yaml_node["model_info"]["material_scripts"];
        ROS_INFO_STREAM_NAMED("character_visual_info", "OGRE material scripts for model: " << material_scripts_node.size());
        for (auto material_it = material_scripts_node.begin(); material_it != material_scripts_node.end(); material_it++)
        {
          std::string material_script = material_it->as<std::string>();
          ROS_INFO_STREAM_NAMED("character_visual_info", " * Material script: " << material_script);
          material_scripts_.emplace_back(material_script);
        }
      }

      std::shared_ptr<CharacterElement> fake_element;
      if (yaml_node["model_info"]["translation"])
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "Initial translation is specified.");
        parseTranslation(yaml_node, model_name_, "", true, fake_element);
      }
      else
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "No 'translation' element found, applying no initial translation.");
        translation_.setX(0.0);
        translation_.setY(0.0);
        translation_.setZ(0.0);
      }

      if (yaml_node["model_info"]["rotations"])
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "Parsing rotations element for character model " << model_name_);
        parseRotations(yaml_node, model_name_, "", true, fake_element);
      }

      if (yaml_node["model_info"]["scaling"])
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "Found 'scaling' element.");
        YAML::Node scaling_node = yaml_node["model_info"]["scaling"];
        float scale_factor_x = 1.0;
        float scale_factor_y = 1.0;
        float scale_factor_z = 1.0;
        if (scaling_node["x"])
        {
          scale_factor_x = scaling_node["x"].as<float>();
          ROS_INFO_STREAM_NAMED("character_visual_info", "Scaling factor for X axis: " << scale_factor_x);
        }
        if (scaling_node["y"])
        {
          scale_factor_y = scaling_node["y"].as<float>();
          ROS_INFO_STREAM_NAMED("character_visual_info", "Scaling factor for Y axis: " << scale_factor_y);
        }
        if (scaling_node["z"])
        {
          scale_factor_z = scaling_node["z"].as<float>();
          ROS_INFO_STREAM_NAMED("character_visual_info", "Scaling factor for Z axis: " << scale_factor_z);
        }

        scaling_.setX(scale_factor_x);
        scaling_.setY(scale_factor_y);
        scaling_.setZ(scale_factor_z);
      }
      else
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "No 'scaling' element found, applying standard scaling.");
        scaling_.setX(1.0);
        scaling_.setY(1.0);
        scaling_.setZ(1.0);
      }

      if (yaml_node["model_info"]["offsets"])
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "Found 'offsets' element.");
        YAML::Node offsets_node = yaml_node["model_info"]["offsets"];
        float offset_x = 0.0;
        float offset_y = 0.0;
        float offset_z = 0.0;
        if (offsets_node["x"])
        {
          offset_x = offsets_node["x"].as<float>();
          ROS_INFO_STREAM_NAMED("character_visual_info", "Offset for X axis: " << offset_x);
        }
        if (offsets_node["y"])
        {
          offset_y = offsets_node["y"].as<float>();
          ROS_INFO_STREAM_NAMED("character_visual_info", "Offset for Y axis: " << offset_y);
        }
        if (offsets_node["z"])
        {
          offset_z = offsets_node["z"].as<float>();
          ROS_INFO_STREAM_NAMED("character_visual_info", "Offset for Z axis: " << offset_z);
        }

        offsets_.setX(offset_x);
        offsets_.setY(offset_y);
        offsets_.setZ(offset_z);
      }
      else
      {
        ROS_INFO_STREAM_NAMED("character_visual_info", "No 'offsets' element found, applying standard offsets.");
        offsets_.setX(1.0);
        offsets_.setY(1.0);
        offsets_.setZ(1.0);
      }

      ROS_INFO_STREAM_NAMED("character_visual_info", "Offsets for model: (" << offsets_.x() << "," << offsets_.y() << "," << offsets_.z() << ")");

      return true;
    }

    return false;
  }

  bool CharacterVisualInfo::parseTranslation(const YAML::Node& yaml_node, const std::string& part_name, const std::string& instance_name, bool whole_character, std::shared_ptr<CharacterElement>& character_element)
  {
    YAML::Node translation_node;
    if (whole_character)
    {
      // ROS_INFO_STREAM_NAMED("character_visual_info", "Parsing translation for complete model: " << yaml_node);
      translation_node = yaml_node["model_info"]["translation"];
    }
    else
    {
      // ROS_INFO_STREAM_NAMED("character_visual_info", "Parsing translation for sub-entity.");
      translation_node = yaml_node["translation"];
    }

    float tr_x = 0.0;
    float tr_y = 0.0;
    float tr_z = 0.0;

    tf2::Vector3 translation_vec;

    ROS_INFO_STREAM_NAMED("character_visual_info", "Parsing translation values.");
    if (translation_node["x"])
    {
      tr_x = translation_node["x"].as<float>();
      ROS_INFO_STREAM_NAMED("character_visual_info", "Translation for X axis: " << tr_x);
    }
    if (translation_node["y"])
    {
      tr_y = translation_node["y"].as<float>();
      ROS_INFO_STREAM_NAMED("character_visual_info", "Translation for Y axis: " << tr_y);
    }
    if (translation_node["z"])
    {
      tr_z = translation_node["z"].as<float>();
      ROS_INFO_STREAM_NAMED("character_visual_info", "Translation for Z axis: " << tr_z);
    }

    translation_vec.setX(tr_x);
    translation_vec.setY(tr_y);
    translation_vec.setZ(tr_z);

    if (whole_character)
    {
      ROS_INFO_STREAM_NAMED("character_visual_info", "Applying parsed translation to entire character.");
      translation_ = translation_vec;
    }
    else
    {
      ROS_INFO_STREAM_NAMED("character_visual_info", "Applying parsed translation to sub-entity: " << part_name);
      std::string translation_inst_name(part_name + ";;;" + instance_name);
      character_element->translations_per_instance_[translation_inst_name] = translation_vec;
    }

    ROS_INFO_STREAM_NAMED("character_visual_info", "Translation parsed.");
    return true;
  }

  bool CharacterVisualInfo::parseRotations(const YAML::Node& yaml_node, const std::string& part_name, const std::string& instance_name, bool whole_character, std::shared_ptr<CharacterElement>& character_element)
  {
    YAML::Node rotations_node;

    if (whole_character)
    {
      ROS_INFO_STREAM_NAMED("character_visual_info", "Using 'model_info' -> 'rotations' sub-element, parsing rotations for entire model.");
      rotations_node = yaml_node["model_info"]["rotations"];
    }
    else
    {
      ROS_INFO_STREAM_NAMED("character_visual_info", "Using 'rotations' sub-element, parsing rotations for child entity.");
      rotations_node = yaml_node["rotations"];
    }

    ROS_INFO_STREAM_NAMED("character_visual_info", "Rotations for model (part): " << rotations_node.size());
    for (auto rotation_it = rotations_node.begin(); rotation_it != rotations_node.end(); rotation_it++)
    {
      if (rotation_it->second["axis"] &&
          rotation_it->second["angle"])
      {
        std::string axis_spec = rotation_it->second["axis"].as<std::string>();
        std::string angle_spec = rotation_it->second["angle"].as<std::string>();

        float angle = std::atof(angle_spec.c_str());
        std::vector<std::string> result;
        boost::split(result, axis_spec, boost::is_any_of(","));

        if (result.size() == 3)
        {
          float x_axis_val = std::atof(result[0].c_str());
          float y_axis_val = std::atof(result[1].c_str());
          float z_axis_val = std::atof(result[2].c_str());

          ROS_INFO_STREAM_NAMED("character_visual_info", "Rotation angle (degrees): " << angle);
          ROS_INFO_STREAM_NAMED("character_visual_info", "Rotation axis specification: (" << x_axis_val << "," << y_axis_val << "," << z_axis_val << ")");

          tf2::Quaternion rotation_quat;
          rotation_quat.setRotation(tf2::Vector3(x_axis_val, y_axis_val, z_axis_val), angles::from_degrees(angle));

          if (whole_character)
          {
            ROS_INFO_STREAM_NAMED("character_visual_info", "Applying parsed rotation to entire character.");
            rotations_[rotation_it->first.as<std::string>()] = rotation_quat;
          }
          else
          {
            ROS_INFO_STREAM_NAMED("character_visual_info", "Applying parsed rotation to sub-entity: " << part_name);
            std::string rotation_inst_name(part_name + ";;;" + instance_name);
            character_element->rotations_per_instance_[rotation_inst_name][rotation_it->first.as<std::string>()] = rotation_quat;
          }
        }
        else
        {
          ROS_WARN_STREAM_NAMED("character_visual_info", "Failed to parse rotation axis specification: " << axis_spec << "!");
          return false;
        }
      }
    }
    return true;
  }
}

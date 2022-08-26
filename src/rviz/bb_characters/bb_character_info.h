#ifndef CHARACTERINFO_H
#define CHARACTERINFO_H

#include <string>
#include <map>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace fs = boost::filesystem;

namespace bb_characters_rviz
{
  class CharacterElement
  {
    public:
      CharacterElement(const std::string& part_name = "", const std::string& mesh_file = "");
      CharacterElement(const CharacterElement& other);
      CharacterElement& operator=(const CharacterElement& other);

      std::string part_name_;
      std::string mesh_file_;
      std::string part_parent_;
      bool visible_;

      // std::map<std::string, tf2::Quaternion> rotations_;

      std::vector<std::string> attachments_;

      std::map<std::string, tf2::Vector3> translations_per_instance_;
      std::map<std::string, std::map<std::string, tf2::Quaternion>> rotations_per_instance_;
  };

  class CharacterVisualInfo
  {
    public:
      CharacterVisualInfo(const std::string& model_name, const std::string& model_folder, const std::string model_author);
      CharacterVisualInfo(const CharacterVisualInfo& other);
      CharacterVisualInfo& operator=(const CharacterVisualInfo& other);

      bool parseCharacterInfo(const YAML::Node&, const std::string&);

      bool parseTranslation(const YAML::Node&, const std::string&, const std::string&, bool, std::shared_ptr<CharacterElement>&);
      bool parseRotations(const YAML::Node&, const std::string&, const std::string&, bool, std::shared_ptr<CharacterElement>&);

      std::string model_folder_;
      std::string model_name_;
      std::string model_author_;
      std::string model_root_folder_;

      std::vector<std::string> material_scripts_;

      tf2::Vector3 scaling_;
      tf2::Vector3 offsets_;
      tf2::Transform world_transform_;
      tf2::Vector3 translation_;
      std::map<std::string, tf2::Quaternion> rotations_;

      std::map<std::string, std::shared_ptr<CharacterElement>> character_elements_;
  };
}

#endif // CHARACTERINFO_H

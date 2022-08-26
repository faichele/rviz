#ifndef CHARACTERSMANAGER_H
#define CHARACTERSMANAGER_H

#include <QObject>

#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/qsingleton.h>

#include "bb_character_info.h"

namespace bb_characters_rviz
{
  class CharactersManager : public QObject, public rviz::QSingleton<bb_characters_rviz::CharactersManager>
  {
    Q_OBJECT
    public:
      typedef std::map<std::string, std::shared_ptr<bb_characters_rviz::CharacterVisualInfo>> CharacterVisualInfosMap;

      explicit CharactersManager(QObject *parent = nullptr);

      bool initialize();

      const CharacterVisualInfosMap& getCharacterVisualInfos();

      void getAvailableModels(const std::string& = "bb_characters_rviz");

      void addCharacterInstance(const std::string&);
      void removeCharacterInstance(const std::string&);

    Q_SIGNALS:
      void characterInstanceAdded(const QString&);
      void characterInstanceRemoved(const QString&);

    private:
      bool parseModelInfoYamlFile(const std::string&, const std::string&, const std::string&);

      std::map<std::string, std::shared_ptr<bb_characters_rviz::CharacterVisualInfo>> character_visual_infos_;

  };
}

#endif // CHARACTERSMANAGER_H

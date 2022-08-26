#include "bb_character_helpers.h"

namespace bb_characters_rviz
{
  RosPackagePathResourceLoadingListener::RosPackagePathResourceLoadingListener(const fs::path& parentPath) : _parentPath(parentPath)
  {

  }

  /** This event is called when a resource beings loading. */
  Ogre::DataStreamPtr RosPackagePathResourceLoadingListener::resourceLoading(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource)
  {
    fs::path absolutePath = _parentPath / name;
    ROS_INFO_STREAM("RosPackagePathResourceLoadingListener loading resource: " << absolutePath.string());

    try
    {
      resource_retriever::Retriever retriever;
      _lastResource = retriever.get(absolutePath.string()); // not thread-safe!
      return Ogre::DataStreamPtr(new Ogre::MemoryDataStream(_lastResource.data.get(), _lastResource.size));
    }
    catch (resource_retriever::Exception& e)
    {
      ROS_ERROR("In RosPackagePathResourceLoadingListener: %s", e.what());
      return Ogre::DataStreamPtr();
    }
  }

  void RosPackagePathResourceLoadingListener::resourceStreamOpened(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource, Ogre::DataStreamPtr& dataStream)
  {

  }

  bool RosPackagePathResourceLoadingListener::resourceCollision(Ogre::Resource *resource, Ogre::ResourceManager *resourceManager)
  {
    return false;
  }
}

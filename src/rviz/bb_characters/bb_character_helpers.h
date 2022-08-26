#ifndef CHARACTERHELPERS_H
#define CHARACTERHELPERS_H

#include <ros/ros.h>
#include <OGRE/Ogre.h>
#include <resource_retriever/retriever.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace bb_characters_rviz
{
  /** This helper class ensures that skeletons can be loaded from a package:// path **/
  class RosPackagePathResourceLoadingListener: public Ogre::ResourceLoadingListener
  {
    public:
    RosPackagePathResourceLoadingListener(const fs::path& parentPath);

    /** This event is called when a resource beings loading. */
    virtual Ogre::DataStreamPtr resourceLoading(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource);

    virtual void resourceStreamOpened(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource, Ogre::DataStreamPtr& dataStream);

    virtual bool resourceCollision(Ogre::Resource *resource, Ogre::ResourceManager *resourceManager);

    private:
      const fs::path& _parentPath;
      resource_retriever::MemoryResource _lastResource;
  };
}


#endif // CHARACTERHELPERS_H

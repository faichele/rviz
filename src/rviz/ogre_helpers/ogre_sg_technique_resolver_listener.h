#ifndef OGRESGTECHNIQUERESOLVERLISTENER_H
#define OGRESGTECHNIQUERESOLVERLISTENER_H

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/RTShaderSystem/OgreShaderProgramManager.h>
#include <OGRE/RTShaderSystem/OgreShaderProgramWriter.h>
#include <OGRE/RTShaderSystem/OgreShaderGenerator.h>
#include <OGRE/RTShaderSystem/OgreShaderRenderState.h>
#include <OGRE/RTShaderSystem/OgreShaderFunction.h>
#include <OGRE/RTShaderSystem/OgreShaderFFPTransform.h>
#include <OGRE/RTShaderSystem/OgreShaderExIntegratedPSSM3.h>
#include <OGRE/RTShaderSystem/OgreShaderExLayeredBlending.h>
#include <OGRE/RTShaderSystem/OgreShaderExHardwareSkinning.h>

class OgreSGTechniqueResolverListener: public Ogre::MaterialManager::Listener
{
  public:
    explicit OgreSGTechniqueResolverListener(Ogre::RTShader::ShaderGenerator* pShaderGenerator);

    /** This is the hook point where shader based technique will be created.
        It will be called whenever the material manager won't find appropriate technique
        that satisfy the target scheme name. If the scheme name is out target RT Shader System
        scheme name we will try to create shader generated technique for it.
    */
    Ogre::Technique* handleSchemeNotFound(unsigned short schemeIndex,
                                          const Ogre::String& schemeName,
                                          Ogre::Material* originalMaterial, unsigned short lodIndex,
                                          const Ogre::Renderable* rend);

    bool afterIlluminationPassesCreated(Ogre::Technique* tech);

    bool beforeIlluminationPassesCleared(Ogre::Technique* tech);

  protected:
    Ogre::RTShader::ShaderGenerator* mShaderGenerator; // The shader generator instance.
};

#endif // OGRESGTECHNIQUERESOLVERLISTENER_H

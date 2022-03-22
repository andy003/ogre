//
// Created by ccros on 2022/3/20.
//

#ifndef CCROBOT_CCROBOT_MAIN_MENU_H_
#define CCROBOT_CCROBOT_MAIN_MENU_H_

#include <Ogre.h>
#include <OgreApplicationContextQt.h>
#include <OgreOverlaySystem.h>
#include <OgreCameraMan.h>

#include <ogre_helper/axes.h>
#include <ogre_helper/arrow.h>
#include <ogre_helper/grid.h>

class MyApp : public OgreBites::ApplicationContextQt, public OgreBites::InputListener
{
    Q_OBJECT

private:
    OgreBites::CameraMan* camera_man_;

public:
    MyApp();

    void createRoot() override
    {
        Ogre::String pluginsPath;

        mRoot = OGRE_NEW Ogre::Root("", "../res/ogre.cfg", "./ogre.log");
        mStaticPluginLoader.load();
        mOverlaySystem = OGRE_NEW Ogre::OverlaySystem();
    }

    void locateResources() override
    {
        auto& rgm = Ogre::ResourceGroupManager::getSingleton();
        // load resource paths from config file
        Ogre::ConfigFile cf;
        Ogre::String resourcesPath = mFSLayer->getConfigFilePath("resources.cfg");

        if (Ogre::FileSystemLayer::fileExists(resourcesPath))
        {
            Ogre::LogManager::getSingleton().logMessage("Parsing '"+resourcesPath+"'");
            cf.load(resourcesPath);
        }
        else
        {
            rgm.addResourceLocation(getDefaultMediaDir(), "FileSystem", Ogre::RGN_DEFAULT);
        }

        Ogre::String sec, type, arch;
        // go through all specified resource groups
        Ogre::ConfigFile::SettingsBySection_::const_iterator seci;
        for(seci = cf.getSettingsBySection().begin(); seci != cf.getSettingsBySection().end(); ++seci) {
            sec = seci->first;
            const Ogre::ConfigFile::SettingsMultiMap& settings = seci->second;
            Ogre::ConfigFile::SettingsMultiMap::const_iterator i;

            // go through all resource paths
            for (i = settings.begin(); i != settings.end(); i++)
            {
                type = i->first;
                arch = i->second;

                Ogre::StringUtil::trim(arch);
                if (arch.empty() || arch[0] == '.')
                {
                    // resolve relative path with regards to configfile
                    Ogre::String baseDir, filename;
                    Ogre::StringUtil::splitFilename(resourcesPath, filename, baseDir);
                    arch = baseDir + arch;
                }

                arch = Ogre::FileSystemLayer::resolveBundlePath(arch);

#if OGRE_PLATFORM != OGRE_PLATFORM_EMSCRIPTEN
                if((type == "Zip" || type == "FileSystem") && !Ogre::FileSystemLayer::fileExists(arch))
                {
                    Ogre::LogManager::getSingleton().logWarning("resource location '"+arch+"' does not exist - skipping");
                    continue;
                }
#endif
                rgm.addResourceLocation(arch, type, sec);
            }
        }

        if(rgm.getResourceLocationList(Ogre::RGN_INTERNAL).empty())
        {
            const auto& mediaDir = getDefaultMediaDir();
            // add default locations
            rgm.addResourceLocation(mediaDir + "/Main", "FileSystem", Ogre::RGN_INTERNAL);
#ifdef OGRE_BUILD_COMPONENT_TERRAIN
            rgm.addResourceLocation(mediaDir + "/Terrain", "FileSystem", Ogre::RGN_INTERNAL);
#endif
#ifdef OGRE_BUILD_COMPONENT_RTSHADERSYSTEM
            rgm.addResourceLocation(mediaDir + "/RTShaderLib/GLSL", "FileSystem", Ogre::RGN_INTERNAL);
            rgm.addResourceLocation(mediaDir + "/RTShaderLib/HLSL_Cg", "FileSystem", Ogre::RGN_INTERNAL);
#endif
        }
    }

    void setup() override
    {
        // do not forget to call the base first
        OgreBites::ApplicationContextQt::setup();

        // register for input events
        addInputListener(this);

        // get a pointer to the already created root
        Ogre::Root* root = getRoot();
        Ogre::SceneManager* scnMgr = root->createSceneManager();

        // register our scene with the RTSS
        //        Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
        //        shadergen->addSceneManager(scnMgr);

        // without light we would just get a black screen
        Ogre::Light* light = scnMgr->createLight("MainLight");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(0, 10, 15);
        lightNode->attachObject(light);

        // also need to tell where we are
        Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        camNode->setPosition(0, 0, 15);
        camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

        // create the camera
        Ogre::Camera* cam = scnMgr->createCamera("myCam");
        cam->setNearClipDistance(5);    // specific to this sample
        cam->setAutoAspectRatio(true);
        camNode->attachObject(cam);
        camera_man_ = new OgreBites::CameraMan(camNode);
        camera_man_->setStyle(OgreBites::CS_ORBIT);

        //        Axes* axes = new Axes(scnMgr, scnMgr->getRootSceneNode()->createChildSceneNode());

        auto* arrow = new Arrow(scnMgr, scnMgr->getRootSceneNode()->createChildSceneNode());
        arrow->setColor(Ogre::ColourValue(1.0, 0.0, 0.0, 1.0));

        auto* grid = new Grid(scnMgr, scnMgr->getRootSceneNode()->createChildSceneNode(), Grid::Lines, 30, 1, 1,
                              Ogre::ColourValue(1.0, 1.0, 0.0, 1.0));

        // and tell it to render into the main window
        Ogre::Viewport* vp = getRenderWindow()->addViewport(cam);
        vp->setBackgroundColour(Ogre::ColourValue(0.12, 0.12, 0.12, 1.0));
    }

    bool mouseMoved(const OgreBites::MouseMotionEvent& evt) override
    {
        return camera_man_->mouseMoved(evt);
    }

    bool mouseWheelRolled(const OgreBites::MouseWheelEvent& evt) override
    {
        return camera_man_->mouseWheelRolled(evt);
    }

    bool mousePressed(const OgreBites::MouseButtonEvent& evt) override
    {
        return camera_man_->mousePressed(evt);
    }

    bool mouseReleased(const OgreBites::MouseButtonEvent& evt) override
    {
        return camera_man_->mouseReleased(evt);
    }
};

#endif    // CCROBOT_CCROBOT_MAIN_MENU_H_

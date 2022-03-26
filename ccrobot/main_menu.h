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

        mRoot = OGRE_NEW Ogre::Root("", "./ogre.cfg", "./ogre.log");
        mStaticPluginLoader.load();
        mOverlaySystem = OGRE_NEW Ogre::OverlaySystem();
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
//        camNode.

        // create the camera
        Ogre::Camera* cam = scnMgr->createCamera("myCam");
        cam->setNearClipDistance(0.1); // specific to this sample
        cam->setAutoAspectRatio(true);
        camNode->attachObject(cam);
        camera_man_ = new OgreBites::CameraMan(camNode);
        camera_man_->setStyle(OgreBites::CS_ORBIT);

        Axes* axes = new Axes(scnMgr, scnMgr->getRootSceneNode()->createChildSceneNode());

        // and tell it to render into the main window
        Ogre::Viewport* vp = getRenderWindow()->addViewport(cam);
        vp->setBackgroundColour(Ogre::ColourValue(1.0, 0.12, 0.12, 1.0));


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

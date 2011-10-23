/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

#include <OGRE/Ogre.h>
#include <QGLWidget>
#include <QX11Info>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

class Scene_Manager : public Ogre::FrameListener, public Ogre::WindowEventListener, public QGLWidget, public OIS::KeyListener, public OIS::MouseListener
{
public:
    Scene_Manager( QWidget *parent=0 ):
        QGLWidget( parent ),
        mRoot(0),
        mCamera(0),
        mSceneMgr(0),
        mWindow(0),
        mResourcesCfg(Ogre::StringUtil::BLANK),
        mPluginsCfg(Ogre::StringUtil::BLANK),
        mCursorWasVisible(false),
        mShutDown(false)
    {
        init( "/home/jmecosta/.ogre/Cthugha/plugins.cfg", "/home/jmecosta/.ogre/Cthugha/ogre.cfg", "/home/jmecosta/.ogre/Cthugha/ogre.log" );
    }
    virtual ~Scene_Manager()
    {
        //Remove ourself as a Window listener
        Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
        windowClosed(mWindow);
        delete mRoot;

    }

protected:
    // qglwidget re-implemented methods
    virtual void initializeGL();
    virtual void resizeGL( int, int );
    virtual void paintGL();

    void init( std::string, std::string, std::string );

    void initializeOgre();
    void glDraw();
    void timerEvent(QTimerEvent *);
    void resizeEvent(QResizeEvent *e);


    bool initialised;

    virtual bool setup();
    virtual bool configure(void);
    virtual void chooseSceneManager(void);
    virtual void createCamera(void);
    virtual void createScene(void);
    virtual void destroyScene(void);
    virtual void createViewports(void);
    virtual void setupResources(void);
    virtual void createResourceListener(void);
    virtual void loadResources(void);
    virtual void createFrameListener(void);

    Ogre::Root *mRoot;
    Ogre::Camera* mCamera;
    Ogre::SceneManager* mSceneMgr;
    Ogre::RenderWindow* mWindow;
    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;
    Ogre::AnimationState* mAnimState;

    std::string ogre_cfg_file;
    std::string ogre_log;

    // OgreBites
    bool mCursorWasVisible;                    // was cursor visible before dialog appeared
    bool mShutDown;

    //OIS Input devices
    OIS::InputManager* mInputManager;
    OIS::Mouse*    mMouse;
    OIS::Keyboard* mKeyboard;


    // Ogre::FrameListener
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    // OIS::KeyListener
    virtual bool keyPressed( const OIS::KeyEvent &arg );
    virtual bool keyReleased( const OIS::KeyEvent &arg );
    // OIS::MouseListener
    virtual bool mouseMoved( const OIS::MouseEvent &arg );
    virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    virtual bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

    // Ogre::WindowEventListener
    //Adjust mouse clipping area
    virtual void windowResized(Ogre::RenderWindow* rw);
    //Unattach OIS before window shutdown (very important under Linux)
    virtual void windowClosed(Ogre::RenderWindow* rw);

};


#endif // SCENE_MANAGER_H

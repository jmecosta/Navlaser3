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

//
#include "scene_manager.h"
using namespace std;
#define THIS Scene_Manager

//-------------------------------------------------------------------------------------

void THIS::init( std::string plugins_file,
                 std::string ogre_cfg_file,
                 std::string ogre_log )
{
    ogre_cfg_file =ogre_cfg_file;
    ogre_log=ogre_log;

    // setup window
#ifdef _DEBUG
    mResourcesCfg = "/home/jmecosta/.ogre/Cthugha/resources_d.cfg";
    mPluginsCfg = "/home/jmecosta/.ogre/Cthugha/plugins_d.cfg";
#else
    mResourcesCfg = "/home/jmecosta/.ogre/Cthugha/resources.cfg";
    mPluginsCfg = "/home/jmecosta/.ogre/Cthugha/plugins.cfg";
#endif

    initialised = false;

    cout << "<TRACE><LOG><SceneManager><init> Constructor" << endl;
    if (!setup())
        return;



}

//-------------------------------------------------------------------------------------
bool THIS::configure(void)
{

    cout << "<TRACE><LOG><SceneManager><configure> Start" << endl;
    // Show the configuration dialog and initialise the system
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg
    //mRoot->showConfigDialog();
    //{
    // If returned true, user clicked OK so initialise
    // Here we choose to let the system create a default rendering window by passing 'true'
    //     mWindow = mRoot->initialise(true, "TutorialApplication Render Window");

    //return true;
    //}
    //else
    //{
    //    return false;
    //}

    // setup a renderer
    Ogre::RenderSystemList::const_iterator renderers = mRoot->getAvailableRenderers().begin();
    while(renderers != mRoot->getAvailableRenderers().end())
    {
        Ogre::String rName = (*renderers)->getName();
        if (rName == "OpenGL Rendering Subsystem")
            break;
        renderers++;
    }

    Ogre::RenderSystem *renderSystem = *renderers;

    mRoot->setRenderSystem( renderSystem );
    QString dimensions = QString( "%1x%2" )
            .arg(this->width())
            .arg(this->height());

    renderSystem->setConfigOption( "Video Mode", dimensions.toStdString() );

    // initialize without creating window
    mRoot->getRenderSystem()->setConfigOption( "Full Screen", "No" );
    mRoot->saveConfig();

    cout << "<TRACE><LOG><SceneManager><configure> initialize" << endl;
    mRoot->initialise(false); // don't create a window

}
void THIS::createFrameListener(void)
{
    Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;

    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

    mInputManager = OIS::InputManager::createInputSystem( pl );

    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));

    mMouse->setEventCallback(this);
    mKeyboard->setEventCallback(this);

    //Set initial mouse clipping size
    windowResized(mWindow);

    //Register as a Window listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    // create a params panel for displaying sample details
    Ogre::StringVector items;
    items.push_back("cam.pX");
    items.push_back("cam.pY");
    items.push_back("cam.pZ");
    items.push_back("");
    items.push_back("cam.oW");
    items.push_back("cam.oX");
    items.push_back("cam.oY");
    items.push_back("cam.oZ");
    items.push_back("");
    items.push_back("Filtering");
    items.push_back("Poly Mode");

    mRoot->addFrameListener(this);

    Ogre::LogManager::getSingletonPtr()->logMessage("*** INIT OIS END ***");
}
//-------------------------------------------------------------------------------------
void THIS::chooseSceneManager(void)
{

    cout << "<TRACE><LOG><SceneManager><chooseSceneManager>  " << endl;
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void THIS::createCamera(void)
{
    cout << "<TRACE><LOG><SceneManager><createCamera>  " << endl;
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

    // Position it at 500 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,0,80));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(5);
}
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
void THIS::destroyScene(void)
{
}
//-------------------------------------------------------------------------------------
void THIS::createViewports(void)
{
    cout << "<TRACE><LOG><SceneManager><createViewports>  " << endl;
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(
                Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void THIS::setupResources(void)
{
    cout << "<TRACE><LOG><SceneManager><setupResources>  " << endl;

    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                        archName, typeName, secName);
        }
    }
}
//-------------------------------------------------------------------------------------
void THIS::createResourceListener(void)
{

}
//-------------------------------------------------------------------------------------
void THIS::loadResources(void)
{
    cout << "<TRACE><LOG><SceneManager><loadResources>  " << endl;
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//-------------------------------------------------------------------------------------
bool THIS::setup(void)
{
    cout << "<TRACE><LOG><SceneManager><setup>  " << endl;
    mRoot = new Ogre::Root(mPluginsCfg, ogre_cfg_file, ogre_log);

    cout << "<TRACE><LOG><SceneManager><setup>  Setup Resources" << endl;
    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    return true;
}
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

//Unattach OIS before window shutdown (very important under Linux)

void THIS::createScene(void)
{
    cout << "<TRACE><LOG><SceneManager><createScene>  " << endl;
    Ogre::Entity* ogreHead = mSceneMgr->createEntity("Head", "ogrehead.mesh");

    Ogre::SceneNode* headNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    headNode->attachObject(ogreHead);

    // Set ambient light
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

    // Create a light
    Ogre::Light* l = mSceneMgr->createLight("MainLight");
    l->setPosition(20,80,50);
}


void THIS::initializeGL()
{
}
void THIS::initializeOgre()
{
    cout << "<TRACE><LOG><SceneManager><initializeOgre>  " << endl;
    initialised = true;
    setFormat(QGLFormat(QGL::DoubleBuffer | QGL::SampleBuffers));
    setAttribute(Qt::WA_OpaquePaintEvent);

#if PARENT_HANDLE
    setAutoBufferSwap(false);
#endif
    Ogre::NameValuePairList params;
#if PARENT_HANDLE
    params["parentWindowHandle"] = Ogre::StringConverter::toString((unsigned long)(effectiveWinId()));
#else
    makeCurrent();
    params["currentGLContext"] = "true";
#endif

    mWindow = mRoot->createRenderWindow( "QOgreWidget_RenderWindow",
                                         this->width(),
                                         this->height(),
                                         false,
                                         &params );

    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    mWindow->setVisible(true);
    mWindow->setActive(true);
    mWindow->setAutoUpdated(true);
    resize(width(), height());


    setAttribute(Qt::WA_OpaquePaintEvent);

    chooseSceneManager();
    createCamera();
    createViewports();


    Ogre::Entity *ent = mSceneMgr->createEntity("Robot", "robot.mesh");
    Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode("RobotNode");
    node->attachObject(ent);


    createScene ();

    mAnimState = ent->getAnimationState("Walk");
    mAnimState->setEnabled(true);
    mAnimState->setLoop(true);

    createFrameListener();

    startTimer(5);

}


/**
 * @brief render a frame
 * @author Kito Berg-Taylor
 */
void THIS::paintGL()
{
    if (!initialised)
        initializeOgre();

    for (Ogre::SceneManager::MovableObjectIterator mit = mSceneMgr->getMovableObjectIterator("Entity");mit.hasMoreElements(); mit.moveNext() )
    {
        Ogre::Entity *entity = static_cast<Ogre::Entity*>(mit.peekNextValue());
        if (entity->hasSkeleton() )
        {
            for (Ogre::AnimationStateIterator animIt = entity->getAllAnimationStates()->getAnimationStateIterator(); animIt.hasMoreElements(); animIt.moveNext() )
            {
                Ogre::AnimationState *animState = animIt.peekNextValue();
                if ( animState->getEnabled() )
                {
                    animState->addTime(mWindow->getBestFPS()/10000);
                }
            }
        }
    }

    //Ogre::WindowEventUtilities::messagePump();
    mRoot->renderOneFrame();


}

/**
 * @brief resize the GL window
 * @author Kito Berg-Taylor
 */
void THIS::resizeGL( int width, int height )
{
    if ( !mWindow ) return;

    mWindow->resize(width, height);
    mWindow->windowMovedOrResized();
    for(int i = 0; i < mWindow->getNumViewports(); ++i)
    {
        Ogre::Viewport* pViewport = mWindow->getViewport(i);
        Ogre::Camera* pCamera = pViewport->getCamera();
        pCamera->setAspectRatio(static_cast<Ogre::Real>(pViewport->getActualWidth()) / static_cast<Ogre::Real>(pViewport->getActualHeight()));
        pViewport->_updateDimensions();
    }
    paintGL();
}

void THIS::timerEvent(QTimerEvent *evt)
{
    Q_UNUSED(evt);
    glDraw();
}

void THIS::glDraw()
{
#if !PARENT_HANDLE
    QGLWidget::glDraw();
    return;
#endif
    if (!mWindow) {
        QGLWidget::glDraw();
        return;
    }
    if (this->autoBufferSwap())
        QGLWidget::glDraw();
    else
    {
        paintGL();
    }

}


void THIS::resizeEvent(QResizeEvent *e)
{
#if !PARENT_HANDLE
    QGLWidget::resizeEvent(e);
    return;
#endif
    if (!mWindow) {
        QGLWidget::resizeEvent(e);
        return;
    }
    resizeGL(width(), height());
}
bool THIS::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

    return true;
}
//-------------------------------------------------------------------------------------
bool THIS::keyPressed( const OIS::KeyEvent &arg )
{
    Ogre::String newVal;
    Ogre::TextureFilterOptions tfo;
    unsigned int aniso;
    cout << "<TRACE><LOG><SceneManager><keyPressed>  " << endl;
    if (arg.key == OIS::KC_F)   // toggle visibility of advanced frame stats
    {
        newVal = "Trilinear";
        tfo = Ogre::TFO_TRILINEAR;
        aniso = 1;
        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
    }
    else if (arg.key == OIS::KC_G)   // toggle visibility of even rarer debugging details
    {
        newVal = "Anisotropic";
        tfo = Ogre::TFO_ANISOTROPIC;
        aniso = 8;
        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
    }
    else if (arg.key == OIS::KC_T)   // cycle polygon rendering mode
    {


            newVal = "None";
            tfo = Ogre::TFO_NONE;
            aniso = 1;

        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
    }
    else if (arg.key == OIS::KC_R)   // cycle polygon rendering mode
    {
        Ogre::String newVal;
        Ogre::PolygonMode pm;

        switch (mCamera->getPolygonMode())
        {
        case Ogre::PM_SOLID:
            newVal = "Wireframe";
            pm = Ogre::PM_WIREFRAME;
            break;
        case Ogre::PM_WIREFRAME:
            newVal = "Points";
            pm = Ogre::PM_POINTS;
            break;
        default:
            newVal = "Solid";
            pm = Ogre::PM_SOLID;
        }

        mCamera->setPolygonMode(pm);
    }
    else if(arg.key == OIS::KC_F5)   // refresh all textures
    {
        Ogre::TextureManager::getSingleton().reloadAll();
    }
    else if (arg.key == OIS::KC_SYSRQ)   // take a screenshot
    {
        mWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
    }
    else if (arg.key == OIS::KC_ESCAPE)
    {
        mShutDown = true;
    }


    return true;
}

bool THIS::keyReleased( const OIS::KeyEvent &arg )
{

    return true;
}

bool THIS::mouseMoved( const OIS::MouseEvent &arg )
{

    return true;
}

bool THIS::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
        Ogre::LogManager::getSingletonPtr()->logMessage("*** Mouse Pressed ***");
    return true;
}

bool THIS::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    return true;
}

//Adjust mouse clipping area
void THIS::windowResized(Ogre::RenderWindow* rw)
{
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

//Unattach OIS before window shutdown (very important under Linux)
void THIS::windowClosed(Ogre::RenderWindow* rw)
{
    //Only close for window that created OIS (the main window in these demos)
    if( rw == mWindow )
    {
        if( mInputManager )
        {
            mInputManager->destroyInputObject( mMouse );
            mInputManager->destroyInputObject( mKeyboard );

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}

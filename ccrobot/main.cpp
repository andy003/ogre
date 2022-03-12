/**
 ***********************************************************************************************************************
 *
 * @author  ZhangRan
 * @version 1.0.0
 *
 * <h2><center>&copy; COPYRIGHT 2022 </center></h2>
 *
 ***********************************************************************************************************************
 */

#include <Ogre.h>
#include <OgreApplicationContextQt.h>

#include <QApplication>

class MyApp : public OgreBites::ApplicationContextQt
{

};

int main(int argc, char* argv[])
{
    MyApp my_app;

    my_app.initApp();
    my_app.startTimer(40);

    QApplication app(argc, argv);

    return QApplication::exec();
}
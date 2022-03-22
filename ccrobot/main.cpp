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

#include <cstdio>
#include "main_menu.h"

#include <QApplication>


int main(int argc, char* argv[])
{
    try
    {
        QApplication app(argc, argv);

        MyApp my_app;
        my_app.initApp();
        my_app.startTimer(40);
        return QApplication::exec();
    }
    catch (const Ogre::Exception& err)
    {
        printf("err------------------- %s\n", err.what());
    }

    return -1;
}
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

#include <urdf/model.h>

int main(int argc, char* argv[])
{
    try
    {
        urdf::Model model;

        if (!model.initFile(R"(..\res\robot_description\urdf\cr5_robot.urdf)"))
        {
            printf("urdf parse failed\n");
            return -1;
        }

        std::vector<urdf::LinkSharedPtr> links;
        model.getLinks(links);
        for (const auto& iter : links)
        {
            printf("links name : %s\n", iter->name.c_str());
            if (iter->visual && iter->visual->geometry)
            {
                if (iter->visual->geometry->type == urdf::Geometry::MESH)
                {
                    urdf::MeshSharedPtr mesh = std::static_pointer_cast<urdf::Mesh>(iter->visual->geometry);
                    printf("mesh file : %s\n", mesh->filename.c_str());
                }
            }
        }

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
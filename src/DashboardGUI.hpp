#include "imgui.h"
#include "implot.h"
#include "implot3d.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include "DashboardController.hpp"
#include "StructComponents.hpp"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers


class DashboardGUI{
    public:
    DashboardGUI(std::shared_ptr<DashboardController> givenDashboardPointer, std::shared_ptr<StructofComponents> givenComponentStruct) : DashboardPointer(givenDashboardPointer), ComponentStructPointer(givenComponentStruct)  {
        DashboardControllerThread = std::jthread(&DashboardController::Controller, DashboardPointer.get());
        std::cout << "GUI construction successful! Program starting up..." << std::endl;
    };
    int Startup();
    private:
         std::shared_ptr<DashboardController> DashboardPointer;
         std::shared_ptr<StructofComponents> ComponentStructPointer;
         std::jthread DashboardControllerThread;
};

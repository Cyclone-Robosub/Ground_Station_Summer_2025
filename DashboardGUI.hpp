#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include "DashboardController.hpp"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
class DashboardGUI{
    public:
    DashboardGUI(std::shared_ptr<DashboardController> givenDashboardPointer) : DashboardPointer(givenDashboardPointer) {
        DashboardControllerThread = std::jthread(&DashboardController::Controller, DashboardPointer.get());
        std::cout << "confirm got out of GUI constructor" << std::endl;
    };
    int Startup();
    private:
         std::shared_ptr<DashboardController> DashboardPointer;
         std::jthread DashboardControllerThread;
};
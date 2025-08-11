// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp
#include "DashboardGUI.hpp"
#include "components/StatusIndicators.hpp"
#include "components/BatteryMonitor.hpp"
#include "components/MessageLogger.hpp"
#include "components/ConfigurationPanel.hpp"
#include "components/RobotPlotting.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <ctime>
// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// This example can also compile and run with Emscripten! See 'Makefile.emscripten' for details.
#ifdef __EMSCRIPTEN__
#include "../libs/emscripten/emscripten_mainloop_stub.h"
#endif

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Main code
int DashboardGUI::Startup()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100 (WebGL 1.0)
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(IMGUI_IMPL_OPENGL_ES3)
    // GL ES 3.0 + GLSL 300 es (WebGL 2.0)
    const char* glsl_version = "#version 300 es";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor()); // Valid on GLFW 3.3+ only
    GLFWwindow* window = glfwCreateWindow((int)(1280 * main_scale), (int)(800 * main_scale), "Dear ImGui GLFW+OpenGL3 example", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot3D::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
#ifdef __EMSCRIPTEN__
    ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
#endif
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    // - Our Emscripten build process allows embedding fonts to be accessible at runtime from the "fonts/" folder. See Makefile.emscripten for details.
    //style.FontSizeBase = 20.0f;
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf");
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf");
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf");
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf");
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf");
    //IM_ASSERT(font != nullptr);

    // Our state
    bool show_demo_window = false;
    bool native_sample_data = true;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
#ifdef __EMSCRIPTEN__
    // For an Emscripten build we are disabling file-system access, so let's not attempt to do a fopen() of the imgui.ini file.
    // You may manually call LoadIniSettingsFromMemory() to load settings from your own storage.
    io.IniFilename = nullptr;
    EMSCRIPTEN_MAINLOOP_BEGIN
#else
    while (!glfwWindowShouldClose(window))
#endif
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }
        
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        static MessageLogger generalLogger;
        
        // Dynamic test messages
        static std::vector<std::string> manipulation_messages = {
            "Manipulator initialized",
            "Manipulator ready",
            "Manipulator error: joint overload",
            "Manipulator recovered"
        };
        static std::vector<std::string> vision_messages = {
            "Vision system online",
            "Vision system detected object",
            "Vision system lost tracking",
            "Vision system reacquired target"
        };
        static size_t manip_idx = 0;
        static size_t vision_idx = 0;
        static auto last_update = std::chrono::steady_clock::now();
        
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_update).count() >= 2) {
            generalLogger.AddSystemMessage("Manipulation", manipulation_messages[manip_idx]);
            generalLogger.AddSystemMessage("Vision", vision_messages[vision_idx]);
            manip_idx = (manip_idx + 1) % manipulation_messages.size();
            vision_idx = (vision_idx + 1) % vision_messages.size();
            last_update = now;
        }

        generalLogger.Render();

        MessageLogger robotController("Robot Controller"); 
        robotController.AddSystemMessage("PWM", "PWM controller initialized");
        robotController.AddSystemMessage("Waypoint", "Waypoint received: [1.0, 2.0, 3.0]");
        robotController.Render();

        static LimitedTrajectory robot_trajectory(5000);
        float coordinates[3];
        for (int i = 0; i < 13; i++) {
            coordinates[0] = (float)ImGui::GetTime() / 10;
            coordinates[1] = (float)ImGui::GetTime() / 10;
            coordinates[2] = (float)ImGui::GetTime() / 10;
            robot_trajectory.addPoint(coordinates); // Example point
        }
        robot_trajectory.plot();

        DemoLinePlots();

        // Sample data for status indicators
        static std::vector<std::string> master_statuses = {
            "DANGER", "warning", "success", "standby"
        };
        static std::vector<SystemStatus> system_statuses_set[] = {
            {
                {"State Saver", "success", "System 1 is connected"},
                {"Robot Operations", "warning", "System 2 has low battery"},
                {"Hardwar Status", "error", "System 3 failed to start"}
            },
            {
                {"State Saver", "warning", "System 1 voltage low"},
                {"Robot Operations", "success", "System 2 is connected"},
                {"success", "System 3 is running"}
            },
            {
                {"State Saver", "error", "System 1 failed"},
                {"Robot Operations", "standby", "System 2 offline"},
                {"Hardwar Status", "success", "System 3 is connected"}
            },
            {
                {"State Saver", "error", "System 1 failed"},
                {"Robot Operations", "standby", "System 2 offline"},
                {"Hardwar Status", "success", "System 3 is connected"}
            }
        };

        static size_t status_idx = 0;
        static auto last_status_update = std::chrono::steady_clock::now();

        auto now_status = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now_status - last_status_update).count() >= 2) {
            status_idx = (status_idx + 1) % 4;
            last_status_update = now_status;
        }
        std::unique_lock<std::mutex> lk(ComponentStructPointer->SystemStatusmutex);
        std::string master_status = ComponentStructPointer->SystemStatusData[0].status;
       // std::vector<SystemStatus> system_statuses = system_statuses_set[status_idx];
        
        RenderStatusIndicators(master_status, ComponentStructPointer->SystemStatusData);
        lk.unlock();
        // Battery voltage test data
        static std::vector<float> battery_voltages = {12.0f, 14.5f, 10.8f, 16.0f, 11.2f, 13.3f};
        static size_t battery_idx = 0;
        static auto last_battery_update = std::chrono::steady_clock::now();

        // For battery voltage
        auto now_battery = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now_battery - last_battery_update).count() >= 50) {
            battery_idx = (battery_idx + 1) % battery_voltages.size();
            last_battery_update = now_battery;
        }
        static float battery_voltage = 12.0f; // Example initial value
        battery_voltage = battery_voltages[battery_idx];

        static float battery_threshold = 12.0f; // Example threshold value
        RenderBatteryMonitor(ComponentStructPointer->BatteryData);
        
        RenderConfigurationPanel(io, show_demo_window);

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot3D::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    DashboardPointer->Shutdown();
    return 0;
}

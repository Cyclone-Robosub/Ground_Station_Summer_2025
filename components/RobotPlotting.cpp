#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <tuple>
#include "RobotPlotting.hpp"
#include "implot3d.h"
#include "imgui.h" // Assuming ImGui headers are available.

// Define a struct to hold the coordinate data.
struct CoordinateData {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

/**
 * @brief Reads coordinates from a file, skipping a specified number of lines.
 *
 * This function opens a text file, reads each line, and parses three
 * floating-point numbers from lines that are multiples of `skipLines`.
 * The values are stored in vectors and returned in a struct.
 *
 * @param filename The path to the input file.
 * @param skipLines The number of lines to skip between each read.
 * @return A CoordinateData struct containing the x, y, and z vectors.
 */
CoordinateData readCoordinatesFromFile(const std::string& filename, int skipLines) {
    // Open the file for reading.
    std::ifstream inputFile(filename);

    // Error handling: Check if the file was opened successfully.
    if (!inputFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return {}; // Return an empty struct.
    }

    // Initialize an empty struct to store the parsed data.
    CoordinateData data;
    std::string line;
    int lineNumber = 0;

    // Loop through the file, reading one line at a time.
    while (std::getline(inputFile, line)) {
        // Increment the line number counter.
        lineNumber++;

        // Check if the current line number is a multiple of (skipLines + 1).
        // Using `lineNumber % (skipLines + 1) == 1` correctly reads the 1st line, then the (skipLines+1)th, and so on.
        // If skipLines is 0, we read every line.
        if (skipLines == 0 || (lineNumber - 1) % (skipLines + 1) == 0) {
            // Use a stringstream to parse the three double values from the line.
            std::stringstream ss(line);
            double x_val, y_val, z_val;
            
            // Check if parsing was successful before pushing to the vectors.
            if (ss >> x_val >> y_val >> z_val) {
                data.x.push_back(x_val);
                data.y.push_back(y_val);
                data.z.push_back(z_val);
            }
        }
    }

    // Close the file.
    inputFile.close();
    
    // Return the struct with the populated data.
    return data;
}

// Helper function to create a dummy file for demonstration.
void createDummyFile(const std::string& filename) {
    std::ofstream dummyFile(filename);
    if (dummyFile.is_open()) {
        dummyFile << "0.0000000e+00 0.0000000e+00 0.0000000e+00\n";
        dummyFile << "3.2136061e-04 3.2135937e-04 5.0211206e-04\n";
        dummyFile << "1.2849982e-03 1.2849783e-03 2.0085007e-03\n";
        dummyFile << "2.8895836e-03 2.8894855e-03 4.5193060e-03\n";
        dummyFile << "5.1329071e-03 5.1326215e-03 8.0353615e-03\n";
        dummyFile << "8.0118807e-03 8.0112853e-03 1.2557838e-02\n";
        dummyFile << "1.1522549e-02 1.1521585e-02 1.8087609e-02\n";
        dummyFile << "1.5660107e-02 1.5658894e-02 2.4625443e-02\n";
        dummyFile << "2.0424034e-02 2.0412796e-02 3.2170926e-02\n";
        dummyFile << "2.5813348e-02 2.5771872e-02 4.0723273e-02\n";
        dummyFile << "3.1821305e-02 3.1730334e-02 5.0219706e-02\n";
        dummyFile << "3.8440149e-02 3.8283581e-02 6.0543738e-02\n";
        dummyFile << "4.5661038e-02 4.5427951e-02 7.1565668e-02\n";
        dummyFile << "5.3442331e-02 5.3129090e-02 8.3136931e-02\n";
        dummyFile << "6.1681458e-02 6.1300531e-02 9.5121607e-02\n";

        // Add every 20th line from the uploaded positions.txt file.
        dummyFile << "7.9106404e-02 7.8675790e-02 1.1987359e-01\n";
        dummyFile << "2.0296489e-01 2.0656699e-01 2.8506315e-01\n";
        dummyFile << "3.7696433e-01 3.9690836e-01 4.9223623e-01\n";
        dummyFile << "5.6291465e-01 5.9242267e-01 6.6890663e-01\n";
        dummyFile << "7.1573145e-01 6.4649610e-01 7.1322073e-01\n";
        dummyFile << "8.1409384e-01 7.0784402e-01 7.5050519e-01\n";
        dummyFile << "8.8058498e-01 7.6432657e-01 7.8188164e-01\n";
        dummyFile << "9.3101683e-01 8.2117183e-01 8.1691238e-01\n";
        dummyFile << "9.7027581e-01 8.7909012e-01 8.6186835e-01\n";
        dummyFile << "9.9483324e-01 9.3807530e-01 9.1764835e-01\n";
        dummyFile << "9.9835619e-01 9.7457792e-01 1.0412355e+00\n";
        dummyFile << "9.9834849e-01 9.7500110e-01 1.0415163e+00\n";
        dummyFile << "9.9832791e-01 9.7605910e-01 1.0422237e+00\n";
        dummyFile << "9.9831411e-01 9.7711719e-01 1.0429402e+00\n";
        dummyFile << "9.9829988e-01 9.7818459e-01 1.0436661e+00\n";
        dummyFile << "9.9828468e-01 9.7924769e-01 1.0443997e+00\n";
        dummyFile << "9.9826847e-01 9.8031268e-01 1.0451408e+00\n";
        dummyFile << "9.9825121e-01 9.8137578e-01 1.0458896e+00\n";
        dummyFile << "9.9823307e-01 9.8243688e-01 1.0466459e+00\n";
        dummyFile << "9.9821382e-01 9.8349589e-01 1.0474098e+00\n";
        dummyFile << "9.9819349e-01 9.8455110e-01 1.0481813e+00\n";
        dummyFile << "9.9817208e-01 9.8560330e-01 1.0489599e+00\n";
        dummyFile << "9.9815011e-01 9.8665040e-01 1.0497453e+00\n";
        dummyFile << "9.9812739e-01 9.8769360e-01 1.0505377e+00\n";
        dummyFile << "9.9810398e-01 9.8873090e-01 1.0513371e+00\n";
        dummyFile << "9.9807987e-01 9.8976191e-01 1.0521431e+00\n";
        dummyFile << "9.9805500e-01 9.9078652e-01 1.0529558e+00\n";
        dummyFile << "9.9802930e-01 9.9180473e-01 1.0537750e+00\n";
        dummyFile << "9.9800271e-01 9.9281654e-01 1.0546006e+00\n";
        dummyFile << "9.9797521e-01 9.9382185e-01 1.0554326e+00\n";
        dummyFile << "9.9794676e-01 9.9482066e-01 1.0562709e+00\n";
        dummyFile << "9.9791739e-01 9.9581337e-01 1.0571153e+00\n";
        dummyFile << "9.9788705e-01 9.9679958e-01 1.0579659e+00\n";
        dummyFile.close();
    }
}

void DemoLinePlots() {
    // Static variables ensure the data is only loaded once and persists between frames.
    static bool dataLoaded = false;
    static CoordinateData myData;
    static double xs[1000], ys[1000], zs[1000];
    static size_t dataSize = 0;

    // Load data from the file only on the first run.
    if (!dataLoaded) {
        const std::string filename = "data.txt";
        createDummyFile(filename); // Create the dummy file for demonstration.
        
        int skipLines = 1; // Read every 2nd line.
        myData = readCoordinatesFromFile(filename, skipLines);
        
        if (!myData.x.empty()) {
            dataSize = myData.x.size();
            for (size_t i = 0; i < dataSize; ++i) {
                xs[i] = myData.x[i];
                ys[i] = myData.y[i];
                zs[i] = myData.z[i];
            }
            dataLoaded = true;
            std::cout << "Successfully read " << dataSize << " data points from file." << std::endl;
        } else {
            std::cerr << "Failed to load data from file." << std::endl;
        }
    }

    static float xs1[1001], ys1[1001], zs1[1001];
    for (int i = 0; i < 1001; i++) {
        xs1[i] = i * 0.001f;
        ys1[i] = 0.5f + 0.5f * cosf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
        zs1[i] = 0.5f + 0.5f * sinf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
    }
    static double xs2[20], ys2[20], zs2[20];
    for (int i = 0; i < 20; i++) {
        xs2[i] = i * 1 / 19.0f;
        ys2[i] = xs2[i] * xs2[i];
        zs2[i] = xs2[i] * ys2[i];
    }
    
    if (ImPlot3D::BeginPlot("Line Plots")) {
        ImPlot3D::SetupAxes("x", "y", "z");

        // Original demo plots
        // ImPlot3D::PlotLine("f(x)", xs1, ys1, zs1, 1001);
        // ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Circle);
        // ImPlot3D::PlotLine("g(x)", xs2, ys2, zs2, 20, ImPlot3DLineFlags_Segments);

        // Plot the data read from the file.
        if (dataLoaded) {
            ImPlot3D::PlotLine("File Data", xs, ys, zs, dataSize, ImPlot3DLineFlags_None);
        }

        ImPlot3D::EndPlot();
    }
}

#include <ctime>
#include <chrono>
#include <iostream>
#include "Dashboard.h"
void Startup() {
    Dashboard getDashboard = Dashboard();
    std::cout << std::endl;
    auto now = std::chrono::system_clock::now();
    std::cout << now;
}


int main(){
    Startup();
}

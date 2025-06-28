#include <chrono>
#include <iostream>
#include <thread>
class Dashboard{
    public:
    Dashboard(){
        std::cout << startuptime << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << std::chrono::system_clock::now();
    }
    private:
        const std::chrono::system_clock::time_point startuptime = std::chrono::system_clock::now();

};
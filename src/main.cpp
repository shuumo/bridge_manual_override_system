#include "app.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "main has been run." << "\n";
    App application;
    std::cout << "good shit, the app is created and didnt break on launch" << "\n";
    return application.run(argc, argv);
}


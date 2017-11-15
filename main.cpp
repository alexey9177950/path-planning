#include <iostream>
#include "tinyxml2.h"

int main() {
    tinyxml2::XMLDocument doc;
    doc.LoadFile("example.xml");
    std::cout << "Hello World!" << std::endl;
    return 0;
}

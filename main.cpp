<<<<<<< HEAD
#include <iostream>
#include "tinyxml2.h"

int main() {
    tinyxml2::XMLDocument doc;
    doc.LoadFile("example.xml");
    std::cout << "Hello World!" << std::endl;
    return 0;
}
=======
#include <QCoreApplication>
#include "tinyxml2.h"

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);
    tinyxml2::XMLDocument doc;
    doc.LoadFile( "example.xml" );
    return a.exec();
}

>>>>>>> 8fdf8f36b1e61bbfe9d74aea1128dfa52df3b6a2

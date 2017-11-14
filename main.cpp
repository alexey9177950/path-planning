#include <QCoreApplication>
#include "tinyxml2.h"

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);
    tinyxml2::XMLDocument doc;
    doc.LoadFile( "example.xml" );
    return a.exec();
}


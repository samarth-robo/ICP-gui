#include <QApplication>
#include "pclviewer.h"

#include <iostream>

int main(int argc, char **argv) {
    QApplication a(argc, argv);

    PCLViewer w;
    w.show();

    return a.exec();
}

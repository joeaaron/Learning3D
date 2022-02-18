#include "mainwindow.h"
#include <QApplication>

#include <vtkOutputWindow.h>
int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);	// ²»µ¯³övtkOutputWindow´°¿Ú
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

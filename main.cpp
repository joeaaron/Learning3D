#include "mainwindow.h"
#include <QApplication>

#include <vtkOutputWindow.h>
int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);	// ²»µ¯³övtkOutputWindow´°¿Ú

    QApplication app(argc, argv);
	app.setOrganizationName("Ride the Wave");
	app.setApplicationName("Vision 3D");
	//set app style by qss file
	QFile fstyle(":/resource/qss/Darcula.qss"); //0-Darcula.qss 1-Windows.qss
	if (!fstyle.exists())
		qDebug("Unable to set stylesheet, file not found\n");
	else {
		fstyle.open(QFile::ReadOnly | QFile::Text);
		QTextStream ts(&fstyle);
		app.setStyleSheet(ts.readAll());
	}

    MainWindow w;
    w.show();

    return app.exec();
}

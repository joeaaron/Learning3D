#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
	m_nPointsNum(0)
{
    ui->setupUi(this);

	Init();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(m_cloud.strDirName.c_str()), tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));
	if (fileNames.empty())
	{
		return;
	}

	for (int i = 0; i < fileNames.size(); ++i)
	{
		TimeStart();

		m_cloud.ptrCloud.reset(new Cloud);

		QString strPathName = fileNames[i];
		std::string strFileName = strPathName.toStdString();
		std::string strSubName = GetFileName(strFileName);

		int nStatus = -1;
		if (strPathName.endsWith(".pcd", Qt::CaseInsensitive))
		{
			nStatus = pcl::io::loadPCDFile(strFileName, *(m_cloud.ptrCloud));
			if (m_cloud.ptrCloud->points[0].r == 0 && m_cloud.ptrCloud->points[0].g == 0 && m_cloud.ptrCloud->points[0].b == 0)
			{
				SetCloudColor(255, 255, 255);
			}
		}
		else if (strPathName.endsWith(".ply", Qt::CaseInsensitive))
		{
			nStatus = pcl::io::loadPLYFile(strFileName, *(m_cloud.ptrCloud));
			if (m_cloud.ptrCloud->points[0].r == 0 && m_cloud.ptrCloud->points[0].g == 0 && m_cloud.ptrCloud->points[0].b == 0)
			{
				SetCloudColor(255, 255, 255);
			}
		}
		else if (strPathName.endsWith(".obj", Qt::CaseInsensitive))
		{
			nStatus = pcl::io::loadOBJFile(strFileName, *(m_cloud.ptrCloud));
			if (m_cloud.ptrCloud->points[0].r == 0 && m_cloud.ptrCloud->points[0].g == 0 && m_cloud.ptrCloud->points[0].b == 0)
			{
				SetCloudColor(255, 255, 255);
			}
		}
		else
		{
			//��ʾ���޷���ȡ����.ply .pcd .obj������ļ�
			QMessageBox::information(this, tr("File format error"), tr("Can't open files except .ply .pcd .obj"));
			return;
		}

		if (nStatus != 0)
		{
			QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
			return;
		}

		SetA(255);

		m_cloud.strPathName = strFileName;
		m_cloud.strFileName = strSubName;
		m_cloud.strDirName = strFileName.substr(0, strFileName.size() - strSubName.size());
		m_vctCloud.push_back(m_cloud);  //�����Ƶ����������

		QString timeDiff = TimeOff();

		// ������Դ������
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(strSubName.c_str()));
		cloudName->setIcon(0, QIcon(":/resource/images/icon.png"));
		ui->dataTree->addTopLevelItem(cloudName);

		// �������
		ConsoleLog("Add", QString::fromLocal8Bit(m_cloud.strFileName.c_str()), QString::fromLocal8Bit(m_cloud.strPathName.c_str()), "Time cost: " + timeDiff + " s, Points: " + QString::number(m_cloud.ptrCloud->points.size()));

		m_nPointsNum += m_cloud.ptrCloud->points.size();
	}

	ViewerAddedCloud();
	SetPropertyTable();
}

void MainWindow::on_actionClear_triggered()
{
	m_vctCloud.clear();

	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	ui->dataTree->clear();
	ui->propertyTable->clear();   //������Դ���propertyTable

	QStringList header;
	header << "Property" << "Value";
	ui->propertyTable->setHorizontalHeaderLabels(header);

	//�������
	ConsoleLog("Clear", "All point clouds", "", "");
	//������ʾ
	ViewerCloud(); 
}

void MainWindow::on_actionSave_triggered()
{
	QString strSaveName = QFileDialog::getSaveFileName(this, tr("Save point cloud"),
		QString::fromLocal8Bit(m_cloud.strDirName.c_str()), tr("Point cloud data(*.pcd *.ply);;Allfile(*.*)"));
	std::string strFileName = strSaveName.toStdString();
	std::string strSubName = GetFileName(strFileName);

	if (strSaveName.isEmpty())
	{
		return;
	}

	if (m_vctCloud.size() > 1)
	{
		SaveMultiCloud();
		return;
	}

	int nStatus = -1;
	if (strSaveName.endsWith(".pcd", Qt::CaseInsensitive))
	{
		nStatus = pcl::io::savePCDFile(strFileName, *(m_cloud.ptrCloud));
	}
	else if (strSaveName.endsWith(".ply", Qt::CaseInsensitive))
	{
		nStatus = pcl::io::savePLYFile(strFileName, *(m_cloud.ptrCloud));
	}
	
	else //��ʾ���޷�����Ϊ����.ply .pcd������ļ�
	{
		QMessageBox::information(this, tr("File format error"),
			tr("Can't save files except .ply .pcd"));
		return;
	}
	//��ʾ����׺û���⣬�����޷�����
	if (nStatus != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"),
			tr("We can not save the file"));
		return;
	}

	//�������
	ConsoleLog("Save", QString::fromLocal8Bit(strSubName.c_str()), strSaveName, "Single save");

	setWindowTitle(strSaveName + " - EasyCloud");

	QMessageBox::information(this, tr("save point cloud file"),
		QString::fromLocal8Bit(("Save " + strSubName + " successfully!").c_str()));
}

void MainWindow::on_actionExit_triggered()
{
	this->close();
}

void MainWindow::on_actionCloudColor_triggered()
{
	QColor color = QColorDialog::getColor(Qt::white, this,
		"Select color for point cloud");

	if (color.isValid())
	{
		QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
		int nSelectedItemNum = ui->dataTree->selectedItems().size();

		if (0 == nSelectedItemNum)
		{
			for (int i = 0; i != m_vctCloud.size(); i++) 
			{
				for (int j = 0; j != m_vctCloud[i].ptrCloud->points.size(); j++) 
				{
					m_vctCloud[i].ptrCloud->points[j].r = color.red();
					m_vctCloud[i].ptrCloud->points[j].g = color.green();
					m_vctCloud[i].ptrCloud->points[j].b = color.blue();
				}
			}
			// �������
			ConsoleLog("Change cloud color", "All point clouds", 
				QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), 
				"");
		}
		else 
		{
			for (int i = 0; i != nSelectedItemNum; i++) 
			{
				int nCloudId = ui->dataTree->indexOfTopLevelItem(itemList[i]);
				for (int j = 0; j != m_vctCloud[nCloudId].ptrCloud->size(); j++)
				{
					m_vctCloud[nCloudId].ptrCloud->points[j].r = color.red();
					m_vctCloud[nCloudId].ptrCloud->points[j].g = color.green();
					m_vctCloud[nCloudId].ptrCloud->points[j].b = color.blue();
				}
			}
			// �������
			ConsoleLog("Change cloud color", "Point clouds selected", 
				QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()),
				"");
		}

		// ������ʾ
		ViewerCloud();
	}
}

void MainWindow::on_actionBGColor_triggered()
{
	QColor color = QColorDialog::getColor(Qt::white, this,
		"Select color for background");

	if (color.isValid())
	{
		viewer->setBackgroundColor(color.red() / 255.0,
			color.green() / 255.0, color.blue() / 255.0);

		// �������
		ConsoleLog("Change bg color", "Background", 
			QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()),
			"");
		// ������ʾ
		ViewerCloud();
	}
}

void MainWindow::on_actionUp_triggered()
{
	if (!m_pCloud->empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMax.z + 5 * m_dMaxLen, 0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMax.z, 0, 1, 0);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionBottom_triggered()
{
	if (!m_pCloud->empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMin.z - 5 * m_dMaxLen, 0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMax.z, 0, 1, 0);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionFront_triggered()
{
	if (!m_pCloud->empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), m_PointMin.y - 5 * m_dMaxLen, 0.5*(m_PointMin.z + m_PointMax.z), 0.5*(m_PointMin.x + m_PointMax.x), m_PointMin.y, 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionBack_triggered()
{
	if (!m_pCloud->empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), m_PointMax.y + 5 * m_dMaxLen, 0.5*(m_PointMin.z + m_PointMax.z), 0.5*(m_PointMin.x + m_PointMax.x), m_PointMin.y, 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionLeft_triggered()
{
	if (!m_pCloud->empty())
	{
		viewer->setCameraPosition(m_PointMin.x - 5 * m_dMaxLen, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), m_PointMax.x, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionRight_triggered()
{
	if (!m_pCloud->empty())
	{
		viewer->setCameraPosition(m_PointMax.x + 5 * m_dMaxLen, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), m_PointMax.x, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::Init()
{
	// �����ʼ��
	setWindowIcon(QIcon(tr(":/resource/images/icon.png")));
	setWindowTitle(tr("EasyCloud"));

	//���Ƴ�ʼ��
	m_cloud.ptrCloud.reset(new Cloud);
	m_cloud.ptrCloud->resize(1);

	m_pCloud.reset(new Cloud);
	m_pInputCloud.reset(new Cloud);

	//���ӻ������ʼ��
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//����VTK���ӻ�����ָ��
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	//���ô��ڽ��������ڿɽ��ܼ��̵��¼�
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	ui->qvtkWidget->update();

	ui->propertyTable->setSelectionMode(QAbstractItemView::NoSelection);	// ��ֹ������Թ������� item
	ui->consoleTable->setSelectionMode(QAbstractItemView::NoSelection);		// ��ֹ���������ڵ� item
	ui->dataTree->setSelectionMode(QAbstractItemView::ExtendedSelection);	// ����dataTree���ж�ѡ

	//����Ĭ������
	/*QString qss = darcula_qss;
	qApp->setStyleSheet(qss);*/
	//���Թ�����
	SetPropertyTable();
	// �������
	SetConsoleTable();

	ConsoleLog("Software start", "EasyCloud", "Welcome to use EasyCloud", "Z");
	// ���ñ�����ɫΪ dark
	viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
}

void MainWindow::SetPropertyTable()
{
	QStringList header;
	header << "Property" << "Value";

	ui->propertyTable->setHorizontalHeaderLabels(header);
	ui->propertyTable->setItem(0, 0, new QTableWidgetItem("Clouds"));
	ui->propertyTable->setItem(1, 0, new QTableWidgetItem("Points"));
	ui->propertyTable->setItem(2, 0, new QTableWidgetItem("Total points"));
	ui->propertyTable->setItem(3, 0, new QTableWidgetItem("RGB"));

	ui->propertyTable->setItem(0, 1, new QTableWidgetItem(QString::number(m_vctCloud.size())));
	ui->propertyTable->setItem(1, 1, new QTableWidgetItem(""));
	ui->propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(m_nPointsNum)));
	ui->propertyTable->setItem(4, 1, new QTableWidgetItem(""));
}

void MainWindow::SetConsoleTable() {
	// �����������
	QStringList header;
	header << "Time" << "Operation" << "Operation object" << "Details" << "Note";

	ui->consoleTable->setHorizontalHeaderLabels(header);
	ui->consoleTable->setColumnWidth(0, 150);
	ui->consoleTable->setColumnWidth(1, 200);
	ui->consoleTable->setColumnWidth(2, 200);
	ui->consoleTable->setColumnWidth(3, 300);

	//ui.consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); //���ò��ɱ༭
	ui->consoleTable->verticalHeader()->setDefaultSectionSize(22);		   //�����о�

	ui->consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
}

void MainWindow::ConsoleLog(QString operation, QString subName, QString fileName, QString note)
{
	if (!m_bEnableConsole)
	{
		return;
	}

	int nRowsNum = ui->consoleTable->rowCount();
	ui->consoleTable->setRowCount(++nRowsNum);
	QDateTime time = QDateTime::currentDateTime();			//��ȡϵͳ���ڵ�ʱ��
	QString strTime = time.toString("MM-dd hh:mm:ss");		//������ʾ��ʽ
	ui->consoleTable->setItem(nRowsNum - 1, 0, new QTableWidgetItem(strTime));
	ui->consoleTable->setItem(nRowsNum - 1, 1, new QTableWidgetItem(operation));
	ui->consoleTable->setItem(nRowsNum - 1, 2, new QTableWidgetItem(subName));
	ui->consoleTable->setItem(nRowsNum - 1, 3, new QTableWidgetItem(fileName));
	ui->consoleTable->setItem(nRowsNum - 1, 4, new QTableWidgetItem(note));

	ui->consoleTable->scrollToBottom();						// �����Զ�������ײ�
}

void MainWindow::ViewerCloud()
{
	for (int i = 0; i != m_vctCloud.size(); i++)
	{
		viewer->updatePointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());
	}
	//viewer->resetCamera();
	ui->qvtkWidget->update();
}

void MainWindow::ViewerAddedCloud()
{
	for (int i = 0; i != m_vctCloud.size(); i++)
	{
		// ��ӵ�����
		viewer->addPointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());
		viewer->updatePointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());
	}

	//�����ӽ�
	viewer->resetCamera();

	//ˢ�´���
	ui->qvtkWidget->update();
}

void MainWindow::UpdateDisplay()
{
	// ��յ���
	m_pCloud->clear();
	viewer->removeAllPointClouds();
	viewer->removeAllCoordinateSystems();

	// �������Ƿ������Чֵ
	if (m_pInputCloud->is_dense)
	{
		pcl::copyPointCloud(*m_pInputCloud, *m_pCloud);
	}
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*m_pInputCloud, *m_pCloud, vec);
	}

	// ��ӵ�����
	viewer->addPointCloud(m_pCloud);

	pcl::getMinMax3D(*m_pCloud, m_PointMin, m_PointMax);

	m_dMaxLen = GetMaxValue(m_PointMin, m_PointMax);

	//�����ӽ�
	viewer->resetCamera();

	//ˢ�´���
	ui->qvtkWidget->update();
}

double MainWindow::GetMaxValue(PointT p1, PointT p2)
{
	double max = 0;

	if (p1.x - p2.x > p1.y - p2.y)
	{
		max = p1.x - p2.x;

	}
	else
	{
		max = p1.y - p2.y;
	}

	if (max < p1.z - p2.z)
	{
		max = p1.z - p2.z;
	}

	return max;
}

void MainWindow::TimeStart()
{
	time.start();
}

QString MainWindow::TimeOff()
{
	int nTimeDiff = time.elapsed();		//���ش��ϴ�start()��restart()��ʼ������ʱ����λms
	QString strTimediff = QString("%1").arg(nTimeDiff /1000.0);  //float->QString
	return strTimediff;
}

std::string MainWindow::GetFileName(std::string strFileName)
{
	std::string strSubName;

	for (auto i = strFileName.end()- 1; *i!='/'; i--)
	{
		strSubName.insert(strSubName.begin(), *i);
	}

	return strSubName;
}

void MainWindow::SetCloudColor(unsigned int r, unsigned int g, unsigned int b)
{
	for (size_t i = 0; i < m_cloud.ptrCloud->size(); i++)
	{
		m_cloud.ptrCloud->points[i].r = r;
		m_cloud.ptrCloud->points[i].g = g;
		m_cloud.ptrCloud->points[i].b = b;
		m_cloud.ptrCloud->points[i].a = 255;
	}
}

void MainWindow::SetA(unsigned int a)
{
	for (size_t i = 0; i < m_cloud.ptrCloud->size(); i++)
	{
		m_cloud.ptrCloud->points[i].a = a;
	}
}

void MainWindow::SaveMultiCloud()
{

}
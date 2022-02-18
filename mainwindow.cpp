#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	//点云初始化
	m_pCloud.reset(new Cloud);

	m_pInputCloud.reset(new Cloud);

	//可视化对象初始化
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	//设置VTK可视化窗口指针
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//设置窗口交互，窗口可接受键盘等事件
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

	ui->qvtkWidget->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{
	//获取点云路径
	QString strPath = QFileDialog::getOpenFileName(this, "选择点云", ".//", "点云文件(*.ply *.pcd);;所有文件(*.*)");

	if (strPath.isEmpty())
	{
		return;
	}

	int nStatus = 0;
	if (strPath.endsWith(".pcd", Qt::CaseInsensitive))
	{
		nStatus = pcl::io::loadPCDFile(strPath.toStdString(), *m_pInputCloud);
	}
	else if (strPath.endsWith(".ply", Qt::CaseInsensitive))
	{
		nStatus = pcl::io::loadPLYFile(strPath.toStdString(), *m_pInputCloud);
	}
	else
	{
		return;
	}

	// 点云显示
	UpdateDisplay();
}

void MainWindow::UpdateDisplay()
{
	// 清空点云
	m_pCloud->clear();
	viewer->removeAllPointClouds();
	viewer->removeAllCoordinateSystems();

	// 点云中是否包含无效值
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

	// 添加到窗口
	viewer->addPointCloud(m_pCloud);

	pcl::getMinMax3D(*m_pCloud, m_PointMin, m_PointMax);

	//重设视角
	viewer->resetCamera();

	//刷新窗口
	ui->qvtkWidget->update();
}
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	//���Ƴ�ʼ��
	m_pCloud.reset(new Cloud);

	m_pInputCloud.reset(new Cloud);

	//���ӻ������ʼ��
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	//����VTK���ӻ�����ָ��
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//���ô��ڽ��������ڿɽ��ܼ��̵��¼�
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

	ui->qvtkWidget->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{
	//��ȡ����·��
	QString strPath = QFileDialog::getOpenFileName(this, "ѡ�����", ".//", "�����ļ�(*.ply *.pcd);;�����ļ�(*.*)");

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

	// ������ʾ
	UpdateDisplay();
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

	//�����ӽ�
	viewer->resetCamera();

	//ˢ�´���
	ui->qvtkWidget->update();
}
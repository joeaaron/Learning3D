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
	// ��ȡ����·��
	QString strPath = QFileDialog::getOpenFileName(this, "ѡ�����", ".//", "�����ļ�(*.ply *.pcd);;�����ļ�(*.*)");

	if (strPath.isEmpty())
	{
		return;
	}

	// ��ȡ�������
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
#include "mainwindow.h"
#include "ui_mainwindow.h"

const int WIDGET_INT_SPACE = 10;
//���������ɫ
int *rand_rgb()
{
	int *rgb = new int[3];
	rgb[0] = rand() % 255;
	rgb[1] = rand() % 255;
	rgb[2] = rand() % 255;
	return rgb;
}

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
	if (ui != nullptr)
	{
		ui = nullptr;
		delete ui;
	}

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

		QCheckBox* ptn = new QCheckBox(this);
		//ptn->setIcon(QIcon(tr(":/resource/images/show.png")));
		ui->dataTree->setItemWidget(cloudName, 1, ptn);
		connect(ptn, SIGNAL(clicked(bool)), this, SLOT(itemClicked(bool)));

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
	ViewerCloud(m_vctCloud);
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
		ViewerCloud(m_vctCloud);
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
		ViewerCloud(m_vctCloud);
	}
}

void MainWindow::on_actionUp_triggered()
{
	if (!m_vctCloud.empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMax.z + 5 * m_dMaxLen, 0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMax.z, 0, 1, 0);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionBottom_triggered()
{
	if (!m_vctCloud.empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMin.z - 5 * m_dMaxLen, 0.5*(m_PointMin.x + m_PointMax.x), 0.5*(m_PointMin.y + m_PointMax.y), m_PointMax.z, 0, 1, 0);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionFront_triggered()
{
	if (!m_vctCloud.empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), m_PointMin.y - 5 * m_dMaxLen, 0.5*(m_PointMin.z + m_PointMax.z), 0.5*(m_PointMin.x + m_PointMax.x), m_PointMin.y, 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionBack_triggered()
{
	if (!m_vctCloud.empty())
	{
		viewer->setCameraPosition(0.5*(m_PointMin.x + m_PointMax.x), m_PointMax.y + 5 * m_dMaxLen, 0.5*(m_PointMin.z + m_PointMax.z), 0.5*(m_PointMin.x + m_PointMax.x), m_PointMin.y, 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionLeft_triggered()
{
	if (!m_vctCloud.empty())
	{
		viewer->setCameraPosition(m_PointMin.x - 5 * m_dMaxLen, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), m_PointMax.x, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionRight_triggered()
{
	if (!m_vctCloud.empty())
	{
		viewer->setCameraPosition(m_PointMax.x + 5 * m_dMaxLen, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), m_PointMax.x, 0.5*(m_PointMin.y + m_PointMax.y), 0.5*(m_PointMin.z + m_PointMax.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

void MainWindow::on_actionSORFilter_triggered()
{
	QPoint point = ui->qvtkWidget->mapToGlobal(QPoint(ui->qvtkWidget->x(), ui->qvtkWidget->y()));
	double dx = point.x();
	double dy = point.y();

	m_cloudFilter.move(dx + WIDGET_INT_SPACE, dy + WIDGET_INT_SPACE);
	m_cloudFilter.show();

	connect(&m_cloudFilter, SIGNAL(runBtnClicked()), this, SLOT(on_cloudFilter_triggered()));
}

void MainWindow::on_actionDownSample_triggered()
{
	if (!m_vctCloud.empty())
	{
		m_nPointsNum = 0;
		
		TimeStart();

		for (int i = 0; i != m_vctCloud.size(); i++)
		{
			pcl::VoxelGrid<PointT> vg;
			Cloud::Ptr ptrCloudFiltered(new Cloud);

			vg.setInputCloud(m_vctCloud[i].ptrCloud);
			vg.setLeafSize(0.01f, 0.01f, 0.01f);

			vg.filter(*ptrCloudFiltered);

			int nDownSamplePoints = ptrCloudFiltered->points.size();
			m_nPointsNum += nDownSamplePoints;

			copyPointCloud(*ptrCloudFiltered, *m_vctCloud[i].ptrCloud);
		}
		
		QString timeDiff = TimeOff();

		// ���Դ���
		SetPropertyTable();
		//�������
		ConsoleLog("DownSample", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
		//������ʾ
		ViewerCloud(m_vctCloud);
	}
}

void MainWindow::on_actionRansacSeg_triggered()
{
	if (!m_vctCloud.empty())
	{
		m_nPointsNum = 0;

		TimeStart();

		for (int i = 0; i != m_vctCloud.size(); i++)
		{
			pcl::SACSegmentation<PointT> seg;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			Cloud::Ptr ptrCloudPlane(new Cloud);
			Cloud::Ptr ptrCloudObj(new Cloud);

			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations(100);
			seg.setDistanceThreshold(0.02);
			seg.setInputCloud(m_vctCloud[i].ptrCloud);
			seg.segment(*inliers, *coefficients);

			if (0 == inliers->indices.size())
			{
				std::cout << "Could not estimate a planar anymore." << std::endl;
			}
			else
			{
				pcl::ExtractIndices<PointT> extract;
				extract.setInputCloud(m_vctCloud[i].ptrCloud);
				extract.setIndices(inliers);
				extract.setNegative(false);
				extract.filter(*ptrCloudPlane);

				pcl::PointXYZ min, max;
				pcl::getMinMax3D(*ptrCloudPlane, m_PointMin, m_PointMax);
				double dMinZ = m_PointMin.z;

				int nRansacSegPoints = ptrCloudPlane->points.size();
				m_nPointsNum += nRansacSegPoints;

				//m_vctCloud[i].ptrCloud = ptrCloudPlane;

				// filter plannar
				extract.setNegative(true);
				extract.filter(*ptrCloudObj);
			
				copyPointCloud(*ptrCloudObj, *m_vctCloud[i].ptrCloud);
			}
		}

		QString timeDiff = TimeOff();

		// ���Դ���
		SetPropertyTable();
		//�������
		ConsoleLog("Ransac", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
		//������ʾ
		ViewerCloud(m_vctCloud);
	}
}

void MainWindow::on_actionDbScan_triggered()
{
	if (!m_vctCloud.empty())
	{
		m_nPointsNum = 0;

		TimeStart();

		for (int i = 0; i != m_vctCloud.size(); i++)
		{
			// KdTree, for more information, please ref [How to use a KdTree to search](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search)
			pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
			tree->setInputCloud(m_vctCloud[i].ptrCloud);

			std::vector<pcl::PointIndices> clusterIndices;

			DBSCANKdtreeCluster<PointT> ec;
			ec.setCorePointMinPts(20);

			ec.setClusterTolerance(0.05);
			ec.setMinClusterSize(100);
			ec.setMaxClusterSize(25000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(m_vctCloud[i].ptrCloud);
			ec.extract(clusterIndices);

			Cloud::Ptr cloudClustered(new Cloud);
			int j = 0;
			// visualization, use indensity to show different color for each cluster.
			for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++, j++)
			{
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				{
					PointT tmp;
					tmp.x = m_vctCloud[i].ptrCloud->points[*pit].x;
					tmp.y = m_vctCloud[i].ptrCloud->points[*pit].y;
					tmp.z = m_vctCloud[i].ptrCloud->points[*pit].z;
					//tmp.intensity = j % 8;
					cloudClustered->points.push_back(tmp);
				}
			}
			
			int nClusterPoints = cloudClustered->points.size();
			m_nPointsNum += nClusterPoints;

			//cloudClustered->width = nClusterPoints;
			//cloudClustered->height = 1;
			//copyPointCloud(*cloudClustered, *m_vctCloud[i].ptrCloud);

		}

		QString timeDiff = TimeOff();

		// ���Դ���
		SetPropertyTable();
		//�������
		ConsoleLog("DBSCAN", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
	}
}

void MainWindow::on_actionMeshSurface_triggered()
{
	if (!m_vctCloud.empty())
	{
		TimeStart();

		pcl::PointXYZ point;
		ptrCloudXYZ.reset(new pcl::PointCloud<pcl::PointXYZ>);

		for (size_t i = 0; i != m_cloud.ptrCloud->size(); i++)
		{
			point.x = m_cloud.ptrCloud->points[i].x;
			point.y = m_cloud.ptrCloud->points[i].y;
			point.z = m_cloud.ptrCloud->points[i].z;
			ptrCloudXYZ->push_back(point);
		}

		if (!ptrCloudXYZ)
		{
			return;
		}

		/****** �������ģ�� ******/
		// �������߹��ƶ��� n
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		// ������������ָ�� normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		// ���� kdtree ���ڷ������ʱ��������
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(ptrCloudXYZ);	 // Ϊ kdtree �������
		n.setInputCloud(ptrCloudXYZ);		 // Ϊ������ƶ����������
		n.setSearchMethod(tree);			 // ���÷������ʱ����ȡ��������ʽΪkdtree
		n.setKSearch(20);					 // ���÷������ʱ��k���������ĵ���
		n.compute(*normals);				 // ���з������

		QMessageBox::information(this, "information", "Normal estimation finished");

		/****** ���������뷨������ƴ�� ******/
		// ����ͬʱ������ͷ��ߵ����ݽṹָ��
		pcl::PointCloud<pcl::PointNormal>::Ptr ptrCloudWithNormal(new  pcl::PointCloud<pcl::PointNormal>);
		// ���ѻ�õĵ����ݺͷ�������ƴ��
		pcl::concatenateFields(*ptrCloudXYZ, *normals, *ptrCloudWithNormal);

		//������һ��kdtree�����ؽ�
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		//Ϊkdtree����������ݣ��õ�����������Ϊ��ͷ���
		tree2->setInputCloud(ptrCloudWithNormal);

		/****** �����ؽ�ģ�� ******/
		// ����̰��������ͶӰ�ؽ�����
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		// ���������������������洢�ؽ����
		pcl::PolygonMesh triangles;
		// ���ò���
		gp3.setSearchRadius(25);				// �������ӵ�֮�������룬����ȷ��k���ڵ���뾶
		gp3.setMu(2.5);							// ��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶
		gp3.setMaximumNearestNeighbors(100);	// ��������������ڵ���������
		gp3.setMaximumSurfaceAngle(M_PI / 2);	// 45�� ���ƽ���
		gp3.setMinimumAngle(M_PI / 18);			// 10�� ÿ�����ǵ����Ƕȣ�
		gp3.setMaximumAngle(2 * M_PI / 3);		// 120��
		gp3.setNormalConsistency(false);		// ��������һ�£���Ϊtrue
		// ���õ������ݺ�������ʽ
		gp3.setInputCloud(ptrCloudWithNormal);
		gp3.setSearchMethod(tree2);
		// ��ʼ�ؽ�
		gp3.reconstruct(triangles);
		QMessageBox::information(this, "informaiton", "Reconstruction finished");

		QString timeDiff = TimeOff();

		/****** ͼ����ʾģ�� ******/
		QMessageBox::information(this, "informaiton", "Start to show");
		viewer->addPolygonMesh(triangles, "my"); //����Ҫ��ʾ���������
		//��������ģ����ʾģʽ
		viewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ
		//viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ
		//viewer->setRepresentationToWireframeForAllActors(); //����ģ�����߿�ͼģʽ��ʾ

		// �������
		ConsoleLog("Convert surface", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));

		viewer->removeAllShapes();
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		return;
	}
}

void MainWindow::on_actionWireFrame_triggered()
{
	if (!m_vctCloud.empty())
	{
		TimeStart();

		pcl::PointXYZ point;
		ptrCloudXYZ.reset(new pcl::PointCloud<pcl::PointXYZ>);

		for (size_t i = 0; i != m_cloud.ptrCloud->size(); i++)
		{
			point.x = m_cloud.ptrCloud->points[i].x;
			point.y = m_cloud.ptrCloud->points[i].y;
			point.z = m_cloud.ptrCloud->points[i].z;
			ptrCloudXYZ->push_back(point);
		}

		if (!ptrCloudXYZ)
		{
			return;
		}

		/****** �������ģ�� ******/
		// �������߹��ƶ��� n
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		// ������������ָ�� normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		// ���� kdtree ���ڷ������ʱ��������
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(ptrCloudXYZ);	 // Ϊ kdtree �������
		n.setInputCloud(ptrCloudXYZ);		 // Ϊ������ƶ����������
		n.setSearchMethod(tree);			 // ���÷������ʱ����ȡ��������ʽΪkdtree
		n.setKSearch(20);					 // ���÷������ʱ��k���������ĵ���
		n.compute(*normals);				 // ���з������

		QMessageBox::information(this, "information", "Normal estimation finished");

		/****** ���������뷨������ƴ�� ******/
		// ����ͬʱ������ͷ��ߵ����ݽṹָ��
		pcl::PointCloud<pcl::PointNormal>::Ptr ptrCloudWithNormal(new  pcl::PointCloud<pcl::PointNormal>);
		// ���ѻ�õĵ����ݺͷ�������ƴ��
		pcl::concatenateFields(*ptrCloudXYZ, *normals, *ptrCloudWithNormal);

		//������һ��kdtree�����ؽ�
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		//Ϊkdtree����������ݣ��õ�����������Ϊ��ͷ���
		tree2->setInputCloud(ptrCloudWithNormal);

		/****** �����ؽ�ģ�� ******/
		// ����̰��������ͶӰ�ؽ�����
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		// ���������������������洢�ؽ����
		pcl::PolygonMesh triangles;
		// ���ò���
		gp3.setSearchRadius(25);				// �������ӵ�֮�������룬����ȷ��k���ڵ���뾶
		gp3.setMu(2.5);							// ��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶
		gp3.setMaximumNearestNeighbors(100);	// ��������������ڵ���������
		gp3.setMaximumSurfaceAngle(M_PI / 2);	// 45�� ���ƽ���
		gp3.setMinimumAngle(M_PI / 18);			// 10�� ÿ�����ǵ����Ƕȣ�
		gp3.setMaximumAngle(2 * M_PI / 3);		// 120��
		gp3.setNormalConsistency(false);		// ��������һ�£���Ϊtrue
		// ���õ������ݺ�������ʽ
		gp3.setInputCloud(ptrCloudWithNormal);
		gp3.setSearchMethod(tree2);
		// ��ʼ�ؽ�
		gp3.reconstruct(triangles);
		QMessageBox::information(this, "informaiton", "Reconstruction finished");

		QString timeDiff = TimeOff();

		/****** ͼ����ʾģ�� ******/
		QMessageBox::information(this, "informaiton", "Start to show");
		viewer->addPolygonMesh(triangles, "my"); //����Ҫ��ʾ���������
		//��������ģ����ʾģʽ
		//viewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ
		//viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ
		viewer->setRepresentationToWireframeForAllActors(); //����ģ�����߿�ͼģʽ��ʾ

		// �������
		ConsoleLog("Convert wireframe", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));

		viewer->removeAllShapes();
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		return;
	}
}

void MainWindow::on_actionAbout_triggered()
{
	AboutWin* pAboutWin = new AboutWin(this);

	pAboutWin->setModal(true);
	pAboutWin->show();

	// �������
	ConsoleLog("About", "Z", "http://nightn.com", "Welcome to my blog!");
}

void MainWindow::itemSelected(QTreeWidgetItem* item, int count)
{
	count = ui->dataTree->indexOfTopLevelItem(item);  //��ȡitem���к�

	for (int i = 0; i != m_vctCloud.size(); i++)
	{
		viewer->updatePointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + QString::number(i).toStdString());
	}

	//��ȡ��ǰ���Ƶ�RGB,������������Ϣ
	int nCloudSize = m_vctCloud[count].ptrCloud->points.size();
	unsigned int cloud_r = m_vctCloud[count].ptrCloud->points[0].r;
	unsigned int cloud_g = m_vctCloud[count].ptrCloud->points[0].g;
	unsigned int cloud_b = m_vctCloud[count].ptrCloud->points[0].b;

	bool bMultiColor = true;
	if (m_vctCloud[count].ptrCloud->points.begin()->r == (m_vctCloud[count].ptrCloud->points.end() - 1)->r)			//�жϵ��Ƶ�ɫ��ɫ�����������Ǻ��Ͻ���
		bMultiColor = false;

	ui->propertyTable->setItem(0, 1, new QTableWidgetItem(QString::number(m_vctCloud.size())));
	ui->propertyTable->setItem(1, 1, new QTableWidgetItem(QString::number(nCloudSize)));
	ui->propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(m_nPointsNum)));
	ui->propertyTable->setItem(3, 1, new QTableWidgetItem(bMultiColor ? "Multi Color" : (QString::number(cloud_r) + " " + QString::number(cloud_g) + " " + QString::number(cloud_b))));

	//ѡ��item����Ӧ�ĵ��Ƴߴ���
	QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
	int selected_item_count = ui->dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++) 
	{
		int cloud_id = ui->dataTree->indexOfTopLevelItem(itemList[i]);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			2, "cloud" + QString::number(cloud_id).toStdString());
	}
	//mycloud = mycloud_vec[count];
	ui->qvtkWidget->update();
}

void MainWindow::itemClicked(bool bState)
{
	QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();

	for (int i = 0; i != ui->dataTree->selectedItems().size(); i++) 
	{
		//QTreeWidgetItem* curItem = ui.dataTree->currentItem();
		QTreeWidgetItem* curItem = itemList[i];

		int id = ui->dataTree->indexOfTopLevelItem(curItem);
		std::string strCloudId = "cloud" + QString::number(id).toStdString();
		//QMessageBox::information(this, "cloud_id", QString::fromLocal8Bit(cloud_id.c_str()));

		if (bState)
		{
			// ��strCloudId����Ӧ�ĵ������ó�͸��
			viewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.0, strCloudId, 0);

			QColor item_color = QColor(112, 122, 132, 255);
			curItem->setTextColor(0, item_color);

			m_vctCloud[id].bVisible = false;
			// �������
			ConsoleLog("Hide point clouds", "Point clouds selected", "", "");
		}
		else
		{
			viewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, strCloudId, 0);

			QColor item_color = QColor(0, 0, 0, 255);
			curItem->setTextColor(0, item_color);

			m_vctCloud[id].bVisible = true;
			// �������
			ConsoleLog("Show point clouds", "Point clouds selected", "", "");
		}
	}

	ui->qvtkWidget->update();
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

	// ���ݴ���
	SetDataTable();
	// ���Թ�����
	SetPropertyTable();
	// �������
	SetConsoleTable();

	ConsoleLog("Software start", "EasyCloud", "Welcome to use EasyCloud", "Z");
	// ���ñ�����ɫΪ dark
	viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
	//�������ϵ
	/*int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);*/

	//viewer->addCoordinateSystem(0.5);
}

void MainWindow::SetDataTable()
{
	ui->dataTree->setColumnCount(2);

	ui->dataTree->setColumnWidth(0, 150);
	ui->dataTree->header()->setSectionResizeMode(1, QHeaderView::Fixed);
	ui->dataTree->setColumnWidth(1, 20);
	//ui->dataTree->setStyleSheet("QTreeView::item{width:250px;height:30px;}");
}

void MainWindow::SetPropertyTable()
{
	QStringList header;
	header << "Property" << "Value";

	QHeaderView* headerView = ui->propertyTable->horizontalHeader();
	headerView->setDefaultAlignment(Qt::AlignLeft);

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

	QHeaderView* headerView = ui->consoleTable->horizontalHeader();
	headerView->setDefaultAlignment(Qt::AlignLeft);
	headerView->setSectionResizeMode(QHeaderView::Stretch);

	ui->consoleTable->setHorizontalHeaderLabels(header);
	
	int nWidth = ui->consoleTable->width();
	ui->consoleTable->setColumnWidth(0, nWidth * 0.15);
	ui->consoleTable->setColumnWidth(1, nWidth * 0.1);
	ui->consoleTable->setColumnWidth(2, nWidth * 0.15);
	ui->consoleTable->setColumnWidth(3, nWidth * 0.4);
	ui->consoleTable->setColumnWidth(4, nWidth * 0.2);

	ui->consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); //���ò��ɱ༭
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

void MainWindow::ViewerCloud(const std::vector<MyCloud::CloudStructure> vctCloud)
{
	for (int i = 0; i != vctCloud.size(); i++)
	{
		if (vctCloud[i].ptrCloud == nullptr)
		{
			return;
		}
		viewer->updatePointCloud(vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());

		pcl::getMinMax3D(*vctCloud[i].ptrCloud, m_PointMin, m_PointMax);
		m_dMaxLen = GetMaxValue(m_PointMin, m_PointMax);
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

		pcl::getMinMax3D(*m_vctCloud[i].ptrCloud, m_PointMin, m_PointMax);
		m_dMaxLen = GetMaxValue(m_PointMin, m_PointMax);
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

void MainWindow::on_cloudFilter_triggered()
{
	if (!m_vctCloud.empty())
	{
		m_nPointsNum = 0;

		TimeStart();

		for (int i = 0; i != m_vctCloud.size(); i++)
		{
			Cloud::Ptr ptrCloudFiltered(new Cloud);

			// 1.��ȡ�˲�����
			int nFilterType = m_cloudFilter.GetFilterType();

			// 2.ͳ���˲�
			if (0 == nFilterType)
			{
				pcl::StatisticalOutlierRemoval<PointT> sor;
				
				double dMeanK = m_cloudFilter.GetMeanKVal();
				double dStdDev = m_cloudFilter.GetStdVal();

				sor.setInputCloud(m_vctCloud[i].ptrCloud);
				sor.setMeanK(dMeanK);
				sor.setStddevMulThresh(dStdDev);

				sor.filter(*ptrCloudFiltered);

			}
			// 3.ֱͨ�˲�
			else
			{
				pcl::PassThrough<PointT> pass;
				Cloud::Ptr ptrCloudFiltered(new Cloud);

				pass.setInputCloud(m_vctCloud[i].ptrCloud);

				std::string strFiledName = m_cloudFilter.GetFieldName().toStdString();
				pass.setFilterFieldName(strFiledName);

				double dMinLimit = m_cloudFilter.GetMinLimit();
				double dMaxLimit = m_cloudFilter.GetMaxLimit();
				pass.setFilterLimits(dMinLimit, dMaxLimit);

				pass.setNegative(true);		// true:������Χ��ĵ�; false:������Χ�ڵĵ�
				pass.filter(*ptrCloudFiltered);
			}

			int nFilteredPoints = ptrCloudFiltered->points.size();
			m_nPointsNum += nFilteredPoints;

			copyPointCloud(*ptrCloudFiltered, *m_vctCloud[i].ptrCloud);
		}

		QString timeDiff = TimeOff();

		// ���Դ���
		SetPropertyTable();
		//�������
		ConsoleLog("CloudFilter", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
		//������ʾ
		ViewerCloud(m_vctCloud);
	}
}



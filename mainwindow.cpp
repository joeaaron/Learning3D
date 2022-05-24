#include "mainwindow.h"
#include "ui_mainwindow.h"

const int WIDGET_INT_SPACE = 10;
//随机产生颜色
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
			//提示：无法读取除了.ply .pcd .obj以外的文件
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
		m_vctCloud.push_back(m_cloud);  //将点云导入点云容器

		QString timeDiff = TimeOff();

		// 设置资源管理器
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(strSubName.c_str()));
		cloudName->setIcon(0, QIcon(":/resource/images/icon.png"));
		ui->dataTree->addTopLevelItem(cloudName);

		QCheckBox* ptn = new QCheckBox(this);
		//ptn->setIcon(QIcon(tr(":/resource/images/show.png")));
		ui->dataTree->setItemWidget(cloudName, 1, ptn);
		connect(ptn, SIGNAL(clicked(bool)), this, SLOT(itemClicked(bool)));

		// 输出窗口
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
	ui->propertyTable->clear();   //清空属性窗口propertyTable

	QStringList header;
	header << "Property" << "Value";
	ui->propertyTable->setHorizontalHeaderLabels(header);

	//输出窗口
	ConsoleLog("Clear", "All point clouds", "", "");
	//更新显示
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
	
	else //提示：无法保存为除了.ply .pcd以外的文件
	{
		QMessageBox::information(this, tr("File format error"),
			tr("Can't save files except .ply .pcd"));
		return;
	}
	//提示：后缀没问题，但是无法保存
	if (nStatus != 0)
	{
		QMessageBox::critical(this, tr("Saving file error"),
			tr("We can not save the file"));
		return;
	}

	//输出窗口
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
			// 输出窗口
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
			// 输出窗口
			ConsoleLog("Change cloud color", "Point clouds selected", 
				QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()),
				"");
		}

		// 更新显示
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

		// 输出窗口
		ConsoleLog("Change bg color", "Background", 
			QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()),
			"");
		// 更新显示
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

		// 属性窗口
		SetPropertyTable();
		//输出窗口
		ConsoleLog("DownSample", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
		//更新显示
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

		// 属性窗口
		SetPropertyTable();
		//输出窗口
		ConsoleLog("Ransac", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
		//更新显示
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

		// 属性窗口
		SetPropertyTable();
		//输出窗口
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

		/****** 法向估计模块 ******/
		// 创建法线估计对象 n
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		// 创建法向数据指针 normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		// 创建 kdtree 用于法向计算时近邻搜索
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(ptrCloudXYZ);	 // 为 kdtree 输入点云
		n.setInputCloud(ptrCloudXYZ);		 // 为法向估计对象输入点云
		n.setSearchMethod(tree);			 // 设置法向估计时所采取的搜索方式为kdtree
		n.setKSearch(20);					 // 设置法向估计时，k近邻搜索的点数
		n.compute(*normals);				 // 进行法向估计

		QMessageBox::information(this, "information", "Normal estimation finished");

		/****** 点云数据与法向数据拼接 ******/
		// 创建同时包含点和法线的数据结构指针
		pcl::PointCloud<pcl::PointNormal>::Ptr ptrCloudWithNormal(new  pcl::PointCloud<pcl::PointNormal>);
		// 将已获得的点数据和法向数据拼接
		pcl::concatenateFields(*ptrCloudXYZ, *normals, *ptrCloudWithNormal);

		//创建另一个kdtree用于重建
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		//为kdtree输入点云数据，该点云数据类型为点和法向
		tree2->setInputCloud(ptrCloudWithNormal);

		/****** 曲面重建模块 ******/
		// 创建贪婪三角形投影重建对象
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		// 创建多边形网格对象，用来存储重建结果
		pcl::PolygonMesh triangles;
		// 设置参数
		gp3.setSearchRadius(25);				// 设置连接点之间最大距离，用于确定k近邻的球半径
		gp3.setMu(2.5);							// 设置最近邻距离的乘子，以得到每个点的最终搜索半径
		gp3.setMaximumNearestNeighbors(100);	// 设置搜索的最近邻点的最大数量
		gp3.setMaximumSurfaceAngle(M_PI / 2);	// 45度 最大平面角
		gp3.setMinimumAngle(M_PI / 18);			// 10度 每个三角的最大角度？
		gp3.setMaximumAngle(2 * M_PI / 3);		// 120度
		gp3.setNormalConsistency(false);		// 若法向量一致，设为true
		// 设置点云数据和搜索方式
		gp3.setInputCloud(ptrCloudWithNormal);
		gp3.setSearchMethod(tree2);
		// 开始重建
		gp3.reconstruct(triangles);
		QMessageBox::information(this, "informaiton", "Reconstruction finished");

		QString timeDiff = TimeOff();

		/****** 图形显示模块 ******/
		QMessageBox::information(this, "informaiton", "Start to show");
		viewer->addPolygonMesh(triangles, "my"); //设置要显示的网格对象
		//设置网格模型显示模式
		viewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示
		//viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示
		//viewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示

		// 输出窗口
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

		/****** 法向估计模块 ******/
		// 创建法线估计对象 n
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		// 创建法向数据指针 normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		// 创建 kdtree 用于法向计算时近邻搜索
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(ptrCloudXYZ);	 // 为 kdtree 输入点云
		n.setInputCloud(ptrCloudXYZ);		 // 为法向估计对象输入点云
		n.setSearchMethod(tree);			 // 设置法向估计时所采取的搜索方式为kdtree
		n.setKSearch(20);					 // 设置法向估计时，k近邻搜索的点数
		n.compute(*normals);				 // 进行法向估计

		QMessageBox::information(this, "information", "Normal estimation finished");

		/****** 点云数据与法向数据拼接 ******/
		// 创建同时包含点和法线的数据结构指针
		pcl::PointCloud<pcl::PointNormal>::Ptr ptrCloudWithNormal(new  pcl::PointCloud<pcl::PointNormal>);
		// 将已获得的点数据和法向数据拼接
		pcl::concatenateFields(*ptrCloudXYZ, *normals, *ptrCloudWithNormal);

		//创建另一个kdtree用于重建
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		//为kdtree输入点云数据，该点云数据类型为点和法向
		tree2->setInputCloud(ptrCloudWithNormal);

		/****** 曲面重建模块 ******/
		// 创建贪婪三角形投影重建对象
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		// 创建多边形网格对象，用来存储重建结果
		pcl::PolygonMesh triangles;
		// 设置参数
		gp3.setSearchRadius(25);				// 设置连接点之间最大距离，用于确定k近邻的球半径
		gp3.setMu(2.5);							// 设置最近邻距离的乘子，以得到每个点的最终搜索半径
		gp3.setMaximumNearestNeighbors(100);	// 设置搜索的最近邻点的最大数量
		gp3.setMaximumSurfaceAngle(M_PI / 2);	// 45度 最大平面角
		gp3.setMinimumAngle(M_PI / 18);			// 10度 每个三角的最大角度？
		gp3.setMaximumAngle(2 * M_PI / 3);		// 120度
		gp3.setNormalConsistency(false);		// 若法向量一致，设为true
		// 设置点云数据和搜索方式
		gp3.setInputCloud(ptrCloudWithNormal);
		gp3.setSearchMethod(tree2);
		// 开始重建
		gp3.reconstruct(triangles);
		QMessageBox::information(this, "informaiton", "Reconstruction finished");

		QString timeDiff = TimeOff();

		/****** 图形显示模块 ******/
		QMessageBox::information(this, "informaiton", "Start to show");
		viewer->addPolygonMesh(triangles, "my"); //设置要显示的网格对象
		//设置网格模型显示模式
		//viewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示
		//viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示
		viewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示

		// 输出窗口
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

	// 输出窗口
	ConsoleLog("About", "Z", "http://nightn.com", "Welcome to my blog!");
}

void MainWindow::itemSelected(QTreeWidgetItem* item, int count)
{
	count = ui->dataTree->indexOfTopLevelItem(item);  //获取item的行号

	for (int i = 0; i != m_vctCloud.size(); i++)
	{
		viewer->updatePointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + QString::number(i).toStdString());
	}

	//提取当前点云的RGB,点云数量等信息
	int nCloudSize = m_vctCloud[count].ptrCloud->points.size();
	unsigned int cloud_r = m_vctCloud[count].ptrCloud->points[0].r;
	unsigned int cloud_g = m_vctCloud[count].ptrCloud->points[0].g;
	unsigned int cloud_b = m_vctCloud[count].ptrCloud->points[0].b;

	bool bMultiColor = true;
	if (m_vctCloud[count].ptrCloud->points.begin()->r == (m_vctCloud[count].ptrCloud->points.end() - 1)->r)			//判断点云单色多色的条件（不是很严谨）
		bMultiColor = false;

	ui->propertyTable->setItem(0, 1, new QTableWidgetItem(QString::number(m_vctCloud.size())));
	ui->propertyTable->setItem(1, 1, new QTableWidgetItem(QString::number(nCloudSize)));
	ui->propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(m_nPointsNum)));
	ui->propertyTable->setItem(3, 1, new QTableWidgetItem(bMultiColor ? "Multi Color" : (QString::number(cloud_r) + " " + QString::number(cloud_g) + " " + QString::number(cloud_b))));

	//选中item所对应的点云尺寸变大
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
			// 将strCloudId所对应的点云设置成透明
			viewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.0, strCloudId, 0);

			QColor item_color = QColor(112, 122, 132, 255);
			curItem->setTextColor(0, item_color);

			m_vctCloud[id].bVisible = false;
			// 输出窗口
			ConsoleLog("Hide point clouds", "Point clouds selected", "", "");
		}
		else
		{
			viewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, strCloudId, 0);

			QColor item_color = QColor(0, 0, 0, 255);
			curItem->setTextColor(0, item_color);

			m_vctCloud[id].bVisible = true;
			// 输出窗口
			ConsoleLog("Show point clouds", "Point clouds selected", "", "");
		}
	}

	ui->qvtkWidget->update();
}

void MainWindow::Init()
{
	// 界面初始化
	setWindowIcon(QIcon(tr(":/resource/images/icon.png")));
	setWindowTitle(tr("EasyCloud"));

	//点云初始化
	m_cloud.ptrCloud.reset(new Cloud);
	m_cloud.ptrCloud->resize(1);

	m_pCloud.reset(new Cloud);
	m_pInputCloud.reset(new Cloud);

	//可视化对象初始化
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//设置VTK可视化窗口指针
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	//设置窗口交互，窗口可接受键盘等事件
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	ui->qvtkWidget->update();

	ui->propertyTable->setSelectionMode(QAbstractItemView::NoSelection);	// 禁止点击属性管理器的 item
	ui->consoleTable->setSelectionMode(QAbstractItemView::NoSelection);		// 禁止点击输出窗口的 item
	ui->dataTree->setSelectionMode(QAbstractItemView::ExtendedSelection);	// 允许dataTree进行多选

	//设置默认主题
	/*QString qss = darcula_qss;
	qApp->setStyleSheet(qss);*/

	// 数据窗口
	SetDataTable();
	// 属性管理窗口
	SetPropertyTable();
	// 输出窗口
	SetConsoleTable();

	ConsoleLog("Software start", "EasyCloud", "Welcome to use EasyCloud", "Z");
	// 设置背景颜色为 dark
	viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
	//添加坐标系
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
	// 设置输出窗口
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

	ui->consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); //设置不可编辑
	ui->consoleTable->verticalHeader()->setDefaultSectionSize(22);		   //设置行距

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
	QDateTime time = QDateTime::currentDateTime();			//获取系统现在的时间
	QString strTime = time.toString("MM-dd hh:mm:ss");		//设置显示格式
	ui->consoleTable->setItem(nRowsNum - 1, 0, new QTableWidgetItem(strTime));
	ui->consoleTable->setItem(nRowsNum - 1, 1, new QTableWidgetItem(operation));
	ui->consoleTable->setItem(nRowsNum - 1, 2, new QTableWidgetItem(subName));
	ui->consoleTable->setItem(nRowsNum - 1, 3, new QTableWidgetItem(fileName));
	ui->consoleTable->setItem(nRowsNum - 1, 4, new QTableWidgetItem(note));

	ui->consoleTable->scrollToBottom();						// 滑动自动滚到最底部
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
		// 添加到窗口
		viewer->addPointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());
		viewer->updatePointCloud(m_vctCloud[i].ptrCloud, "cloud" + QString::number(i).toStdString());

		pcl::getMinMax3D(*m_vctCloud[i].ptrCloud, m_PointMin, m_PointMax);
		m_dMaxLen = GetMaxValue(m_PointMin, m_PointMax);
	}

	//重设视角
	viewer->resetCamera();

	//刷新窗口
	ui->qvtkWidget->update();
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

	m_dMaxLen = GetMaxValue(m_PointMin, m_PointMax);

	//重设视角
	viewer->resetCamera();

	//刷新窗口
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
	int nTimeDiff = time.elapsed();		//返回从上次start()或restart()开始以来的时间差，单位ms
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

			// 1.获取滤波类型
			int nFilterType = m_cloudFilter.GetFilterType();

			// 2.统计滤波
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
			// 3.直通滤波
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

				pass.setNegative(true);		// true:保留范围外的点; false:保留范围内的点
				pass.filter(*ptrCloudFiltered);
			}

			int nFilteredPoints = ptrCloudFiltered->points.size();
			m_nPointsNum += nFilteredPoints;

			copyPointCloud(*ptrCloudFiltered, *m_vctCloud[i].ptrCloud);
		}

		QString timeDiff = TimeOff();

		// 属性窗口
		SetPropertyTable();
		//输出窗口
		ConsoleLog("CloudFilter", "All point clouds", "", "Time cost: " + timeDiff + " s, Points: " + QString::number(m_nPointsNum));
		//更新显示
		ViewerCloud(m_vctCloud);
	}
}



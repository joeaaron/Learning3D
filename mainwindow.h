/****************************************************************
// Copyright © 2022, iTech. Co., Ltd.
// File name: PCLViewer.h
// Author: Z
// Version: 1.0.0
// Date: 2012/02/18
// Description: 点云显示
// History:
// <version>    <date>	    <author>	<desc>
// 1.0.0	    2012/02/18	   Z 	    build this module
****************************************************************/

#pragma once
//设置中文编码
#pragma execution_character_set("utf-8")

// !PCL
#include <pcl/common/common.h>
#include <pcl/point_types.h>	
#include <pcl/point_cloud.h>	
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>	
#include <pcl/filters/filter.h>
#include <vtkRenderWindow.h>	

#include <QMainWindow>
#include <QFileDialog.h>

// !VTK
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

namespace Ui {
class MainWindow;
}

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
	//*****************************************************
	// Function:	 UpdateDisplay
	// FullName:	 MainWindow::UpdateDisplay
	// Description:	 点云显示
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void UpdateDisplay();

	//*****************************************************
	// Function:	 GetMaxValue
	// FullName:	 MainWindow::GetMaxValue
	// Description:	 获取长宽高最大值
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	double GetMaxValue(PointT p1, PointT p2);
    
	Ui::MainWindow *ui;

	Cloud::Ptr m_pCloud;												// 当前点云

	Cloud::Ptr m_pInputCloud;											// 输入点云

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;		//可视化窗口类

	PointT m_PointMin;													//点云坐标极值-最小

	PointT m_PointMax;													//点云坐标极值-最大

	double m_dMaxLen;															

public slots:
	//*****************************************************
	// Function:	 on_actionOpen_triggered
	// FullName:	 MainWindow::on_actionOpen_triggered
	// Description:	 打开点云文件
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionOpen_triggered();
	//*****************************************************
	// Function:	 on_actionClear_triggered
	// FullName:	 MainWindow::on_actionClear_triggered
	// Description:	 清空点云文件
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionClear_triggered();
	//*****************************************************
	// Function:	 on_actionUp_triggered
	// FullName:	 MainWindow::on_actionUp_triggered
	// Description:	 俯视图
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionUp_triggered();
	//*****************************************************
	// Function:	 on_actionBottom_triggered
	// FullName:	 MainWindow::on_actionBottom_triggered
	// Description:	 仰视图
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionBottom_triggered();
	//*****************************************************
	// Function:	 on_actionFront_triggered
	// FullName:	 MainWindow::on_actionFront_triggered
	// Description:	 前视图
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionFront_triggered();
	//*****************************************************
	// Function:	 on_actionBack_triggered
	// FullName:	 MainWindow::on_actionBack_triggered
	// Description:	 后视图
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionBack_triggered();
	//*****************************************************
	// Function:	 on_actionBack_triggered
	// FullName:	 MainWindow::on_actionBack_triggered
	// Description:	 左视图
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionLeft_triggered();
	//*****************************************************
	// Function:	 on_actionBack_triggered
	// FullName:	 MainWindow::on_actionBack_triggered
	// Description:	 右视图
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionRight_triggered();
};



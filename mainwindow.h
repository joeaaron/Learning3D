/****************************************************************
// Copyright © 2022, iTech. Co., Ltd.
// File name: PCLViewer.h
// Author: Z
// Version: 1.0.0
// Date: 2012/02/18
// Description: 点云显示
// History:
// <version>    <date>	    <author>	<desc>
// 1.0.0	    2022/02/18	   Z 	    build this module
****************************************************************/

#pragma once

// !QT
#include <QMainWindow>
#include <QFileDialog>
#include <QColorDialog>
#include <QTime>
#include <QMessageBox>
#include <QTreeWidget>

// !Own
#include "EasyCloudDef.h"
#include "AboutWin.h"
#include "filter.h"

#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
	//*****************************************************
	// Function:	 Init
	// FullName:	 MainWindow::Init
	// Description:	 初始化界面
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void Init();
	//*****************************************************
	// Function:	 SetPropertyTable
	// FullName:	 MainWindow::SetPropertyTable
	// Description:	 设置属性管理窗口
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void SetPropertyTable();
	//*****************************************************
	// Function:	 SetConsoleTable
	// FullName:	 MainWindow::SetConsoleTable
	// Description:	 设置输出窗口
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void SetConsoleTable();
	//*****************************************************
	// Function:	 ConsoleLog
	// FullName:	 MainWindow::ConsoleLog
	// Description:	 输出日志
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void ConsoleLog(QString operation, QString subName, QString fileName, QString note);
	//*****************************************************
	// Function:	 ViewerCloud
	// FullName:	 MainWindow::ViewerCloud
	// Description:	 显示点云
	// Parameters:	 const std::vector<MyCloud::CloudStructure> vctCloud
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void ViewerCloud(const std::vector<MyCloud::CloudStructure> vctCloud);
	//*****************************************************
	// Function:	 ViewerAddedCloud
	// FullName:	 MainWindow::ViewerAddedCloud
	// Description:	 添加点云到viewer,并显示点云
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void ViewerAddedCloud();
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
	// Function:	 SetCloudColor
	// FullName:	 MainWindow::SetCloudColor
	// Description:	 设置点云颜色
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void SetCloudColor(unsigned int r, unsigned int g, unsigned int b);
	//*****************************************************
	// Function:	 SetA
	// FullName:	 MainWindow::SetA
	// Description:	 设置透明
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void SetA(unsigned int a);
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
	//*****************************************************
	// Function:	 TimeStart
	// FullName:	 MainWindow::TimeStart
	// Description:	 计时开始
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void TimeStart();
	//*****************************************************
	// Function:	 TimeOff
	// FullName:	 MainWindow::TimeOff
	// Description:	 计时结束
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	QString TimeOff();
	//*****************************************************
	// Function:	 GetFileName
	// FullName:	 MainWindow::GetFileName
	// Description:	 获取文件名
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	std::string GetFileName(std::string strFileName);
	//*****************************************************
	// Function:	 SaveMultiCloud
	// FullName:	 MainWindow::SaveMultiCloud
	// Description:	 保存多个点云
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void SaveMultiCloud();

	Ui::MainWindow *ui;

	MyCloud::CloudStructure m_cloud;									// 点云对象

	std::vector<MyCloud::CloudStructure> m_vctCloud;					// 点云集合
	
	int m_nPointsNum;													// 视图中所有的点数

	Cloud::Ptr m_pCloud;												// 当前点云

	Cloud::Ptr m_pInputCloud;											// 输入点云

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;		// 可视化窗口类

	PointT m_PointMin;													// 点云坐标极值-最小

	PointT m_PointMax;													// 点云坐标极值-最大

	double m_dMaxLen;	

	bool m_bEnableConsole;												// 是否启用输出日志

	QTime time;

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloudXYZ;

	Filter m_cloudFilter;

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
	// Function:	 on_actionSave_triggered
	// FullName:	 MainWindow::on_actionSave_triggered
	// Description:	 保存点云文件
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionSave_triggered();
	//*****************************************************
	// Function:	 on_actionExit_triggered
	// FullName:	 MainWindow::on_actionExit_triggered
	// Description:	 退出软件
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionExit_triggered();
	/***** Slots of Display *****/
	//*****************************************************
	// Function:	 on_actionCloudColor_triggered
	// FullName:	 MainWindow::on_actionCloudColor_triggered
	// Description:	 改变点云颜色
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionCloudColor_triggered();
	//*****************************************************
	// Function:	 on_actionBGColor_triggered
	// FullName:	 MainWindow::on_actionBGColor_triggered
	// Description:	 改变背景颜色
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionBGColor_triggered();
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

	/****************** Slots of Process ******************/
	
	//*****************************************************
	// Function:	 on_actionSORFilter_triggered
	// FullName:	 MainWindow::on_actionSORFilter_triggered
	// Description:	 点云去噪
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionSORFilter_triggered();
	//*****************************************************
	// Function:	 on_actionDownSample_triggered
	// FullName:	 MainWindow::on_actionDownSample_triggered
	// Description:	 下采样
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionDownSample_triggered();
	//*****************************************************
	// Function:	 on_actionRansacSeg_triggered
	// FullName:	 MainWindow::on_actionRansacSeg_triggered
	// Description:	 随机采样一致性分割
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionRansacSeg_triggered();
	//*****************************************************
	// Function:	 on_actionDbScan_triggered
	// FullName:	 MainWindow::on_actionDbScan_triggered
	// Description:	 点云密度聚类
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionDbScan_triggered();
	//*****************************************************
	// Function:	 on_actionMeshSurface_triggered
	// FullName:	 MainWindow::on_actionMeshSurface_triggered
	// Description:	 以面片形式显示重建
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionMeshSurface_triggered();
	//*****************************************************
	// Function:	 on_actionWireFrame_triggered
	// FullName:	 MainWindow::on_actionWireFrame_triggered
	// Description:	 以线框图模式显示重建
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionWireFrame_triggered();

	/****************** Slots of About ******************/

	//*****************************************************
	// Function:	 on_actionAbout_triggered
	// FullName:	 MainWindow::on_actionAbout_triggered
	// Description:	 关于
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void on_actionAbout_triggered();


	/****************** Slots of ToolBar ******************/

	//*****************************************************
	// Function:	 itemSelected
	// FullName:	 MainWindow::itemSelected
	// Description:	 选中点云文件
	// Parameters:   @ QTreeWidgetItem*
	//			     @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void itemSelected(QTreeWidgetItem*, int);

	//*****************************************************
	// Function:	 StatisticalFilter
	// FullName:	 MainWindow::StatisticalFilter
	// Description:	 统计滤波
	// Parameters:   @ QTreeWidgetItem*
	//			     @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void StatisticalFilter();
	
};



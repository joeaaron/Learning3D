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
#include <QFileDialog.h>
#include <QTime>
#include <QMessageBox>

// !Own
#include "EasyCloudDef.h"

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
	// Parameters:
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void ViewerCloud();
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



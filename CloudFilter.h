#pragma once

#include "ui_filter.h"

#include "MyTitle.h"
#include "EasyCloudDef.h"

class CloudFilter : public QWidget
{
	Q_OBJECT

public:
	CloudFilter(QWidget*parents = 0);

	~CloudFilter();

public:
	double GetMeanKVal() const;
	double GetStdVal() const;

private:
	void initTitleBar();

	Ui::Form ui;
	MyTitleBar* m_titleBar;

	double m_dMeanK;		// 邻域点数
	double m_dStdDev;		// 标准偏差
signals:
	// 按钮触发的信号;
	void runBtnClicked();

public slots:
	//*****************************************************
	// Function:	 filterMethodChanged
	// FullName:	 CloudFilter::filterMethodChanged
	// Description:	 切换滤波方法
	// Parameters:   @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void filterMethodChanged(int);

	//*****************************************************
	// Function:	 meanKChanged
	// FullName:	 CloudFilter::meanKChanged
	// Description:	 邻域点数修改
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void meanKChanged(QString);

	//*****************************************************
	// Function:	 stdDevChanged
	// FullName:	 CloudFilter::stdDevChanged
	// Description:	 标准偏差修改
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void stdDevChanged(QString);

	//*****************************************************
	// Function:	 onButtonCloseClicked
	// FullName:	 CloudFilter::onButtonCloseClicked
	// Description:	 关闭
	// Parameters:   
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void onButtonCloseClicked(); 

	//*****************************************************
	// Function:	 onButtonRunClicked
	// FullName:	 CloudFilter::onButtonRunClicked
	// Description:	 执行
	// Parameters:   
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void onButtonRunClicked();
};


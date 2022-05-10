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
	// 获取滤波类型
	int GetFilterType() const;
	// 获取邻域点数
	double GetMeanKVal() const;
	// 获取标准差
	double GetStdVal() const;
	// 获取滤波方向
	QString GetFieldName() const;
	// 获取滤波下限
	double GetMinLimit() const;
	// 获取滤波上限
	double GetMaxLimit() const;
private:
	void initTitleBar();
	void initParam();

	Ui::Form ui;
	MyTitleBar* m_titleBar;

	int m_nFilterType;		// 滤波类型
	double m_dMeanK;		// 邻域点数
	double m_dStdDev;		// 标准偏差

	QString m_strFieldName; // 滤波方向
	double m_dMinLimit;		// 滤波下限
	double m_dMaxLimit;		// 滤波上限

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
	// Description:	 统计滤波-邻域点数修改
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void meanKChanged(QString);

	//*****************************************************
	// Function:	 stdDevChanged
	// FullName:	 CloudFilter::stdDevChanged
	// Description:	 统计滤波-标准偏差修改
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void stdDevChanged(QString);

	//*****************************************************
	// Function:	 fieldNameChanged
	// FullName:	 CloudFilter::fieldNameChanged
	// Description:	 直通滤波-滤波方向设置
	// Parameters:   @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void fieldNameChanged(int);

	//*****************************************************
	// Function:	 maxLimitChanged
	// FullName:	 CloudFilter::maxLimitChanged
	// Description:	 统计滤波-上限修改
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void maxLimitChanged(QString);

	//*****************************************************
	// Function:	 minLimitChanged
	// FullName:	 CloudFilter::minLimitChanged
	// Description:	 统计滤波-下限修改
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void minLimitChanged(QString);

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


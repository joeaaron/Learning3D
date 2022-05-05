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

	double m_dMeanK;		// �������
	double m_dStdDev;		// ��׼ƫ��
signals:
	// ��ť�������ź�;
	void runBtnClicked();

public slots:
	//*****************************************************
	// Function:	 filterMethodChanged
	// FullName:	 CloudFilter::filterMethodChanged
	// Description:	 �л��˲�����
	// Parameters:   @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void filterMethodChanged(int);

	//*****************************************************
	// Function:	 meanKChanged
	// FullName:	 CloudFilter::meanKChanged
	// Description:	 ��������޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void meanKChanged(QString);

	//*****************************************************
	// Function:	 stdDevChanged
	// FullName:	 CloudFilter::stdDevChanged
	// Description:	 ��׼ƫ���޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void stdDevChanged(QString);

	//*****************************************************
	// Function:	 onButtonCloseClicked
	// FullName:	 CloudFilter::onButtonCloseClicked
	// Description:	 �ر�
	// Parameters:   
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void onButtonCloseClicked(); 

	//*****************************************************
	// Function:	 onButtonRunClicked
	// FullName:	 CloudFilter::onButtonRunClicked
	// Description:	 ִ��
	// Parameters:   
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void onButtonRunClicked();
};


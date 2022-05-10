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
	// ��ȡ�˲�����
	int GetFilterType() const;
	// ��ȡ�������
	double GetMeanKVal() const;
	// ��ȡ��׼��
	double GetStdVal() const;
	// ��ȡ�˲�����
	QString GetFieldName() const;
	// ��ȡ�˲�����
	double GetMinLimit() const;
	// ��ȡ�˲�����
	double GetMaxLimit() const;
private:
	void initTitleBar();
	void initParam();

	Ui::Form ui;
	MyTitleBar* m_titleBar;

	int m_nFilterType;		// �˲�����
	double m_dMeanK;		// �������
	double m_dStdDev;		// ��׼ƫ��

	QString m_strFieldName; // �˲�����
	double m_dMinLimit;		// �˲�����
	double m_dMaxLimit;		// �˲�����

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
	// Description:	 ͳ���˲�-��������޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void meanKChanged(QString);

	//*****************************************************
	// Function:	 stdDevChanged
	// FullName:	 CloudFilter::stdDevChanged
	// Description:	 ͳ���˲�-��׼ƫ���޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void stdDevChanged(QString);

	//*****************************************************
	// Function:	 fieldNameChanged
	// FullName:	 CloudFilter::fieldNameChanged
	// Description:	 ֱͨ�˲�-�˲���������
	// Parameters:   @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void fieldNameChanged(int);

	//*****************************************************
	// Function:	 maxLimitChanged
	// FullName:	 CloudFilter::maxLimitChanged
	// Description:	 ͳ���˲�-�����޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void maxLimitChanged(QString);

	//*****************************************************
	// Function:	 minLimitChanged
	// FullName:	 CloudFilter::minLimitChanged
	// Description:	 ͳ���˲�-�����޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void minLimitChanged(QString);

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


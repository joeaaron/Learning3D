#pragma once

#include "ui_filter.h"

#include "MyTitle.h"
#include "EasyCloudDef.h"

class Filter : public QWidget
{
	Q_OBJECT

public:
	Filter(QWidget*parents = 0);

	~Filter();

private:
	void initTitleBar();

	Ui::Form ui;
	MyTitleBar* m_titleBar;

public slots:
	//*****************************************************
	// Function:	 filterMethodChanged
	// FullName:	 Filter::filterMethodChanged
	// Description:	 �л��˲�����
	// Parameters:   @ int
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void filterMethodChanged(int);

	//*****************************************************
	// Function:	 meanKChanged
	// FullName:	 Filter::meanKChanged
	// Description:	 ��������޸�
	// Parameters:   @ QString
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void meanKChanged(QString);

	//*****************************************************
	// Function:	 onButtonCloseClicked
	// FullName:	 Filter::onButtonCloseClicked
	// Description:	 �ر�
	// Parameters:   
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void onButtonCloseClicked(); 

	//*****************************************************
	// Function:	 onButtonRunClicked
	// FullName:	 Filter::onButtonRunClicked
	// Description:	 ִ��
	// Parameters:   
	// Return value: 
	// Remarks:		 
	//				 
	//*****************************************************
	void onButtonRunClicked();
};


#include "CloudFilter.h"
#include "mainwindow.h"

CloudFilter::CloudFilter(QWidget* parents) :
	m_titleBar(nullptr)
{
	ui.setupUi(this);
	// 无标题栏
	this->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowMinimizeButtonHint);

	//setAttribute(Qt::WA_TranslucentBackground);
	// 重绘标题栏;
	initTitleBar();

	// 初始化参数
	initParam();
};

CloudFilter::~CloudFilter()
{
	if (m_titleBar != nullptr)
	{
		m_titleBar = nullptr;
		delete m_titleBar;
	}

}

void CloudFilter::filterMethodChanged(int nIdx)
{
	nIdx = ui.comboBox->currentIndex();
	if (-1 == nIdx)
	{
		return;
	}

	if (0 == nIdx)
	{
		ui.stackedWidget->setCurrentWidget(ui.page);
	}
	else
	{
		ui.stackedWidget->setCurrentWidget(ui.page_2);
	}

	m_nFilterType = nIdx;
}

void CloudFilter::meanKChanged(QString strMeanK)
{
	if (strMeanK.isEmpty())
	{
		return;
	}

	QDoubleValidator* v = new QDoubleValidator(0, 9999, 3, this);  //[0,9999]小数点后三位
	v->setNotation(QDoubleValidator::StandardNotation);

	ui.meanK->setValidator(v);
	ui.meanK->setText(strMeanK);

	m_dMeanK = strMeanK.toDouble();
}

void CloudFilter::stdDevChanged(QString strStdDev)
{
	if (strStdDev.isEmpty())
	{
		return;
	}

	QDoubleValidator* v = new QDoubleValidator(0, 9999, 3, this);  //[0,9999]小数点后三位
	v->setNotation(QDoubleValidator::StandardNotation);

	ui.stddev->setValidator(v);
	ui.stddev->setText(strStdDev);

	m_dStdDev = strStdDev.toDouble();
}

void CloudFilter::fieldNameChanged(int nIdx)
{
	nIdx = ui.comboBox_2->currentIndex();
	if (-1 == nIdx)
	{
		return;
	}

	if (0 == nIdx)
	{
		m_strFieldName = "x";
	}
	else if (1 == nIdx)
	{
		m_strFieldName = "y";
	}
	else
	{
		m_strFieldName = "z";
	}
}

void CloudFilter::minLimitChanged(QString strMinLimit)
{
	if (strMinLimit.isEmpty())
	{
		return;
	}

	QDoubleValidator* v = new QDoubleValidator(0, 9999, 3, this);  //[0,9999]小数点后三位
	v->setNotation(QDoubleValidator::StandardNotation);

	ui.stddev->setValidator(v);
	ui.stddev->setText(strMinLimit);

	m_dMinLimit = strMinLimit.toDouble();
}

void CloudFilter::maxLimitChanged(QString strMaxLimit)
{
	if (strMaxLimit.isEmpty())
	{
		return;
	}

	QDoubleValidator* v = new QDoubleValidator(0, 9999, 3, this);  //[0,9999]小数点后三位
	v->setNotation(QDoubleValidator::StandardNotation);

	ui.stddev->setValidator(v);
	ui.stddev->setText(strMaxLimit);

	m_dMaxLimit = strMaxLimit.toDouble();
}

void CloudFilter::initTitleBar()
{
	m_titleBar = new MyTitleBar(this);
	m_titleBar->move(0, 0);

	m_titleBar->setButtonType(ButtonType::RUN_BUTTON);
	m_titleBar->setTitleContent("点云滤波");

	connect(m_titleBar, SIGNAL(signalButtonCloseClicked()), this, SLOT(onButtonCloseClicked()));
	connect(m_titleBar, SIGNAL(signalButtonRunClicked()), this, SLOT(onButtonRunClicked()));
}

void CloudFilter::initParam()
{
	// 初始化滤波类型
	m_nFilterType = ui.comboBox->currentIndex();
	if (0 == m_nFilterType)
	{
		ui.stackedWidget->setCurrentWidget(ui.page);
	}
	else
	{
		ui.stackedWidget->setCurrentWidget(ui.page_2);
	}

	// 初始化邻域点数和标准偏差
	m_dMeanK = ui.meanK->text().toDouble();
	m_dStdDev = ui.stddev->text().toDouble();

	// 初始化滤波上下限
	m_dMinLimit = ui.minLimit->text().toDouble();
	m_dMaxLimit = ui.maxLimit->text().toDouble();
}

void CloudFilter::onButtonRunClicked()
{
	emit runBtnClicked();
}

void CloudFilter::onButtonCloseClicked()
{
	close();
}

int CloudFilter::GetFilterType() const 
{
	return m_nFilterType;
}

double CloudFilter::GetMeanKVal() const
{
	return m_dMeanK;
}

double CloudFilter::GetStdVal() const
{
	return m_dStdDev;
}

QString CloudFilter::GetFieldName() const
{
	return m_strFieldName;
}

double CloudFilter::GetMinLimit() const
{
	return m_dMinLimit;
}

double CloudFilter::GetMaxLimit() const
{
	return m_dMaxLimit;
}
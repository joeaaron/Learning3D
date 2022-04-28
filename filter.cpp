#include "filter.h"
#include <windows.h>

Filter::Filter(QWidget* parents)
{
	ui.setupUi(this);

	this->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowMinimizeButtonHint);

	//setAttribute(Qt::WA_TranslucentBackground);
	// 初始化标题栏;
	initTitleBar();
};

Filter::~Filter()
{
}

void Filter::filterMethodChanged(int nIdx)
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
}

void Filter::meanKChanged(QString strMeanK)
{
	if (strMeanK.isEmpty())
	{
		return;
	}

	QDoubleValidator* v = new QDoubleValidator(0, 9999, 3, this);  //[0,9999]小数点后三位
	v->setNotation(QDoubleValidator::StandardNotation);

	ui.meanK->setValidator(v);
	ui.meanK->setText(strMeanK);
}

void Filter::initTitleBar()
{
	m_titleBar = new MyTitleBar(this);
	m_titleBar->move(0, 0);

	m_titleBar->setButtonType(ButtonType::RUN_BUTTON);
	m_titleBar->setTitleContent("点云滤波");

	connect(m_titleBar, SIGNAL(signalButtonCloseClicked()), this, SLOT(onButtonCloseClicked()));
	connect(m_titleBar, SIGNAL(signalButtonRunClicked()), this, SLOT(onButtonRunClicked()));
}

void Filter::onButtonRunClicked()
{
	std::cout << "执行滤波";
}

void Filter::onButtonCloseClicked()
{
	close();
}


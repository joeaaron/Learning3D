#pragma once

#include "ui_filter.h"

class Filter : public QWidget
{
	Q_OBJECT

public:
	Filter(QWidget*parents = 0) 
	{
		ui.setupUi(this);
	};

	~Filter()
	{}


private:
	Ui::Form ui;
};


#pragma once
#include "ui_MainWindow.h"
#include <qmainwindow.h>
#include <qpushbutton.h>
#include <iostream>
#include <qfiledialog.h>
#include <qfile.h>

namespace MeasureMainForm {
	class MainForm :public QMainWindow
	{
		Q_OBJECT
	public:
		MainForm(QWidget* parent = nullptr);
		~MainForm();

	private:
		Ui::MeasureMainWindow *ui;
	};
}

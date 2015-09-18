/*+-------------------------------------------------------------------------+
|                             OpenBeam                                    |
|                                                                         |
| Copyright (C) 2015  Jose Luis Blanco Claraco                            |
| Distributed under GNU General Public License version 3                  |
|   See <http://www.gnu.org/licenses/>                                    |
+-------------------------------------------------------------------------+  */

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	QString sPath = ".";
	dirmodel  = new QFileSystemModel(this);

	dirmodel->setRootPath(sPath);
	ui->tvBrowserDirs->setModel(dirmodel);
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::on_tvBrowserDirs_clicked(const QModelIndex &index)
{
	const bool is_file  = dirmodel->fileInfo(index).isFile();
	const QString sPath = dirmodel->fileInfo(index).absoluteFilePath();




}

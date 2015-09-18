/*+-------------------------------------------------------------------------+
  |                             OpenBeam                                    |
  |                                                                         |
  | Copyright (C) 2015  Jose Luis Blanco Claraco                            |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileSystemModel>

#include <openbeam/openbeam.h>
#include <deque>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void on_tvBrowserDirs_clicked(const QModelIndex &index);

private:
	Ui::MainWindow *ui;
	QFileSystemModel *dirmodel;

};

#endif // MAINWINDOW_H

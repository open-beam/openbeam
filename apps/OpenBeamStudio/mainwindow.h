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
#include <QSettings>

#include "highlighter.h"

#include <openbeam/openbeam.h>

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
	void on_action_Open_model_triggered();

private:
	Ui::MainWindow *ui;
	QSettings m_app_setting;

	QFileSystemModel *m_dirmodel;
	Highlighter * m_highlighter;

	void setupTreeView();
	void setupEditor();


	void loadOpenBeamFile(const QString &sPath);
};

#endif // MAINWINDOW_H

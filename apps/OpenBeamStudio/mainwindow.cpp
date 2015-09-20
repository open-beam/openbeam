/*+-------------------------------------------------------------------------+
|                             OpenBeam                                    |
|                                                                         |
| Copyright (C) 2015  Jose Luis Blanco Claraco                            |
| Distributed under GNU General Public License version 3                  |
|   See <http://www.gnu.org/licenses/>                                    |
+-------------------------------------------------------------------------+  */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	m_app_setting("openbeam","OpenBeamStudio")
{
	ui->setupUi(this);
	setupTreeView();
	setupEditor();
}

MainWindow::~MainWindow()
{
	delete ui;
}

// Dir tree view:
void MainWindow::setupTreeView()
{
	QString sPath = m_app_setting.value("app/default_dir",".").toString();  // Default path
	m_dirmodel = new QFileSystemModel(this);

	QStringList filters;
	filters << "*.txt";
	filters << "*.ob";
	m_dirmodel->setNameFilters(filters);
	m_dirmodel->setNameFilterDisables(true);

	m_dirmodel->setRootPath(sPath);
	ui->tvBrowserDirs->setModel(m_dirmodel);
}

// Prepare syntax highlighter editor
void MainWindow::setupEditor()
{
	QFont font;
	font.setFamily("Courier");
	font.setFixedPitch(true);
	font.setPointSize(10);
	ui->edCodeEditor->setFont(font);
	m_highlighter = new Highlighter(ui->edCodeEditor->document());
}


void MainWindow::on_tvBrowserDirs_clicked(const QModelIndex &index)
{
	const bool is_file = m_dirmodel->fileInfo(index).isFile();
	const QString sPath = m_dirmodel->fileInfo(index).absoluteFilePath();
	
	if (is_file && !sPath.isEmpty())
		this->loadOpenBeamFile(sPath);
}

void MainWindow::on_action_Open_model_triggered()
{
	QString fileName = QFileDialog::getOpenFileName(
		this,
		tr("Load model file"), 
		m_app_setting.value("app/default_dir", ".").toString(),  // Default path
		tr("OpenBeam files (*.ob *.txt *.TXT *.OB)")
		);
	if (!fileName.isEmpty())
		this->loadOpenBeamFile(fileName);
}


void MainWindow::loadOpenBeamFile(const QString &sPath)
{
	QFile file(sPath);
	if (file.open(QFile::ReadOnly | QFile::Text))
	{
		// Read:
		ui->edCodeEditor->setPlainText(file.readAll());
		// Save default dir for future sessions:
		m_app_setting.setValue("app/default_dir", QFileInfo(sPath).absolutePath() );
	}
	else {
		ui->edCodeEditor->clear();

		QMessageBox::warning(this, tr("OpenBeamStudio"),
			tr("Cannot open the file!"),
			QMessageBox::Ok);
	}
}

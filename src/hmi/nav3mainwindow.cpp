#include "nav3mainwindow.h"
#include "ui_nav3mainwindow.h"


#include <QVBoxLayout>

Nav3MainWindow::Nav3MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Nav3MainWindow)
{
    // setup ui
    ui->setupUi(this);


    // setup scene manager
    RenderWin = new Scene_Manager;
    //RenderWin->createScene();

    //ui->centralWidget->layout()->addWidget(RenderWin);
    //ui->centralWidget->layout()->activate();


    //ui->frame->layout()->addWidget(RenderWin);
    //ui->frame->layout()->activate();

    //ui->tab->layout()->addWidget(RenderWin);

}

Nav3MainWindow::~Nav3MainWindow()
{
    delete ui;
}

void Nav3MainWindow::on_actionExit_triggered()
{
    delete ui;
    exit(0);
}

void Nav3MainWindow::on_actionShow_ogre_triggered()
{


    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(RenderWin);
    ui->frame->setLayout(layout);

    //RenderWin->go();
}

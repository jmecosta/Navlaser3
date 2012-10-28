#ifndef NAV3MAINWINDOW_H
#define NAV3MAINWINDOW_H

#include <QMainWindow>

#include "ui_nav3mainwindow.h"

#include "scene_manager.h"

namespace Ui {
    class Nav3MainWindow;
}

class Nav3MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Nav3MainWindow(QWidget *parent = 0);
    ~Nav3MainWindow();

private slots:
    void on_actionExit_triggered();

    void on_actionShow_ogre_triggered();

private:
    Ui::Nav3MainWindow *ui;

    Scene_Manager * RenderWin;
};

#endif // NAV3MAINWINDOW_H

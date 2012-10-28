/********************************************************************************
** Form generated from reading UI file 'nav3mainwindow.ui'
**
** Created: Sun Oct 28 18:34:57 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NAV3MAINWINDOW_H
#define UI_NAV3MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QListWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Nav3MainWindow
{
public:
    QAction *actionExit;
    QAction *actionShow_ogre;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QListWidget *listWidget;
    QFrame *frame;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuEdit;
    QMenu *menuWindow;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Nav3MainWindow)
    {
        if (Nav3MainWindow->objectName().isEmpty())
            Nav3MainWindow->setObjectName(QString::fromUtf8("Nav3MainWindow"));
        Nav3MainWindow->resize(627, 285);
        actionExit = new QAction(Nav3MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionShow_ogre = new QAction(Nav3MainWindow);
        actionShow_ogre->setObjectName(QString::fromUtf8("actionShow_ogre"));
        centralWidget = new QWidget(Nav3MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        listWidget = new QListWidget(centralWidget);
        new QListWidgetItem(listWidget);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        listWidget->setMinimumSize(QSize(111, 0));
        listWidget->setMaximumSize(QSize(111, 16777215));

        horizontalLayout->addWidget(listWidget);

        frame = new QFrame(centralWidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setLayoutDirection(Qt::LeftToRight);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        listWidget->raise();
        listWidget->raise();

        horizontalLayout->addWidget(frame);

        Nav3MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Nav3MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 627, 25));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuEdit = new QMenu(menuBar);
        menuEdit->setObjectName(QString::fromUtf8("menuEdit"));
        menuWindow = new QMenu(menuBar);
        menuWindow->setObjectName(QString::fromUtf8("menuWindow"));
        Nav3MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Nav3MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        Nav3MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Nav3MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        Nav3MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuEdit->menuAction());
        menuBar->addAction(menuWindow->menuAction());
        menuFile->addAction(actionExit);
        menuWindow->addAction(actionShow_ogre);

        retranslateUi(Nav3MainWindow);

        QMetaObject::connectSlotsByName(Nav3MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *Nav3MainWindow)
    {
        Nav3MainWindow->setWindowTitle(QApplication::translate("Nav3MainWindow", "Nav3MainWindow", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("Nav3MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionShow_ogre->setText(QApplication::translate("Nav3MainWindow", "show ogre", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled = listWidget->isSortingEnabled();
        listWidget->setSortingEnabled(false);
        QListWidgetItem *___qlistwidgetitem = listWidget->item(0);
        ___qlistwidgetitem->setText(QApplication::translate("Nav3MainWindow", "Item 1", 0, QApplication::UnicodeUTF8));
        listWidget->setSortingEnabled(__sortingEnabled);

        menuFile->setTitle(QApplication::translate("Nav3MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuEdit->setTitle(QApplication::translate("Nav3MainWindow", "Edit", 0, QApplication::UnicodeUTF8));
        menuWindow->setTitle(QApplication::translate("Nav3MainWindow", "window", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Nav3MainWindow: public Ui_Nav3MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NAV3MAINWINDOW_H

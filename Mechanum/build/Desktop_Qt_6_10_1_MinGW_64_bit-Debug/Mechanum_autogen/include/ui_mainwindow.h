/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.10.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTextBrowser *WizfiLOG;
    QPushButton *btnConnectNucleo;
    QComboBox *portcomboBox;
    QLabel *label_8;
    QTextBrowser *UART_LOG;
    QLabel *label_9;
    QLineEdit *txtReceived;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QFormLayout *formLayout;
    QLineEdit *txtCntrlStatus;
    QLCDNumber *LCDLeftX;
    QLCDNumber *LCDLeftY;
    QLCDNumber *LCDRightX;
    QLCDNumber *LCDRightY;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout;
    QLabel *label_7;
    QPushButton *btnConnectWizfi;
    QLineEdit *txtPort;
    QSpacerItem *horizontalSpacer;
    QLineEdit *txtIPAdr;
    QLabel *label_6;
    QLCDNumber *LCD_vx;
    QLCDNumber *LCD_vy;
    QLCDNumber *LCD_phi;
    QLabel *label_10;
    QLineEdit *txtOutMsg;
    QGraphicsView *graphicsView;
    QLabel *label_11;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QLCDNumber *LCD_N1;
    QLCDNumber *LCD_N2;
    QLCDNumber *LCD_N3;
    QLCDNumber *LCD_N4;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1500, 750);
        QFont font;
        font.setBold(false);
        MainWindow->setFont(font);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        WizfiLOG = new QTextBrowser(centralwidget);
        WizfiLOG->setObjectName("WizfiLOG");
        WizfiLOG->setGeometry(QRect(40, 340, 251, 271));
        btnConnectNucleo = new QPushButton(centralwidget);
        btnConnectNucleo->setObjectName("btnConnectNucleo");
        btnConnectNucleo->setGeometry(QRect(480, 18, 91, 31));
        portcomboBox = new QComboBox(centralwidget);
        portcomboBox->setObjectName("portcomboBox");
        portcomboBox->setGeometry(QRect(320, 20, 151, 28));
        label_8 = new QLabel(centralwidget);
        label_8->setObjectName("label_8");
        label_8->setGeometry(QRect(320, 60, 191, 20));
        UART_LOG = new QTextBrowser(centralwidget);
        UART_LOG->setObjectName("UART_LOG");
        UART_LOG->setGeometry(QRect(320, 80, 251, 192));
        label_9 = new QLabel(centralwidget);
        label_9->setObjectName("label_9");
        label_9->setGeometry(QRect(40, 320, 191, 20));
        txtReceived = new QLineEdit(centralwidget);
        txtReceived->setObjectName("txtReceived");
        txtReceived->setGeometry(QRect(360, 310, 113, 28));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName("layoutWidget");
        layoutWidget->setGeometry(QRect(40, 20, 253, 152));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        label = new QLabel(layoutWidget);
        label->setObjectName("label");

        verticalLayout_2->addWidget(label);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName("label_2");

        verticalLayout_2->addWidget(label_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName("label_3");

        verticalLayout_2->addWidget(label_3);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName("label_4");

        verticalLayout_2->addWidget(label_4);

        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName("label_5");

        verticalLayout_2->addWidget(label_5);


        horizontalLayout->addLayout(verticalLayout_2);

        formLayout = new QFormLayout();
        formLayout->setObjectName("formLayout");
        txtCntrlStatus = new QLineEdit(layoutWidget);
        txtCntrlStatus->setObjectName("txtCntrlStatus");

        formLayout->setWidget(0, QFormLayout::ItemRole::LabelRole, txtCntrlStatus);

        LCDLeftX = new QLCDNumber(layoutWidget);
        LCDLeftX->setObjectName("LCDLeftX");
        QFont font1;
        font1.setBold(true);
        LCDLeftX->setFont(font1);
        LCDLeftX->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCDLeftX->setDigitCount(6);
        LCDLeftX->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        formLayout->setWidget(1, QFormLayout::ItemRole::LabelRole, LCDLeftX);

        LCDLeftY = new QLCDNumber(layoutWidget);
        LCDLeftY->setObjectName("LCDLeftY");
        LCDLeftY->setFont(font1);
        LCDLeftY->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCDLeftY->setDigitCount(6);
        LCDLeftY->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        formLayout->setWidget(2, QFormLayout::ItemRole::LabelRole, LCDLeftY);

        LCDRightX = new QLCDNumber(layoutWidget);
        LCDRightX->setObjectName("LCDRightX");
        LCDRightX->setFont(font1);
        LCDRightX->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCDRightX->setDigitCount(6);
        LCDRightX->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        formLayout->setWidget(3, QFormLayout::ItemRole::LabelRole, LCDRightX);

        LCDRightY = new QLCDNumber(layoutWidget);
        LCDRightY->setObjectName("LCDRightY");
        LCDRightY->setFont(font1);
        LCDRightY->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCDRightY->setDigitCount(6);
        LCDRightY->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        formLayout->setWidget(4, QFormLayout::ItemRole::LabelRole, LCDRightY);


        horizontalLayout->addLayout(formLayout);

        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName("layoutWidget1");
        layoutWidget1->setGeometry(QRect(40, 180, 251, 128));
        gridLayout = new QGridLayout(layoutWidget1);
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_7 = new QLabel(layoutWidget1);
        label_7->setObjectName("label_7");

        gridLayout->addWidget(label_7, 2, 0, 1, 1);

        btnConnectWizfi = new QPushButton(layoutWidget1);
        btnConnectWizfi->setObjectName("btnConnectWizfi");

        gridLayout->addWidget(btnConnectWizfi, 3, 0, 1, 3);

        txtPort = new QLineEdit(layoutWidget1);
        txtPort->setObjectName("txtPort");

        gridLayout->addWidget(txtPort, 2, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(110, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        gridLayout->addItem(horizontalSpacer, 2, 1, 1, 1);

        txtIPAdr = new QLineEdit(layoutWidget1);
        txtIPAdr->setObjectName("txtIPAdr");

        gridLayout->addWidget(txtIPAdr, 0, 1, 1, 2);

        label_6 = new QLabel(layoutWidget1);
        label_6->setObjectName("label_6");
        label_6->setMinimumSize(QSize(80, 0));

        gridLayout->addWidget(label_6, 0, 0, 1, 1);

        LCD_vx = new QLCDNumber(centralwidget);
        LCD_vx->setObjectName("LCD_vx");
        LCD_vx->setGeometry(QRect(310, 370, 73, 23));
        LCD_vx->setFont(font1);
        LCD_vx->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_vx->setSmallDecimalPoint(false);
        LCD_vx->setDigitCount(6);
        LCD_vx->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);
        LCD_vy = new QLCDNumber(centralwidget);
        LCD_vy->setObjectName("LCD_vy");
        LCD_vy->setGeometry(QRect(310, 400, 73, 23));
        LCD_vy->setFont(font1);
        LCD_vy->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_vy->setDigitCount(6);
        LCD_vy->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);
        LCD_phi = new QLCDNumber(centralwidget);
        LCD_phi->setObjectName("LCD_phi");
        LCD_phi->setGeometry(QRect(310, 430, 73, 23));
        LCD_phi->setFont(font1);
        LCD_phi->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_phi->setDigitCount(6);
        LCD_phi->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);
        label_10 = new QLabel(centralwidget);
        label_10->setObjectName("label_10");
        label_10->setGeometry(QRect(470, 390, 121, 20));
        txtOutMsg = new QLineEdit(centralwidget);
        txtOutMsg->setObjectName("txtOutMsg");
        txtOutMsg->setGeometry(QRect(470, 410, 191, 28));
        graphicsView = new QGraphicsView(centralwidget);
        graphicsView->setObjectName("graphicsView");
        graphicsView->setGeometry(QRect(670, 50, 801, 641));
        label_11 = new QLabel(centralwidget);
        label_11->setObjectName("label_11");
        label_11->setGeometry(QRect(310, 490, 201, 20));
        widget = new QWidget(centralwidget);
        widget->setObjectName("widget");
        widget->setGeometry(QRect(310, 510, 75, 115));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        LCD_N1 = new QLCDNumber(widget);
        LCD_N1->setObjectName("LCD_N1");
        LCD_N1->setFont(font1);
        LCD_N1->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_N1->setDigitCount(7);
        LCD_N1->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        verticalLayout->addWidget(LCD_N1);

        LCD_N2 = new QLCDNumber(widget);
        LCD_N2->setObjectName("LCD_N2");
        LCD_N2->setFont(font1);
        LCD_N2->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_N2->setDigitCount(7);
        LCD_N2->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        verticalLayout->addWidget(LCD_N2);

        LCD_N3 = new QLCDNumber(widget);
        LCD_N3->setObjectName("LCD_N3");
        LCD_N3->setFont(font1);
        LCD_N3->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_N3->setDigitCount(7);
        LCD_N3->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        verticalLayout->addWidget(LCD_N3);

        LCD_N4 = new QLCDNumber(widget);
        LCD_N4->setObjectName("LCD_N4");
        LCD_N4->setFont(font1);
        LCD_N4->setStyleSheet(QString::fromUtf8("color: rgb(0, 170, 255);\n"
"background-color: rgb(0, 0, 65);"));
        LCD_N4->setDigitCount(7);
        LCD_N4->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        verticalLayout->addWidget(LCD_N4);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1500, 25));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Mecanum App", nullptr));
        btnConnectNucleo->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "UART VCP comm log", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "Wizfi comm log", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Controller status", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Forward speed", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Left Stick Y", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Right Stick X", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Right Stick Y", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "Port:", nullptr));
        btnConnectWizfi->setText(QCoreApplication::translate("MainWindow", "Connect to Wizfi", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "IP Adress", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "Kik\303\274ld\303\266tt String:", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "Fordulatsz\303\241mok kerekenk\303\251nt", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

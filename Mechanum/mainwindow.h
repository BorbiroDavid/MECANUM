#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <SDL3/SDL.h>
#include <QTcpSocket>
#include <QHostAddress>
#include <QSerialPort>
#include <QSerialPortInfo>


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void loopSDL();
    void SendVCP();
    void SendWizfi();
    void on_btnConnectWizfi_clicked();
    void on_btnConnectNucleo_clicked();

    void readSerialData();
    void onSocketConnected();
    void onSocketError(QAbstractSocket::SocketError socketError);

private:
    Ui::MainWindow *ui;
    void findController();
    QSerialPort *serialPort;
    QByteArray readBuffer;

    //VCP beállításai
    struct SerialConfig {
        qint32 baudRate = QSerialPort::Baud115200;
        QSerialPort::DataBits dataBits = QSerialPort::Data8;
        QSerialPort::Parity parity = QSerialPort::NoParity;
        QSerialPort::StopBits stopBits = QSerialPort::OneStop;
        QSerialPort::FlowControl flowControl = QSerialPort::NoFlowControl;
        int maxBufferSize = 4096;
    } config;

    //Nucleo-F401RE board VID-je
    static const quint16 STM32_VID = 0x0483;


    uint16_t dcmCtrlSetp = 0;
    uint16_t motorSpeed = 0;
    uint16_t motorExtAngle = 0;
    uint16_t motorActVal = 0;
    uint16_t remTstamp = 0;
    QString rx_demo = "";


    // SDL Variables
    SDL_Gamepad *m_gamepad = nullptr;   // The controller object (Pointer)
    SDL_JoystickID m_gamepadId = 0;     // The controller ID (Number)

    // Qt Variables
    QTimer *m_pollTimer;
    QTimer *ControlTimer;
    QTimer *SendDataTimer;


    //Wizfi
    QTcpSocket *socket; // socket object

    // UART Függvények
    void populatePortList();
    void parseData(const QByteArray &data);

};
#endif // MAINWINDOW_H

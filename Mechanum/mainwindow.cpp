#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QScrollBar>
#include <QLCDNumber>
//#include <stdio.h>
#include <math.h>


 //Mechanum modell geometriai állandói
#define mech_lx 0.0875
#define mech_ly 0.125
#define mech_R 0.054

//Bemenetek skálázó faktorai reális sebességekért
#define vx_scaler 10000.0
#define vy_scaler 10000.0
#define w_scaler 10000.0

#define RPS_Factor 13440000 // Scale factor between rps and period

//Valós sebességek [m/s]
float vx = 0;
float vy = 0;
float w_z = 0;
float phi = 0;
float phi_ref = 0;

//Lokális fordulatszámok [1/s]
float w1 = 0;
float w2 = 0;
float w3 = 0;
float w4 = 0;

//Skálázott fordulatszámok
Sint16 N1 = 0;
Sint16 N2 = 0;
Sint16 N3 = 0;
Sint16 N4 = 0;

Sint16 Wizfi_Active = 0;

QString message;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(new QSerialPort(this))
{
    ui->setupUi(this);

    // Kezdeti állapot beállítása
    ui->btnConnectWizfi->setText("Csatlakozás");

    populatePortList();

    // A QSerialPort readyRead jele szól nekünk, ha új adat érkezett az USB-n
    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    // Initialize SDL
    SDL_Init(SDL_INIT_GAMEPAD);

    // Timer to poll SDL events every 16ms (~60 FPS)
    m_pollTimer = new QTimer(this);
    connect(m_pollTimer, &QTimer::timeout, this, &MainWindow::loopSDL);
    m_pollTimer->start(16);

    //Timer to send on VCP port
    SendDataTimer = new QTimer(this);
    connect(SendDataTimer, &QTimer::timeout, this, &MainWindow::SendVCP);
    SendDataTimer->start(50);

    socket = (new QTcpSocket(this)); // Initialize the socket

    // Connect socket signals to custom slots
    connect(socket, &QTcpSocket::connected, this, &MainWindow::onSocketConnected);
    //connect(socket, &QTcpSocket::readyRead, this, &MainWindow::onSocketReadyRead);
    // Errorhandling
    connect(socket, &QTcpSocket::errorOccurred, this, &MainWindow::onSocketError);
}

MainWindow::~MainWindow()
{

    if (serialPort->isOpen()) {
        serialPort->close();
    }
    delete ui;

    if (m_gamepad) {
        SDL_CloseGamepad(m_gamepad);
    }
    SDL_Quit();

    if(socket->isOpen())
        socket->close();
}

void MainWindow::loopSDL()
{


    // SDL3 Event Loop
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_EVENT_GAMEPAD_ADDED:{
            if (!m_gamepad) {
                m_gamepadId = event.gdevice.which;
                m_gamepad = SDL_OpenGamepad(m_gamepadId);

                ui->txtCntrlStatus->setText("Controller Connected!");
            }
            break;
        }
        case SDL_EVENT_GAMEPAD_REMOVED:{
            if (event.gdevice.which == m_gamepadId) {
                SDL_CloseGamepad(m_gamepad);
                m_gamepad = nullptr;
                ui->txtCntrlStatus->setText("Waiting for controller...");
            }
            break;
        }
        default:
            break;
        }
    }

    // Read Data if connected
    if (m_gamepad) {
        // Read Controller data
        Sint16 lx = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_LEFTX);
        Sint16 ly = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_LEFTY);
        Sint16 rx = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_RIGHTX);
        Sint16 ry = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_RIGHTY);

        vx = -ly / vx_scaler; //Koordinátacsere, hogy a hosszanti sebesség legyen a vx és "-", hogy az előre legyen a pozitív
        vy = lx / vy_scaler;
        w_z = rx / w_scaler;

        /*float r_length = sqrt(pow(rx,2)+pow(ry,2));
        if(r_length > 1000){
            phi_ref = phi;
            phi = tan(double(rx/ry));
            w_z = (phi-phi_ref) / w_scaler;
        }
        else w_z = 0;*/


        w1 = 1 / mech_R * (vx + vy - (mech_lx+mech_ly) * w_z);
        w2 = 1 / mech_R * (-vx + vy + (mech_lx+mech_ly) * w_z);
        w3 = 1 / mech_R * (-vx + vy - (mech_lx+mech_ly) * w_z);
        w4 = 1 / mech_R * (vx + vy + (mech_lx+mech_ly) * w_z);

        N1 =  Sint16(trunc(w1 * RPS_Factor));
        N2 =  Sint16(trunc(w2 * RPS_Factor));
        N3 =  Sint16(trunc(w3 * RPS_Factor));
        N4 =  Sint16(trunc(w4 * RPS_Factor));

        ui->LCDLeftX->display(lx);
        ui->LCDLeftY->display(ly);
        ui->LCDRightX->display(rx);
        ui->LCDRightY->display(ry);
        ui->LCD_vx->display(double(vx)*100);
        ui->LCD_vy->display(vy);
        ui->LCD_phi->display(w_z);


        message = QString("$C%1%2%3%4\r")
                      .arg((quint16)N1, 4, 16, QLatin1Char('0'))
                      .arg((quint16)N2, 4, 16, QLatin1Char('0'))
                      .arg((quint16)N3, 4, 16, QLatin1Char('0'))
                      .arg((quint16)N4, 4, 16, QLatin1Char('0'));

        message = message.toUpper();
        //Itt nem küldjük ki, hanem egy külön loopban van küldve 50ms-onként

        if(Wizfi_Active == 1){
            SendWizfi();
        }

    }
}

void MainWindow::SendVCP()
{
    QByteArray payload = message.toLatin1(); //ASCII kódolásra való konverzió
    ui->txtOutMsg->setText(payload);
    serialPort->write(payload); //kiírás soros portra
}

void MainWindow::SendWizfi()
{
    QByteArray payload = message.toLatin1();

    socket->write(payload);
}

void MainWindow::populatePortList()
{
    ui->portcomboBox->clear();
    const auto infos = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &info : infos) {
        // STM32 eszközök szűrése VID alapján
        if (info.vendorIdentifier() == STM32_VID) {
            QString portName = info.portName();
            QString description = info.description();
            ui->portcomboBox->addItem(portName + " (" + description + ")", portName);
        }
    }

    if (ui->portcomboBox->count() == 0) {
        ui->portcomboBox->addItem("Nem található STM32 Nucleo eszköz");
        ui->btnConnectNucleo->setEnabled(false);
    }
}

void MainWindow::on_btnConnectNucleo_clicked()
{
    // Ha már nyitva van a port, akkor bontjuk a kapcsolatot
    if (serialPort->isOpen()) {
        serialPort->close();
        ui->btnConnectNucleo->setText("Csatlakozás");
        ui->portcomboBox->setEnabled(true);
        return;
    }

    // A ComboBox-ban elmentettük a "COMx" nevű adatot (második paraméter az addItem-nél)
    QString portName = ui->portcomboBox->currentData().toString();
    if (portName.isEmpty()) return;

    // Paraméterek beállítása a config struktúrából
    serialPort->setPortName(portName);
    serialPort->setBaudRate(config.baudRate);
    serialPort->setDataBits(config.dataBits);
    serialPort->setParity(config.parity);
    serialPort->setStopBits(config.stopBits);
    serialPort->setFlowControl(config.flowControl);

    if (serialPort->open(QIODevice::ReadWrite)) {
        serialPort->setDataTerminalReady(true);

        ui->btnConnectNucleo->setText("Bontás");
        ui->portcomboBox->setEnabled(false);
        readBuffer.clear(); // Új csatlakozásnál ürítjük a korábbi szemetet
    } else {
        QMessageBox::critical(this, "Hiba", "Nem sikerült megnyitni a portot: " + serialPort->errorString());
    }
}

void MainWindow::readSerialData()
{
    // Olvassuk ki az éppen beérkezett adatokat és fűzzük hozzá a pufferhez
    QByteArray newData = serialPort->readAll();

    ui->UART_LOG->insertPlainText(QString::fromLatin1(newData));

    QScrollBar *scrollbar = ui->UART_LOG->verticalScrollBar();
    scrollbar->setValue(scrollbar->maximum());

    readBuffer.append(newData);

    // Biztonsági védelem, nehogy elfogyjon a RAM, ha az STM32 valamiért nem küld '\r'-t
    if (readBuffer.size() > config.maxBufferSize) {
        readBuffer.clear();
    }

    // Feldolgozzuk a puffert addig, amíg van benne teljes ( '\r'-el lezárt ) üzenet
    int endIndex = readBuffer.indexOf('\r');
    while (endIndex != -1) {
        // Kinyerünk egyetlen teljes sort
        QByteArray message = readBuffer.left(endIndex);

        // Eltávolítjuk a pufferből a feldolgozott üzenetet és a '\r'-t (+1)
        readBuffer.remove(0, endIndex + 1);

        parseData(message);

        // Ellenőrizzük, van-e még másik lezárt üzenet is a pufferben
        endIndex = readBuffer.indexOf('\r');
    }
}

void MainWindow::parseData(const QByteArray &data)
{
    // C példád: "$CD%04X%04X%04X%04X%04X"
    // Felépítés: 3 betű ($CD) + 5 db 4 karakteres hex string = 23 karakter
    if (data.startsWith("$CD") && data.length() >= 23) {

        bool ok; // Ide jelzi a Qt, ha sikeres volt a hex->int konverzió
        QString msgString = QString::fromLatin1(data);

        // A .mid(kezdet_index, hossz) függvénnyel vágjuk ki a darabokat
        // A .toUShort(..., 16) alakítja át a szöveges hexát egy 16 bites előjel nélküli egész számmá
        uint16_t tempSetp   = QStringView(msgString).mid(3, 4).toUShort(&ok, 16);
        uint16_t tempSpeed  = QStringView(msgString).mid(7, 4).toUShort(&ok, 16);
        uint16_t tempAngle  = QStringView(msgString).mid(11, 4).toUShort(&ok, 16);
        uint16_t tempActVal = QStringView(msgString).mid(15, 4).toUShort(&ok, 16);
        uint16_t tempTstamp = QStringView(msgString).mid(19, 4).toUShort(&ok, 16);


        // Megjelenítés a UI-on:
        if (ok) {
            dcmCtrlSetp   = tempSetp;
            motorSpeed    = tempSpeed;
            motorExtAngle = tempAngle;
            motorActVal   = tempActVal;
            remTstamp     = tempTstamp;

        }

    }

    if(data.startsWith("Meg"))
    {
        rx_demo = QString::fromLatin1(data);
    }
}

void MainWindow::onSocketConnected()
{
    ui->WizfiLOG->append("Successfully connected to WizFi360!");
    Wizfi_Active = 1;
}

void MainWindow::onSocketError(QAbstractSocket::SocketError socketError)
{
    ui->WizfiLOG->append("Error: " + socket->errorString());
    Wizfi_Active = 0;
}

void MainWindow::on_btnConnectWizfi_clicked()
{
    QString ipAddress = ui->txtIPAdr->text();
    quint16 port = ui->txtPort->text().toUShort();

    ui->WizfiLOG->append("Attempting to connect to " + ipAddress + ":" + QString::number(port) + "...");
    socket->connectToHost(ipAddress, port);
}


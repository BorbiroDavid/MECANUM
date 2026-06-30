#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QScrollBar>
#include <QLCDNumber>
#include <QGraphicsItem>
#include <QPixmap>
#include <QGraphicsScene>

//#include <stdio.h>
#include <cmath>


 //Mechanum modell geometriai állandói
#define mech_lx 0.0875
#define mech_ly 0.125
#define mech_R 0.054

//Bemenetek skálázó faktorai reális sebességekért
#define vx_scaler 100000.0
#define vy_scaler 100000.0
#define w_scaler 100000.0

#define RPS_Factor 1344 // Scale factor between rps and period

//Szimulációhoz
float currentTheta = 0.0;
float currentX = 0.0;
float currentY = 0.0;

//Valós sebességek [m/s]
float vx = 0.0;
float vy = 0.0;
float w_z = 0.0;
float phi = 0.0;

//Lokális fordulatszámok [1/s]
float w1 = 0.0;
float w2 = 0.0;
float w3 = 0.0;
float w4 = 0.0;

//Skálázott fordulatszámok
qint16 N1_out = 0, N1_in =0;
qint16 N2_out = 0, N2_in =0;
qint16 N3_out = 0, N3_in =0;
qint16 N4_out = 0, N4_in =0;

qint16 Wizfi_Active = 0;

qint16 ctrlr_lx = 0;
qint16 ctrlr_ly = 0;
qint16 ctrlr_rx = 0;
qint16 ctrlr_ry = 0;


QString message;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(new QSerialPort(this))
{
    ui->setupUi(this);

    populatePortList();

    // A QSerialPort readyRead jele
    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    // Initialize SDL
    SDL_Init(SDL_INIT_GAMEPAD);

    socket = (new QTcpSocket(this)); // Initialize the socket

    // Connect socket signals
    connect(socket, &QTcpSocket::connected, this, &MainWindow::onSocketConnected);
    connect(socket, &QTcpSocket::errorOccurred, this, &MainWindow::onSocketError);

    //Grafikus szimuláció inicializálása
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    QPixmap carPixmap("Mecanum.png");
    carItem = new QGraphicsPixmapItem(carPixmap);
    scene->addItem(carItem);
    carItem->setTransformOriginPoint(carItem->boundingRect().center());
    carItem->setScale(0.2);

    if (carPixmap.isNull()) {
        qDebug() << "Hiba: A kiskocsi képe nem tölthető be!";
    }

    // Timer to poll SDL events every 16ms (~60 FPS)
    m_pollTimer = new QTimer(this);
    connect(m_pollTimer, &QTimer::timeout, this, &MainWindow::loopSDL);
    m_pollTimer->start(10);

    //Timer to send on VCP port
    SendDataTimer = new QTimer(this);
    connect(SendDataTimer, &QTimer::timeout, this, &MainWindow::SendVCP);
    SendDataTimer->start(50);

    //Timer for graphical simulation
    SimulationTimer = new QTimer(this);
    connect(SimulationTimer, &QTimer::timeout, this, &MainWindow::Simulation);
    SimulationTimer->start(100);

    //Timer for Control and Display updates
    ControlTimer = new QTimer(this);
    connect(ControlTimer, &QTimer::timeout, this, &MainWindow::Control_and_Display);
    ControlTimer->start(20);

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
        ctrlr_lx = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_LEFTX);
        ctrlr_ly = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_LEFTY);
        ctrlr_rx = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_RIGHTX);
        ctrlr_ry = SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_RIGHTY);
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
    if (data.startsWith("$C") && data.length() >= 18) {

        bool ok; // Ide jelzi a Qt, ha sikeres volt a hex->int konverzió
        QString msgString = QString::fromLatin1(data);

        // A .mid(kezdet_index, hossz) függvénnyel vágjuk ki a darabokat
        // A .toUShort(..., 16) alakítja át a szöveges hexát egy 16 bites előjel nélküli egész számmá
        uint16_t temp_N1_in   = QStringView(msgString).mid(3, 4).toUShort(&ok, 16);
        uint16_t temp_N2_in  = QStringView(msgString).mid(7, 4).toUShort(&ok, 16);
        uint16_t temp_N3_in  = QStringView(msgString).mid(11, 4).toUShort(&ok, 16);
        uint16_t temp_N4_in = QStringView(msgString).mid(15, 4).toUShort(&ok, 16);

        if (ok) {
            N1_in = temp_N1_in;
            N2_in = temp_N2_in;
            N3_in = temp_N3_in;
            N4_in = temp_N4_in;
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

void MainWindow::Simulation(){

    const float dt = 0.1;
    currentTheta += w_z * dt;

    // 3. Pozíció frissítése a jármű orientációja alapján (B verzió)
    float vx_global = (vx * std::sin(currentTheta) + vy * std::cos(currentTheta)) * 30.0f; // Qt Y iránya miatt itt + van a forgatásiránytól függően
    float vy_global = (vx * std::cos(currentTheta) - vy * std::sin(currentTheta)) * -30.0f;

    currentX += vx_global * dt;
    currentY += vy_global * dt;

    carItem->setPos(currentX, currentY);
    // Átváltás radiánból fokba
    float theta_Deg = currentTheta * (180.0 / M_PI);
    carItem->setRotation(theta_Deg);

}

void MainWindow::Control_and_Display(){

    if(abs(ctrlr_ly) > 300) vx = -ctrlr_ly / vx_scaler;
    else vx = 0;    //Koordinátacsere, hogy a hosszanti sebesség legyen a vx és "-", hogy az előre legyen a pozitív
    if(abs(ctrlr_lx)> 300) vy = ctrlr_lx / vy_scaler;
    else vy = 0;
    if(abs(ctrlr_rx)>300) w_z = ctrlr_rx / w_scaler;
    else w_z = 0;


    w1 = 1 / mech_R * (vx + vy - (mech_lx+mech_ly) * w_z);
    w2 = 1 / mech_R * (-vx + vy + (mech_lx+mech_ly) * w_z);
    w3 = 1 / mech_R * (-vx + vy - (mech_lx+mech_ly) * w_z);
    w4 = 1 / mech_R * (vx + vy + (mech_lx+mech_ly) * w_z);

    N1_out =  qint16(trunc(w1 * RPS_Factor));
    N2_out =  qint16(trunc(w2 * RPS_Factor));
    N3_out =  qint16(trunc(w3 * RPS_Factor));
    N4_out =  qint16(trunc(w4 * RPS_Factor));

    ui->LCDLeftX->display(ctrlr_lx);
    ui->LCDLeftY->display(ctrlr_ly);
    ui->LCDRightX->display(ctrlr_rx);
    ui->LCDRightY->display(ctrlr_ry);
    ui->LCD_vx->display(vx);
    ui->LCD_vy->display(vy);
    ui->LCD_phi->display(w_z);
    ui->LCD_N1->display(N1_out);
    ui->LCD_N2->display(N2_out);
    ui->LCD_N3->display(N3_out);
    ui->LCD_N4->display(N4_out);

    message = QString("$C%1%2%3%4\r")
                  .arg((quint16)N1_out, 4, 16, QLatin1Char('0'))
                  .arg((quint16)N2_out, 4, 16, QLatin1Char('0'))
                  .arg((quint16)N3_out, 4, 16, QLatin1Char('0'))
                  .arg((quint16)N4_out, 4, 16, QLatin1Char('0'));

    message = message.toUpper();
    //Itt nem küldjük ki, hanem egy külön loopban van küldve 50ms-onként

    if(Wizfi_Active == 1){
        SendWizfi();
    }
}

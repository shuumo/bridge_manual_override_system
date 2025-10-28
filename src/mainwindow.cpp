#include "mainwindow.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      startupPage(new QWidget(this)),
      controlPage(new QWidget(this)),
      stackedWidget(new QStackedWidget(this)),
      connection(new ConnectionManager(this)),
      fsm(new StateMachine(this))
{
    setupStartupPage();
    setupControlPage();

    stackedWidget->addWidget(startupPage);
    stackedWidget->addWidget(controlPage);
    setCentralWidget(stackedWidget);

    connect(connection, &ConnectionManager::statusUpdated,
            this, &MainWindow::updateBridgeStatus);
    connect(connection, &ConnectionManager::connectionLost,
            this, &MainWindow::handleConnectionLost);

    //hardcoded ip and port
    connection->start("192.168.10.179", 4210);
}

void MainWindow::setupStartupPage() {
    auto layout = new QVBoxLayout();
    auto label = new QLabel("Connecting to ESP32 Bridge...", startupPage);
    label->setAlignment(Qt::AlignCenter);
    layout->addWidget(label);
    startupPage->setLayout(layout);
}

void MainWindow::setupControlPage() {
    auto layout = new QVBoxLayout();

    connectionLabel = new QLabel("Connected to ESP32", controlPage);
    stateLabel = new QLabel("Bridge State: INACTIVE", controlPage);
    connectionLabel->setAlignment(Qt::AlignCenter);
    stateLabel->setAlignment(Qt::AlignCenter);

    layout->addWidget(connectionLabel);
    layout->addWidget(stateLabel);

    overrideButton = new QPushButton("OVERRIDE: OFF", controlPage);
    overrideButton->setStyleSheet("background-color: green; color: white; font-weight: bold;");
    connect(overrideButton, &QPushButton::clicked, this, &MainWindow::onOverrideToggled);
    layout->addWidget(overrideButton);

    QGridLayout* grid = new QGridLayout();
    QStringList states = {"IDLE", "OPENING", "OPEN", "CLOSING", "ERROR"};
    for (int i = 0; i < states.size(); ++i) {
        QPushButton* btn = new QPushButton(states[i], controlPage);
        btn->setEnabled(false);  // disabled until override
        btn->setStyleSheet("background-color: #333333; color: white;");
        connect(btn, &QPushButton::clicked, this, &MainWindow::onStateButtonPressed);
        stateButtons.append(btn);
        grid->addWidget(btn, i / 3, i % 3);
    }
    layout->addLayout(grid);

    controlPage->setLayout(layout);
}

void MainWindow::switchToControlPage() {
    stackedWidget->setCurrentWidget(controlPage);
}

void MainWindow::onOverrideToggled() {
    auto mode = fsm->currentMode();

    if (mode == ControlMode::AUTO) {
        fsm->setMode(ControlMode::OVERRIDE);
        overrideButton->setText("OVERRIDE: ON");
        overrideButton->setStyleSheet("background-color: red; color: white; font-weight: bold;");
        for (auto* b : stateButtons)
            b->setEnabled(true),
            b->setStyleSheet("background-color: lightgreen; color: black;");
        connection->sendCommand("MODE=OVERRIDE");
    } else {
        fsm->setMode(ControlMode::AUTO);
        overrideButton->setText("OVERRIDE: OFF");
        overrideButton->setStyleSheet("background-color: green; color: white; font-weight: bold;");
        for (auto* b : stateButtons)
            b->setEnabled(false),
            b->setStyleSheet("background-color: #333333; color: white;");
        connection->sendCommand("MODE=AUTO");
    }
}

void MainWindow::onStateButtonPressed() {
    if (fsm->currentMode() != ControlMode::OVERRIDE)
        return;

    QPushButton* senderBtn = qobject_cast<QPushButton*>(sender());
    if (!senderBtn) return;

    QString command = "SET_STATE " + senderBtn->text();
    connection->sendCommand(command);
}

void MainWindow::updateBridgeStatus(const BridgeStatus& status) {
    fsm->setState(status.state);
    fsm->setMode(status.mode);

    if (!status.connected) {
        connectionLabel->setText("Disconnected");
        stateLabel->setText("Bridge State: INACTIVE");
        return;
    }

    connectionLabel->setText("Connected to ESP32");
    QString stateText = "Bridge State: ";

    switch (status.state) {
        case BridgeState::IDLE: stateText += "IDLE"; break;
        case BridgeState::OPENING: stateText += "OPENING"; break;
        case BridgeState::OPEN: stateText += "OPEN"; break;
        case BridgeState::CLOSING: stateText += "CLOSING"; break;
        case BridgeState::ERROR: stateText += "ERROR"; break;
        default: stateText += "UNKNOWN";
    }

    stateLabel->setText(stateText);

    if (stackedWidget->currentWidget() == startupPage)
        switchToControlPage();
}

void MainWindow::handleConnectionLost() {
    QMessageBox::warning(this, "Connection Lost",
                         "Lost connection to the bridge. Attempting to reconnect...");
    stackedWidget->setCurrentWidget(startupPage);
}


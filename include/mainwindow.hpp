/*
* The actual Qt Widget-based GUI logic.
* Includes main shit like startup page, control panel, button grid.
*/

#pragma once
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QGridLayout>
#include "connection_manager.hpp"
#include "state_machine.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onOverrideToggled();
    void onStateButtonPressed();
    void updateBridgeStatus(const BridgeStatus& status);
    void handleConnectionLost();

private:
    void setupStartupPage();
    void setupControlPage();
    void switchToControlPage();

    QWidget* startupPage;
    QWidget* controlPage;

    QLabel* connectionLabel;
    QLabel* stateLabel;
    QPushButton* overrideButton;
    QVector<QPushButton*> stateButtons;
    QStackedWidget* stackedWidget;

    ConnectionManager* connection;
    StateMachine* fsm;
};


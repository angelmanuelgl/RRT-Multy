#pragma once
#include <QMainWindow>

#include <string>

class RRTWidget;

//Declaracion clase RRTWindow para heredar QMainWindow
class RRTWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit RRTWindow(QWidget *parent = nullptr);
    RRTWindow(const std::string& scenarioPath,
              const std::string& obstaclesPath,
              QWidget *parent = nullptr);
};


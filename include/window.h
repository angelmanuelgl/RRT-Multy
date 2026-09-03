#pragma once
#include <QMainWindow>

class RRTWidget;

//Declaracion clase RRTWindow para heredar QMainWindow
class RRTWindow : public QMainWindow {
    Q_OBJECT
public:
    RRTWindow(QWidget *parent = nullptr);
};


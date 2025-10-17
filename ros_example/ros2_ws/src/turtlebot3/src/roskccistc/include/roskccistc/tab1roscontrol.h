#ifndef TAB1ROSCONTROL_H
#define TAB1ROSCONTROL_H

#include <QWidget>
#include <QDebug>
#include "rosnode.h"

namespace Ui {
class Tab1RosControl;
}

class Tab1RosControl : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1RosControl(int argc, char** argv, QWidget *parent = nullptr);
    ~Tab1RosControl();

private:
    Ui::Tab1RosControl *ui;
    RosNode *pRosNode;
    int a;
    int b;
private slots:
    void slotLdsReceive(float *);
    void on_pPBStudy_clicked();
    void on_pPBFront_clicked();
    void on_pPBLiving_clicked();
    void on_pPBBedroom_clicked();
    void on_pPushButtonGo_clicked();
    void go_PubSlot(double,double,double);
    void on_pLELed10_returnPressed();
    void on_pPBLed3_clicked(bool checked);
    void on_pPBLed2_clicked(bool checked);
    void slotLedLcdDisplay(int);
};

#endif // TAB1ROSCONTROL_H

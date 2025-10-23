#include "tab1roscontrol.h"
#include "ui_tab1roscontrol.h"

Tab1RosControl::Tab1RosControl(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab1RosControl)
{
    ui->setupUi(this);
    a=0;
    b=0;
    pRosNode = new RosNode(argc,argv);
    pRosNode->pLcamView = ui->pLabelCam;
    pRosNode->start();
    connect(pRosNode, SIGNAL(sigLdsReceive(float *)), this, SLOT(slotLdsReceive(float *)));
    connect(pRosNode, SIGNAL(rosShutdown()), qApp, SLOT(quit()));
//    connect(pRosNode, SIGNAL(sigBot3GpioService(int)),ui->pLcdNumberRes,SLOT(display(int)));
    connect(pRosNode, SIGNAL(sigBot3GpioService(int)), this, SLOT(slotLedLcdDisplay(int)));
}

Tab1RosControl::~Tab1RosControl()
{
    delete ui;
}

void Tab1RosControl::slotLdsReceive(float * pScanData)
{
    ui->pLcdNumber1->display(pScanData[0]);
    ui->pLcdNumber2->display(pScanData[1]);
    ui->pLcdNumber3->display(pScanData[2]);
    ui->pLcdNumber4->display(pScanData[3]);
}

void Tab1RosControl::on_pPBStudy_clicked()
{
    pRosNode->go_goal("map",0,0,0.7);
}

void Tab1RosControl::on_pPBFront_clicked()
{
    pRosNode->go_goal("map",0.03,2.99,0.7);
}

void Tab1RosControl::on_pPBLiving_clicked()
{
    pRosNode->go_goal("map",0.03,1.99,0.7);
}

void Tab1RosControl::on_pPBBedroom_clicked()
{
    pRosNode->go_goal("map",0.03,0.99,0.7);
}

void Tab1RosControl::on_pPushButtonGo_clicked()
{
    pRosNode->go_goal("map",ui->pDoubleSpinBox1->value(),ui->pDoubleSpinBox2->value(),ui->pDoubleSpinBox3->value());
}

void Tab1RosControl::go_PubSlot(double x,double y,double w)
{
    pRosNode->go_goal("map",x,y,w);
}

void Tab1RosControl::on_pLELed10_returnPressed()
{

    b = ui->pLELed10->text().toInt();
    pRosNode->slotBot3GpioService(a,b);
//    qDebug() << " b : " << b;
}

void Tab1RosControl::on_pPBLed2_clicked(bool checked)
{
    if(checked)
    {
        a |=  0x00000001;       // led2 on
        ui->pPBLed2->setChecked(false);
    }
    else
    {
        a &= ~0x00000001;       // led2 off
        ui->pPBLed2->setChecked(true);
    }
    pRosNode->slotBot3GpioService(a,b);
//    qDebug() << " a : " << a;
}

void Tab1RosControl::on_pPBLed3_clicked(bool checked)
{
    if(checked)
    {
        a |= 0x00000002;       // led3 on
        ui->pPBLed3->setChecked(false);
    }
    else
    {
        a &= ~0x00000002;       // led3 off
        ui->pPBLed3->setChecked(true);
    }
    pRosNode->slotBot3GpioService(a,b);
//    qDebug() << " a : " << a;
}

void Tab1RosControl::slotLedLcdDisplay(int res)
{

    if(res & 0x4)   // LED2 on
    {
        ui->pPBLed2->setChecked(true);
    }
    else if(res & 0x4 == 0)
    {
         ui->pPBLed2->setChecked(false);
    }
    if(res & 0x8)
    {
        ui->pPBLed3->setChecked(true);
    }
    else if(res & 0x8 == 0)
    {
       ui->pPBLed3->setChecked(false);
    }
    ui->pLcdNumberRes->display(res);
}



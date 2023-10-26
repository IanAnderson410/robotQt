#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QSerialPort1 = new QSerialPort(this);
    QPaintBox1  = new QPaintBox( 0, 0, ui->widget); //Create adentro del widget
    QTimer1     = new QTimer(this);

    connect(QSerialPort1, &QSerialPort:: readyRead, this, &MainWindow::OnRxQSerialPort1);
    connect(QTimer1, &QTimer::timeout, this, &MainWindow::Every10ms);
    ui->CMDcomboBox->addItem("GetALIVE", 0xF0);
    ui->CMDcomboBox->addItem("GetFIRMWARE", 0xF1);
    ui->CMDcomboBox->addItem("SetSERVO", 0xA2);
    ui->CMDcomboBox->addItem("GetDistance", 0xA3);
    ui->ModeLineEdit->setText("Mode: WAITING");

    QTimer1->start(200);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Every10ms(){
    QPainter paint(QPaintBox1->getCanvas());
    QPen pen;
    QPoint centerCircle;



    centerCircle.setX(250);
    centerCircle.setY(500);
    pen.setWidth(2);
    pen.setColor(Qt::green);
    paint.setPen(pen);
    paint.setBrush(Qt::black);
    pen.setWidth(0.5);
    paint.drawEllipse(centerCircle, 500, 500);
    paint.drawEllipse(centerCircle, 450, 450);
    paint.drawEllipse(centerCircle, 400, 400);
    paint.drawEllipse(centerCircle, 350, 350);
    paint.drawEllipse(centerCircle, 300, 300);
    paint.drawEllipse(centerCircle, 250, 250);
    paint.drawEllipse(centerCircle, 200, 200);
    paint.drawEllipse(centerCircle, 150, 150);
    paint.drawEllipse(centerCircle, 100, 100);

    if(QSerialPort1->isOpen()){
        if(state == RADAR){
            angulo +=2;
            if(angulo > 90){
                angulo= -90;
            }
            currentValue.servo = angulo;
            ui->SetServoAngleLineEdit->setText("angle: " + QString::number(currentValue.servo));
            cmdID = 0xA2; //setServo
            SendDataTx();
            DrawRadar();
            }
    }else{
        state=WAITING;
        ui->openPortPushButton->setText("OPEN");
        char mode[30];
        strcpy( mode, "WAITING");
        ui->ModeLineEdit->setText("Mode: " + QString(mode));
    }
}
void MainWindow::on_openPortPushButton_clicked(){
    if( QSerialPort1->isOpen()){
        ui->openPortPushButton->setText("OPEN");
        QSerialPort1->close();
        state=WAITING;
    }else{
        QSerialPort1->setPortName(ui->OpenPortcomboBox->currentText());
        QSerialPort1->setBaudRate(QSerialPort::Baud115200);
        if(QSerialPort1->open(QSerialPort::ReadWrite)){
            ui->openPortPushButton->setText("CLOSE");
        }
    }
}
void MainWindow::OnRxQSerialPort1(){
    unsigned char *buf; // Buffer dinamico
    int count;
    QString str;

    count = QSerialPort1->bytesAvailable();     // Esta linea pregunta cuantos datos llegaron para
        // leer en el buffer
    if(count <= 0)                              // Si es menor o igual a cero algo esta mal
        return;
    buf = new unsigned char[count];    //Reservamos memoria una cantidad count bytes

    QSerialPort1->read( (char*)buf, count);             // Leemos el buffer una cantidad count de bytes

    str = "";

    for(int i=0; i<=count; i++){
        if(isalnum(buf[i]))
            str = str + QString("%1").arg((char)buf[i]);
        else
            str = str +"{" + QString("%1").arg(buf[i],2,16,QChar('0')) + "}";
    }
    ui->RxTextEdit->append("MBED->SERIAL->PC (" + str + ")");
    //Cada vez que se recibe un dato reinicio el timeOut
    rxData.timeOut=6;
    for(int i=0;i<count; i++){
        switch (proState) {
        case header_U:
            if (buf[i]=='U'){
                proState = header_N;
            }
            break;
        case header_N:
            if (buf[i]=='N')
                proState = header_E;
            else{
                i--;
                proState = header_U;
            }
            break;
        case header_E:
            if (buf[i]=='E')
                proState = header_R;
            else{
                i--;
                proState = header_U;
            }
            break;
        case header_R:
            if (buf[i]=='R')
                proState = nBytes;
            else{
                i--;
                proState = header_U;
            }
            break;
        case nBytes:
            rxData.nBytes= buf[i];

            proState = token;
            break;
        case token:
            if (buf[i]==':'){
                proState = payload;
                rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                rxData.payLoad[0]=rxData.nBytes;
                rxData.index=1;
            }
            else{
                i--;
                proState = header_U;
            }
            break;
        case payload:
            if (rxData.nBytes>1){
                rxData.payLoad[rxData.index++]=buf[i];
                rxData.cheksum^=buf[i];
            }
            rxData.nBytes--;
            if(rxData.nBytes==0){
                proState = header_U;
                if(rxData.cheksum== buf[i]){
                    decodeData(&rxData.payLoad[0], SERIE);
                }else{
                    //ui->->append("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    //ui->textEdit_RAW->append("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );
                    ui->InfoTextEdit->append("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    ui->InfoTextEdit->append("Chk recibido ** " +QString().number(buf[i],16) + " **" );
                }
            }
            break;
        default:
            proState = header_U;
            break;
        }
    }
    delete [] buf;
}
void MainWindow::SendDataTx(){
    unsigned char bufTx[256];
    char CMDTx[10];
    _udat   w;
    uint8_t index=0;
    uint8_t checksum;
    uint16_t value;

    if(QSerialPort1->isOpen()){
        bufTx[index++] = 'U';
        bufTx[index++] = 'N';
        bufTx[index++] = 'E';
        bufTx[index++] = 'R';
        bufTx[index++] = 0x00;
        bufTx[index++] = ':';
        //cmdID = ui->toSendLineEdit->text().toInt();
        switch(cmdID){
        case getAlive:
            bufTx[index++] = cmdID;
            bufTx[nBytes] = 0x02;
            //strcpy(CMDTx, "ALIVE");
            break;
        case getFirmware:
            bufTx[index++] = cmdID;
            bufTx[nBytes] = 0x02;
            //strcpy(CMDTx, "FIRMWARE");
            break;
        case setServo:
            bufTx[index++] = setServo;
            w.i32 = currentValue.servo;
            bufTx[index++] = w.i8[0];
            bufTx[nBytes]= 0x03;
            break;
        case 0xA3:  //getDistance
            bufTx[index++]  = 0xA3;
            bufTx[nBytes]   = 0x02;
            //strcpy(CMDTx, "GETDISTANCE");
            break;
        //case Set
        }
        for(int i=0 ;i<index;i++)
                checksum^=bufTx[i];
        bufTx[index] = checksum;
        //ui->TxTextEdit->setText("BufTx : " + QString::number(bufTx));        
        //ui->TxTextEdit->append(QString::fromUtf8(reinterpret_cast<const char*>(bufTx)));
        if(QSerialPort1->isWritable()){
            QSerialPort1->write(reinterpret_cast<char *>(bufTx),bufTx[nBytes]+payload);
        }

        value = bufTx[nBytes]+payload;

        ui->TxTextEdit->append("CMD         = " +QString(CMDTx));
        ui->TxTextEdit->append("Index      = " +QString().number(index,10));
        ui->TxTextEdit->append("n Bytes    = " +QString().number(value,10));
        ui->TxTextEdit->append("Checksun   = " +QString().number(checksum,16));

    }
    cmdID = '\0';
}
void MainWindow::on_SendPushButton_clicked(){
    if(QSerialPort1->isOpen()){
        cmdID = ui->CMDcomboBox->currentData().toInt();
        if(cmdID == 0xA2){
            currentValue.servo = ui->SetServoDoubleSpinBox->value();
            //ui->SetServoAngleLineEdit->setText("angle: " + QString::number(currentValue.servo));
        }
        SendDataTx();
    }
}
void MainWindow::decodeData(uint8_t *datosRx, uint8_t source){
    if(QSerialPort1->isOpen()){
        int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
        QString str, strOut;
        _udat w;
        for(int i = 1; i<length; i++){
            if(isalnum(datosRx[i]))
                str = str + QString("%1").arg(char(datosRx[i]));
            else
                str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
        }
        ui->InfoTextEdit->append("(MBED->PC)->decodeData (" + str + ")");
        switch (datosRx[1]) {
        case getAlive://     GETALIVE=0xF0,
            if(datosRx[2]==ACK){
                contadorAlive++;
                str="ALIVE recibido";
                /* if(source)
                    str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
                else{
                    contadorAlive++;
                    str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
                }*/
            }else{
                str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
            }
            ui->RxTextEdit->append(str);
            break;
        case getFirmware://     GETFIRMWARE=0xF1
            str = "FIRMWARE:";
            for(uint8_t a=0;a<(datosRx[0]-1);a++){
                str += (QChar)datosRx[2+a];
            }
            ui->RxTextEdit->append("FIRMWARE: " + str);
            break;
        case setServo://     SERVOANGLE=0xA2,
            if(datosRx[2]==0x0D)
                str= "Servo moviendose";
            else{
                if(datosRx[2]==0x0A)
                    str= "Servo en Posicion final";
            }
            ui->RxTextEdit->append(str);
            break;
        case getDistance://     GETDISTANCE=0xA3,
            w.ui8[0] = datosRx[2];
            w.ui8[1] = datosRx[3];
            w.ui8[2] = datosRx[4];
            w.ui8[3] = datosRx[5];
            str = QString().number(w.ui32/58);
            currentValue.distance = str.toInt();
            if( currentValue.distance < 0 ) currentValue.distance = -currentValue.distance;
            ui->LastDistance_LineEdit->setText(str+ " [cm]");
            ui->RxTextEdit->append("DISTANCIA: "+QString().number(w.ui32/58)+ " [cm]");
            //ui->InfoTextEdit->append("DISTANCIA: "+QString().number(w.ui32/58)+ " [cm]");
            //angulo = ui->SetServoDoubleSpinBox->value();
            //cmd= setServo;
            //SendDataTx();
            DrawRadar();
            break;
        /*case GETSPEED://     GETSPEED=0xA4,
             str = "VM1: ";
             w.ui8[0] = datosRx[2];
             w.ui8[1] = datosRx[3];
             w.ui8[2] = datosRx[4];
             w.ui8[3] = datosRx[5];
             strOut = QString("%1").arg(w.i32, 4, 10, QChar('0'));
             ui->label_LENC->setText(strOut);
             str = str + QString("%1").arg(w.i32, 4, 10, QChar('0')) + " - VM2: ";
             w.ui8[0] = datosRx[6];
             w.ui8[1] = datosRx[7];
             w.ui8[2] = datosRx[8];
             w.ui8[3] = datosRx[9];
             strOut = QString("%1").arg(w.i32, 4, 10, QChar('0'));
             ui->label_RENC->setText(strOut);
             str = str + QString("%1").arg(w.i32, 4, 10, QChar('0'));
             ui->textEdit_PROCCES->append(str);
             break;
        case GETSWITCHES: //GETSWITCHES=0xA5
             str = "SW3: ";
             if(datosRx[2] & 0x08)
                str = str + "HIGH";
             else
                str = str + "LOW";
             str = str + " - SW2: ";
             if(datosRx[2] & 0x04)
                str = str + "HIGH";
             else
                str = str + "LOW";
             str = str + " - SW1: ";
             if(datosRx[2] & 0x02)
                str = str + "HIGH";
             else
                str = str + "LOW";
             str = str + " - SW0: ";
             if(datosRx[2] & 0x01)
                str = str + "HIGH";
             else
                str = str + "LOW";
             ui->textEdit_PROCCES->append(str);
             break;    */
        /*case SETLEDS:
             str = "LD3: ";
             if(datosRx[2] & 0x08)
                str = str + "HIGH";
             else
                str = str + "LOW";
             str = str + " - LD2: ";
             if(datosRx[2] & 0x04)
                str = str + "HIGH";
             else
                str = str + "LOW";
             str = str + " - LD1: ";
             if(datosRx[2] & 0x02)
                str = str + "HIGH";
             else
                str = str + "LOW";
             str = str + " - LD0: ";
             if(datosRx[2] & 0x01)
                str = str + "HIGH";
             else
                str = str + "LOW";
             ui->textEdit_PROCCES->append(str);
             break;*/
            /*
        case GETANALOGSENSORS://     ANALOGSENSORS=0xA0,
             w.ui8[0] = datosRx[2];
             w.ui8[1] = datosRx[3];
             str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
             strOut = "LEFT IR: " + str;
             ui->textEdit_PROCCES->append(strOut);
             ui->label_LIR->setText(str);
             w.ui8[0] = datosRx[4];
             w.ui8[1] = datosRx[5];
             str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
             strOut = "CENTER IR: " + str;
             ui->textEdit_PROCCES->append(strOut);
             ui->label_CIR->setText(str);
             w.ui8[0] = datosRx[6];
             w.ui8[1] = datosRx[7];
             str =QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
             strOut = "RIGHT IR: " + str;
             ui->label_RIR->setText(str);
             ui->textEdit_PROCCES->append(strOut);
             break;
             */
            /*
        case SETMOTORTEST://     MOTORTEST=0xA1,
             if(datosRx[2]==0x0D)
                str= "Test Motores ACK";
             ui->textEdit_PROCCES->append(str);
             break;*/
        default:
            str = str + "UNKNOWN COMMAND ";
            ui->RxTextEdit->append(str);
        }
    }
}
void MainWindow::on_cleanTxPushButton_clicked(){    ui->TxTextEdit->clear(); }
void MainWindow::on_cleanRxPushButton_clicked(){    ui->RxTextEdit->clear(); }
void MainWindow::on_ModePushButton_clicked(){
    char mode[30];
    if(QSerialPort1->isOpen()){
        state ++;
        QPainter paint(QPaintBox1->getCanvas());
        QPen pen;
        QPoint centerCircle;
        if(state > 4 ) state = WAITING;

        switch(state){
        case WAITING:
            strcpy( mode, "WAITING");
            break;
        case RADAR:
            strcpy( mode, "RADAR");
            centerCircle.setX( 250);
            centerCircle.setY(500);
            pen.setWidth(2);
            pen.setColor(Qt::green);
            paint.setPen(pen);
            paint.setBrush(Qt::black);
            paint.drawEllipse(centerCircle, 250, 250);
            pen.setWidth(0.5);
            paint.drawEllipse(centerCircle, 500, 500);
            paint.drawEllipse(centerCircle, 450, 450);
            paint.drawEllipse(centerCircle, 400, 400);
            paint.drawEllipse(centerCircle, 350, 350);
            paint.drawEllipse(centerCircle, 300, 300);
            paint.drawEllipse(centerCircle, 250, 250);
            paint.drawEllipse(centerCircle, 200, 200);
            paint.drawEllipse(centerCircle, 150, 150);
            paint.drawEllipse(centerCircle, 100, 100);
            break;
        case MODE_1:
            strcpy( mode, "Seguidor de Linea");
            break;
        case MODE_2:
            strcpy( mode, "Esquivar Obstáculo");
                break;
        case MODE_3:
            strcpy( mode, "Mantener Distancia");
            break;
        }
    }else{
        strcpy( mode, "WAITING");
        state=WAITING;
    }
    ui->ModeLineEdit->setText("Mode: " + QString(mode));
}
void MainWindow::DrawRadar(){
    QPainter paint(QPaintBox1->getCanvas());
    QPen pen;
    QPoint MyPointsLine[2];
    cmdID = 0xA3;
    SendDataTx();
    MyPointsLine[0].setX(0);
    MyPointsLine[0].setY(0);
    MyPointsLine[1].setX(-(currentValue.distance * 2));
    MyPointsLine[1].setY(0);
    pen.setWidth(4);
    pen.setColor(Qt::green);
    paint.setPen(pen);
    paint.setBrush(Qt::green);
    paint.translate(ui->widget->width()/2, ui->widget->height());
    paint.rotate(90 - (angulo/2));
    paint.drawLines(MyPointsLine, 2);
    ui->InfoTextEdit->append("Angulo: " + QString::number(angulo));
    QPaintBox1->update();
}
void MainWindow::on_pushButton_clicked(){
    //angulo +=2;
    //DrawRadar();
}
void MainWindow::on_RightServoPushButton_clicked(){
    if( QSerialPort1->isOpen() && state == WAITING){
        currentValue.servo = 90;
        ui->SetServoAngleLineEdit->setText("angle: " + QString::number(currentValue.servo));
        cmdID =     0xA2; //setServo
        SendDataTx();
    }
}
void MainWindow::on_LeftServoPushButton_clicked(){
    if( QSerialPort1->isOpen() && state == WAITING){
        currentValue.servo = -90;
        ui->SetServoAngleLineEdit->setText("angle: " + QString::number(currentValue.servo));
        cmdID =     0xA2; //setServo
        SendDataTx();
    }
}
void MainWindow::on_SetWhitePushButton_clicked(){
    if(QSerialPort1->isOpen()){
        switch( ui->IR_ComboBox->currentData().toInt()){
        case 49:
            ui->InfoTextEdit->append("IR_1");
            break;
        case 50:
        case 51:

        default:
            ui->InfoTextEdit->append("IR_2");
            break;
        }
    }
}
void MainWindow::on_SetBlackPushButton_clicked(){

}
void MainWindow::on_GetAlivePushButton_clicked(){
    if(QSerialPort1->isOpen()){

    }
}


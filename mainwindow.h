#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QTextEdit>
#include <qpaintbox.h>
#include <QMessageBox>
#include <QLabel>
#include <QInputDialog>
#include <QTimer>
#include <math.h>
#include <QColor>



/*
    Consideraciones:
        -Mandar Alive cada un segundo

*/
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private slots:
    void Every10ms();

    void on_openPortPushButton_clicked();

    void OnRxQSerialPort1();
    /**
     * @brief SendDataTx()
     *
     *
     * */
    void SendDataTx();
    /**
     * @brief ReciveDataRx
     */



    void on_SendPushButton_clicked();    

    void decodeData(uint8_t *datosRx, uint8_t source);

    void on_cleanTxPushButton_clicked();

    void on_cleanRxPushButton_clicked();
    /**
     * @brief on_ModePushButton_clicked
     *
     * This funtion choose the state of the car, the default is "WAITING" and if the port
     * is closed for any reason, the car switches to this mode automatically
     */
    void on_ModePushButton_clicked();
    /**
     * @brief DrawRadar
     * DrawRadar draws a radar using a servo motor and an ultrasonic sensor. Every 100ms
     * the servo rotate 2 degrees and the sensor measure the distance. This information
     * is transformed in to a graphic of an marine radar and indicate if there are object
     * close to the car.
     */
    void DrawRadar();

    void on_pushButton_clicked();

    void on_RightServoPushButton_clicked();

    void on_LeftServoPushButton_clicked();

    void on_SetWhitePushButton_clicked();

    void on_SetBlackPushButton_clicked();

    void on_GetAlivePushButton_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *QSerialPort1;
    QPaintBox *QPaintBox1;
    QTimer *QTimer1;

    uint32_t every100ms;

    typedef enum{
        WAITING =0,
        RADAR,
        MODE_1, //Seguidor de linea
        MODE_2, //Esquivar obstaculo
        MODE_3, //Mantener distancia
    }_eMode;
    uint8_t state = WAITING;

    typedef struct{
    uint8_t header[4];
    uint8_t token = ';';
    uint8_t length = 9; // 8+1
    uint8_t payload[256];
    uint8_t checksum=0;
    }_sProtocolo;

    _sProtocolo pro;

    typedef enum{
        header_U,
        header_N,
        header_E,
        header_R,
        nBytes,
        token,
        payload
    }_ePro;

    _ePro proState;

    typedef enum{
        UDP=0,
        SERIE=1,
        ACK                 = 0x0D,
        getAlive            = 0xF0,
        getFirmware         = 0xF1,
        getButtonState      = 0x12,
        getAnalogSensors    = 0xA0,
        setMotor            = 0xA1, //int32_t  int32_t
        setServo            = 0xA2, //int8_t
        getDistance         = 0xA3,
        getVelocity         = 0xA4,
        setServoLimits      = 0xA5, // uint8_t; uint16_t; uint16_t
        setThisIsBlack      = 0xA6, // uint8_t; uint16_t; uint16_t; uint16_t
        setThisIsWhite      = 0xA7 // uint8_t; uint16_t; uint16_t; uint16_t
    }_eCMD;
    _eCMD cmd;
    /*Variables para los comandos*/

    typedef union {
    double  d32;
    float f32;
    int i32;
    unsigned int ui32;
    unsigned short ui16[2];
    short i16[2];
    uint8_t ui8[4];
    char chr[4];
    unsigned char uchr[4];
    int8_t  i8[4];
    }_udat;
    _udat myWord;



    /**
     * @brief _sValue
     *
     *Almacena los datos de las diferentes piezas de hardware
     * */
    typedef struct{
        int8_t servo;
        int8_t distance;
    }_sValue;
    _sValue currentValue;

    int16_t angulo= -90;

    uint16_t cmdID;



    typedef struct{
    uint8_t timeOut;
    uint8_t cheksum;
    uint8_t payLoad[256];
    uint8_t nBytes;
    uint8_t index;
    }_sDatos ;

    _sDatos rxData, rxDataUdp;

    int contadorAlive=0;

    typedef struct{
        uint8_t thisIsWhite;
        uint8_t setThisBlack;
    }_sSetColor;
};
#endif // MAINWINDOW_H

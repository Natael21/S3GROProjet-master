#ifndef CAR_H
#define CAR_H

#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QPropertyAnimation>
#include <QJsonObject>
#include <QJsonDocument>

class CarItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x WRITE setX)
    qreal x();

public:
    explicit CarItem();
    ~CarItem();
    void receiveFromSerial(QString);
    void onMessageReceived(QString);
    void setX(qreal x);

signals:

public slots:
private:
    double positionVoiture = 0;
    double largeurRobot = 50;
    double hauteurRobot = 10;
    qreal m_x;
    QGraphicsPixmapItem * vehicule;
    QPropertyAnimation * xAnimation;
    QString msgReceived_{""};
    QString msgBuffer_{""};

};

#endif // CAR_H

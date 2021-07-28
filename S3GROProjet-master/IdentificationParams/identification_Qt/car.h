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
    explicit CarItem(double position);
    ~CarItem();
    void receiveFromSerial(QString);
    void onMessageReceived(QString);
    void setX(qreal x);

signals:

public slots:
private:
    double positionVehicule;
    qreal m_x;
    QGraphicsPixmapItem * vehicule;
    QPropertyAnimation * xAnimation;

};

#endif // CAR_H

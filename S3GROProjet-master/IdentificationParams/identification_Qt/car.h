#ifndef CAR_H
#define CAR_H

#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QGraphicsScene>
#include<QPropertyAnimation>

class CarItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x WRITE setX)

public:
    explicit CarItem(double currentpos, double newpos);
    ~CarItem();
    qreal x();

signals:

public slots:
     void setX(qreal x);
private:
    double startpositionVehicule;
    double newpositionVehicule;
    qreal m_x;
    QGraphicsPixmapItem * vehicule;
    QPropertyAnimation * xAnimation;

};

#endif // CAR_H

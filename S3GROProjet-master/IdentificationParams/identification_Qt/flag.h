#ifndef FLAG_H
#define FLAG_H

#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QPropertyAnimation>

class FlagItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x WRITE setX)
    qreal x();

public:
    explicit FlagItem();
    ~FlagItem();
    void setX(qreal x);

signals:

public slots:
private:
    double positionDepot = 500;
    qreal m_x;
    QGraphicsPixmapItem * flag;
};

#endif // FLAG_H

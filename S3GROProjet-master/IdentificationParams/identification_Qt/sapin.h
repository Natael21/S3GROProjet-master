#ifndef SAPIN_H
#define SAPIN_H


#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QGraphicsScene>
#include<QPropertyAnimation>
#include<QtMath>
#include<QtDebug>
class SapinItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal y READ getY WRITE setY)

public:
    explicit SapinItem(double angle,double currentpos,bool sapinlacher);
    ~SapinItem();
    qreal getY();

signals:

public slots:
     void setY(qreal y);
private:
    qreal groundposition;
    qreal x1;
    qreal y1;
    qreal postionSapin;
    double longeurPendule =      40;
    qreal m_y;
    QGraphicsPixmapItem * sapin;
    QPropertyAnimation * yAnimation;

};
#endif // SAPIN_H

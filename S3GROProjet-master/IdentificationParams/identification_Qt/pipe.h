#ifndef PIPE_H
#define PIPE_H

#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QPropertyAnimation>
#include <QJsonObject>
#include <QJsonDocument>

class PipeItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x WRITE setX)
    qreal x();

public:
    explicit PipeItem();
    ~PipeItem();
    void receiveFromSerial(QString);
    void onMessageReceived(QString);
    void setX(qreal x);

signals:

public slots:
private:
    double positionObstacle =    350;
    qreal m_x;
    QGraphicsPixmapItem * pipe;
    QString msgReceived_{""};
    QString msgBuffer_{""};
};

#endif // PIPE_H

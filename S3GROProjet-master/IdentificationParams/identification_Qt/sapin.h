#ifndef SAPIN_H
#define SAPIN_H


#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QPropertyAnimation>
#include <QJsonObject>
#include <QJsonDocument>

class SapinItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x WRITE setX)
    qreal x();

public:
    explicit SapinItem();
    ~SapinItem();
    void receiveFromSerial(QString);
    void onMessageReceived(QString);
    void setQ(qreal Q);

signals:

public slots:
private:
    double anglePendule =        0;
    double longeurPendule =      40;
    qreal m_x;
    QGraphicsPixmapItem * vehicule;
    QPropertyAnimation * xAnimation;
    QString msgReceived_{""};
    QString msgBuffer_{""};

};


#endif // SAPIN_H

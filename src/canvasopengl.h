#ifndef CANVASOPENGL_H
#define CANVASOPENGL_H

// Qt Libs
#include <QObject>
#include <QWidget>
#include <QOpenGLWidget>
#include <QtOpenGL>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPoint>
#include <QVector3D>
#include <QMatrix4x4>

#include <QDebug>
#include <QtGui>
#include <QOpenGLFunctions_1_5>

// C Libs
#include <vector>
#include <stdlib.h>
#include <math.h>

// Custom Structs
#include "structs.h"


using namespace std;

class CanvasOpenGL : public QOpenGLWidget
{
public:
    CanvasOpenGL(QWidget *parent);

    ~CanvasOpenGL();
    GLfloat R, G, B;
    GLfloat Z;
    GLfloat Xluz, Yluz, Zluz;
    GLfloat Xobs, Yobs, Zobs;
    GLfloat intensidadeLuz = 1;
    GLfloat luzAmbiente;
    int desenhando = 2;
    bool isPerspective;
    int modelo;
    int especular;

    void setvMax (GLfloat arg1);
    void setvMin (GLfloat arg1);
    void sethMax (GLfloat arg1);
    void sethMin (GLfloat arg1);
    void setFar (GLfloat arg1);
    void setNear (GLfloat arg1);
    void setDesenhando(int d);
    void setR (double arg1);
    void setG (double arg1);
    void setB (double arg1);
    void setZ (double arg1);
    void undo();
    void reset ();
    void resetRot();
    void clear();
    void setXluz(GLfloat x);
    void setYluz(GLfloat y);
    void setZluz(GLfloat z);
    void setXobs(GLfloat x);
    void setYobs(GLfloat y);
    void setZobs(GLfloat z);
    void setClockwise(bool v);
    void scanline(vector<vector<GLfloat>> vr, float R, float G, float B);
    void set_modelo(int index);
    GLfloat intensidadeDifusa(vector<GLfloat> &pos, vector<GLfloat> &normal, vector<GLfloat> &luz, vector<GLfloat> &vetorLuz, GLfloat intensidadeLuz);
    GLfloat intensidadeEspecular(vector<GLfloat> &pos, vector<GLfloat> &normal, vector<GLfloat> &obs, vector<GLfloat> &vetorObs, vector<GLfloat> &luz,
                                 vector<GLfloat> vetorLuz, GLfloat intensidadeLuz, vector<GLfloat> reflexao, int n);

    void createPolygon();
    void toggleProjection ();
    void setFovY(GLfloat arg1);
    vector<GLfloat> converte_coord(vector<GLint> ponto);
    GLint height, width;
private:
    // VIEWING
    GLfloat
        hMin,   hMax,
        vMin,   vMax,
        novoNear, novoFar,
        aspect, fovY;
    void setParameters();
    void resetParameters();
    void perspectiveGL();
    // OBSERVER
    QVector3D *up, *eye, *center;
    GLfloat xRot, yRot, zRot;
    QPoint lastPos;
    void LookAt();

    // polygon
    Poligono* currentPolygon;
    vector<Poligono*> polygons;

    // SCANLINE
    void drawLine(vector<GLfloat> v1, vector<GLfloat> v2,  vector<GLfloat> color);
    void fillPoligon(Poligono* polygon);

    // ADT
    vector<Edge*> edgeTable;
    vector<GLint> activeEdgeList;

    // HELPERS
    const GLfloat pi = 3.1415926535897932384626433832795;
    GLfloat euclidean (QVector3D, QVector3D);
    bool clockwise = true;


protected:
    // OpenGL
    void initializeGL();
    void resizeGL(GLint w, GLint h);
    void paintGL();

    // Qt Events
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void wheelEvent (QWheelEvent * event);
};


#endif // CANVASOPENGL_H

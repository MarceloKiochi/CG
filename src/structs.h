#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>
#include <QPoint>
#include <QtOpenGL>
using namespace std;

typedef struct Edge {
    GLint yMax;
    GLint x;
    GLfloat slope;
    GLint deltaX;
    GLint deltaY;
} Edge;

//STRUCT DOS NOS DA ET E DA AET
typedef struct no{
    int Ymax;
    int Ymin;
    int Xmax;
    int Xmin;
    int X;
    int XFrac;   //PARTE FRACIONARIA DO X
    int MNum, MDen; //COEFICIENTE ANGULAR DA ARESTA (NUMERADOR E DENOMINADOR)
    vector<GLfloat> normalFace;
    float intensidadeFace;
    vector<GLfloat> normalVminF;
    vector<GLfloat> normalVmaxF;
    float intensidadeVminF;
    float intensidadeVmaxF;
    vector<GLfloat> normalVminT;
    vector<GLfloat> normalVmaxT;
    float intensidadeVminT;
    float intensidadeVmaxT;

    float especularVminT;
    float especularVminF;
    float especularVmaxT;
    float especularVmaxF;
} Node;

typedef struct l{
    GLfloat X;
    GLfloat Y;
    GLfloat Z;
    GLfloat Intensidade;
} Luz;

/*
typedef struct Poligono {
    vector<GLfloat[3]> vertices;
    GLfloat color[4];
} Poligono;
*/

class Poligono{
    public:
    vector<vector<GLfloat>> vertices;
    vector<GLfloat> color;
    Poligono(){
        for(int i=0;i<4;i++){
            this->color.push_back(0);
        }
    }
};

enum ShadingMethods {
    FLAT    = 0,
    GOURAUD = 1,
    PHONG   = 2
};

#endif // STRUCTS_H

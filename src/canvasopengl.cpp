#include "canvasopengl.h"
#include <iostream>
#include <vector>
#include <list>
#include <climits>
#include <cmath>
#include <iomanip>
#include <QPainter>

#include <random>

#define expoente 20

using namespace std;

//FUNCAO DE COMPARACAO DOS NOS PARA ORDENAR PELO X
bool comp(Node A, Node B){
    return A.X < B.X;
}

CanvasOpenGL::CanvasOpenGL(QWidget *parent) : QOpenGLWidget(parent) {
    this->currentPolygon = new Poligono;
    //this->polygons.push_back(this->currentPolygon);
    this->desenhando = 2;
    this->width = 600;
    this->height = 800;
    this->R = 1;
    this->G = 1;
    this->B = 1;
    this->Z = 100;
    this->Xluz = 250;
    this->Yluz = 250;
    this->Zluz = 100;
    this->Xobs = 100;
    this->Yobs = 100;
    this->Zobs = 100;
    this->modelo = FLAT;
    this->luzAmbiente=0.1;
    this->especular=1;
}

CanvasOpenGL::~CanvasOpenGL() {}
void CanvasOpenGL::toggleProjection() {
    this->isPerspective = !this->isPerspective;

    this->update();
}

void CanvasOpenGL::initializeGL() {
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_PROGRAM_POINT_SIZE);

    this->isPerspective = false;

    this->aspect = this->width/this->height;

    this->fovY = 100.0;

    this->resetParameters();

    glViewport(0, 0, this->width, this->height);
}

void CanvasOpenGL::LookAt()
{

}

template<class T>
void swapV(T& a, T& b){
    T aux = b;
    b = a;
    a = aux;
}

// V1[0] TEM QUE SER <= V2[0]
void CanvasOpenGL::drawLine(vector<GLfloat> v1, vector<GLfloat> v2, vector<GLfloat> color)
{
    GLfloat x1, x2, y1, y2;
    x1 = v1[0];
    x2 = v2[0];
    y1 = v1[1];
    y2 = v2[1];

    if(x2 - x1 == 0){
        glBegin(GL_POINTS);
        //glColor4f(color[0], color[1], color[2], color[3]);
        if(y2 > y1){
            for(float i=y1; i<=y2; i++){
                glVertex3f(x1, i, 0);
            }
        } else {
            for(float i=y2; i<=y1; i++){
                glVertex3f(x1, i, 0);
            }
        }
        glEnd();
    } else if(y2 - y1 == 0){
        glBegin(GL_POINTS);
        //glColor4f(color[0], color[1], color[2], color[3]);
        for(float i=x1; i<=x2; i++){
            glVertex3f(i, y1, 0);
        }
        glEnd();
    } else {
        GLfloat mirror = 1;
        if(y1 > y2){
            mirror = -1;
            swapV(x1, x2);
            swapV(y1, y2);
            x2 = (-1) * x2;
            x1 = (-1) * x1;
        }
        bool inverted = false;
        if(y2 - y1 > x2 - x1){
            inverted = true;
            swapV(x1, y1);
            swapV(x2, y2);
        }

        float dy = y2 - y1;
        float dx = x2 - x1;
        float d = 2 * dy - dx;
        float DE = dy;
        float DNE = dy - dx;
        GLfloat y = y1;
        for(GLfloat x = x1; x<=x2; x++){
            //glColor4f(color[0], color[1], color[2], color[3]);
            glBegin(GL_POINTS);
            if(inverted){
                glVertex3f(mirror * y, x, 0);
            } else{
                glVertex3f(mirror * x, y, 0);
            }
            if(d > 0){
                y++;
                d += DNE;
            } else {
                d += DE;
            }
            glEnd();
        }
    }
}

void CanvasOpenGL::fillPoligon(Poligono* polygon)
{
    (void) polygon;
}

GLfloat CanvasOpenGL::euclidean(QVector3D p1, QVector3D p2)
{
    GLfloat result = GL_DOUBLE;
    (void) p1;
    (void) p2;
    return result;
}

void drawDot(GLfloat x, GLfloat y, GLfloat z){
    glPointSize(5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_POINTS);
    glVertex3f(x, y, z);
    glEnd();
    glPointSize(1);
}

vector<GLfloat> CanvasOpenGL::converte_coord(vector<GLint> ponto){
    vector<GLfloat> r;
    r.push_back((GLfloat)ponto[0]-this->hMax);
    r.push_back(this->vMax-(GLfloat)ponto[1]);
    r.push_back((GLfloat)ponto[2]);
    return r;
}

//calcula a norma de um vetor
double norma_vetor(vector<GLfloat> &vetor){
    double norma=0;
    for(int i=0;i<3;i++){
        norma+=(vetor[i]*vetor[i]);
    }
    norma = sqrt(norma);
    return norma;
}

//calcula o vetor normal de uma face, fazendo produto vetorial entre dois vetores
vector<GLfloat> vetor_normal(vector<GLfloat> v1, vector<GLfloat> v2){
    vector<GLfloat> normal(3, 0);

    normal[0] = v1[1]*v2[2] - v2[1]*v1[2];
    normal[1] = (v1[0]*v2[2] - v2[0]*v1[2])*(-1);
    normal[2] = v1[0]*v2[1] - v2[0]*v1[1];

       //normaliza o vetor normal
    double norma = norma_vetor(normal);
    for(int i=0;i<3;i++){
        normal[i]/=norma;
    }
    return normal;
}

//calcula o cosseno do angulo entre dois vetores, fazendo produto escalar entre os vetores
double cos_entre_vetores(vector<GLfloat> &v1, vector<GLfloat> &v2){
    double p_escalar=0;
    for(int i=0;i<3;i++){
        p_escalar+=(v1[i]*v2[i]);
    }

    double cos;
    double norma1 = norma_vetor(v1);
    double norma2 = norma_vetor(v2);
    cos = p_escalar/(norma1*norma2);
    return cos;
}

//calcula o seno do angulo entre dois vetores a partir do calculo do cosseno
double sen_entre_vetores(vector<GLfloat> v1, vector<GLfloat> v2){
    double cos = cos_entre_vetores(v1, v2);
    double sen = 1 - (cos*cos);
    sen = sqrt(sen);
    return sen;
}

//forma um vetor normalizado entre dois pontos
//resultado eh colocado em um vetor passado por parametro para melhorar o desempenho
void forma_vetor(vector<GLfloat> &vetor, vector<GLfloat> &p1, vector<GLfloat> &p2){
    float norma=0;
    for(int i=0;i<3;i++){
        vetor[i] = p2[i] - p1[i];
        norma+=pow(vetor[i], 2);
    }
    norma = sqrt(norma);
    for(int i=0;i<3;i++){
        vetor[i]/=norma;
    }
}

vector<GLfloat> eixoX(){
    vector<GLfloat> x(3, 0);
    x[0]=1;
    return x;
}

vector<GLfloat> eixoY(){
    vector<GLfloat> x(3, 0);
    x[1]=1;
    return x;
}

vector<GLfloat> eixoZ(){
    vector<GLfloat> x(3, 0);
    x[2]=1;
    return x;
}

//calcula o vetor resultado da media entre tres vetores passados
vector<GLfloat> media_vetores(vector<GLfloat> v1, vector<GLfloat> v2, vector<GLfloat> v3){
    vector<GLfloat> r(3);
    float norma=0;
    for(int i=0;i<3;i++){
        r[i] = (v1[i]+v2[i]+v3[i])/3;
        norma+=pow(r[i], 2);
    }
    norma = sqrt(norma);
    for(int i=0;i<3;i++){
        r[i]/=norma;
    }
    return r;
}

//calcula o valor da intensidade difusa em um ponto. Soma com a intensidade da luz ambiente
//pos: ponto a ser calculado
//normal: vetor normal da face no ponto
//luz: posicao da fonte de luz
//vetorLuz: recebera vetor do ponto ateh a luz
GLfloat CanvasOpenGL::intensidadeDifusa(vector<GLfloat> &pos, vector<GLfloat> &normal, vector<GLfloat> &luz, vector<GLfloat> &vetorLuz, GLfloat intensidadeLuz){
    forma_vetor(vetorLuz, pos, luz);
    double cos = cos_entre_vetores(vetorLuz, normal);
    GLfloat intensidade = intensidadeLuz*cos;
    if(intensidade<0){
        intensidade=0;
    }
    intensidade+=this->luzAmbiente;
    return intensidade;
}

//calcula a intensidade especular em um ponto
//obs: posicao do observador
//vetorObs: vetor do ponto ateh o observador
//reflexao: recebera o vetor de reflexao da luz
//n: expoente de reflexao especular
GLfloat CanvasOpenGL::intensidadeEspecular(vector<GLfloat> &pos, vector<GLfloat> &normal, vector<GLfloat> &obs, vector<GLfloat> &vetorObs, vector<GLfloat> &luz,
                             vector<GLfloat> vetorLuz, GLfloat intensidadeLuz, vector<GLfloat> reflexao, int n){
    if(this->especular==false){
        return 0;
    }
    forma_vetor(vetorLuz, pos, luz);
    if(cos_entre_vetores(normal, vetorLuz) <= 0){   //verifica se a luz atinge o ponto
        return 0;
    }
    forma_vetor(vetorObs, pos, obs);
    if(cos_entre_vetores(normal, obs) <= 0){    //verifica se o observador enxerga o ponto
        return 0;
    }

    //calcula o vetor de reflexao da luz
    float escalar=0;
    for(int i=0;i<3;i++){
        escalar+=(vetorLuz[i]*normal[i]);
    }
    for(int i=0;i<3;i++){
        reflexao[i] = 2*escalar*normal[i] - vetorLuz[i];
    }

    double cos = cos_entre_vetores(reflexao, obs);
    if(cos<=0){
        return 0;
    }

    GLfloat intensidade = intensidadeLuz*pow(cos, n);
    if(intensidade<0){
        intensidade=0;
    }
    return intensidade;
}

//calcula vetor interpolado entre dois vetores para o algoritmo phong
//normal: vetor resultante
//n1 e n2: normais dos extremos
//pos: posicao a ser calculada
//max e min: posicao dos extremos
void interp_phong(vector<GLfloat> &normal, vector<GLfloat> &n1, vector<GLfloat> &n2, int pos, int max, int min){
    for(int i=0;i<3;i++){
        normal[i] = n1[i] - ((n1[i]-n2[i])*(max-pos)/(max-min));
    }
}

void cria_ponto_ou_vetor(vector<GLfloat> &ponto, GLfloat p1, GLfloat p2, GLfloat p3){
    ponto[0] = p1;
    ponto[1] = p2;
    ponto[2] = p3;
}

//preenche todas as faces do prisma
//scanline da face frontal adaptado para preencher as outras faces tambem
//faz os 3 modelos de tonalizacao
//vr: vertices do poligono a ser tranformado em prisma
//os vertices possuem o Z a ser extrapolado
//a face frontal fica em Z=0
//R, G e B: RGB da cor do prisma
void CanvasOpenGL::scanline(vector<vector<GLfloat>> vr, float R, float G, float B){

    glPointSize(1.5);   //aumenta tamanho do pixel
                        //para resolver efeito peneira que estava dando

//    if((int)vr.size()==4){
//        vr[0][0] = -30;
//        vr[0][1] = 0;
//        vr[0][2] = -100;
//        vr[1][0] = 30;
//        vr[1][1] = 0;
//        vr[1][2] = -100;
//        vr[2][0] = 30;
//        vr[2][1] = 30;
//        vr[2][2] = -100;
//        vr[3][0] = -30;
//        vr[3][1] = 30;
//        vr[3][2] = -100;
//    }

    vector<list<Node>> ET; //cria ET como vetor de listas
    list<Node> AET; //NO DA ET

    int Hmax = 0, Hmin = INT_MAX;   //limites verticais do poligono
    int Lmin = INT_MAX, Lmax = 0;   //limites horizontais do poligono

    //vetor que possui a posicao da luz
    vector<GLfloat> luz(3);
    cria_ponto_ou_vetor(luz, this->Xluz, this->Yluz, this->Zluz);

    //vetor que possui a posicao do observador
    vector<GLfloat> obs(3);
    cria_ponto_ou_vetor(obs, this->Xobs, this->Yobs, this->Zobs);

    //coloca um ponto amarelo na posicao da luz
    //e um ponto vermelho na posicao do observador
    glBegin(GL_POINTS);
    glColor3f(1, 1, 0);
    glVertex3i(luz[0], luz[1], luz[2]);
    glColor3f(1, 0, 0);
    glVertex3i(obs[0], obs[1], obs[2]);
    glEnd();

    //calcula limites de altura e largura do poligono
    for (int i=0;i<(int)vr.size();i++) {
        if(vr[i][1] > Hmax){
            Hmax = vr[i][1];
        }
        if(vr[i][1] < Hmin){
            Hmin = vr[i][1];
        }
        if(vr[i][0] > Lmax){
            Lmax = vr[i][0];
        }
        if(vr[i][0] < Lmin){
            Lmin = vr[i][0];
        }

    }

    //insere listas necessarias na ET
    for(int i=0; i<Hmax-Hmin+1; i++){
        list<Node> lista;
        ET.push_back(lista);
    }

    vector<GLfloat> vetorLuz(3);
    vector<GLfloat> vetorObs(3);
    vector<GLfloat> reflexaoLuz(3);
    vector<GLfloat> v1(3);
    vector<GLfloat> v2(3);
    vector<GLfloat> meioDaFace(3);
    vector<GLfloat> vetorZ = eixoZ();
    vector<GLfloat> normalFace;
    vector<GLfloat> normalFace2;
    vector<GLfloat> menosZ(3);
    vector<GLfloat> normalv1T;
    vector<GLfloat> normalv1F;
    vector<GLfloat> normalv2T;
    vector<GLfloat> normalv2F;

    //cria nohs das arestas da ET
    //os nos possuem (alem dos elementos normais da ET): os vertices da aresta, a normal da face formada pela extrapolacao da aresta,
    //                  a intensidade difusa dessa face, as normais dos 4 vertices (2 da aresta frontal e 2 da traseira do prisma),
    //                  as intensidades difusa e especular de cada um dos 4 vertices
    int t = (int)vr.size();
    for (int i=0;i<t;i++) { //percorre os vertices para fazer as arestas
        Node n;

        forma_vetor(v1, vr[i], vr[(i+1)%t]);    //v1 recebe vetor entre ponto atual e proximo ponto

        normalFace = vetor_normal(eixoZ(), v1); //normal da face lateral referente a aresta entre esses pontos
                                                //essa aresta e o eixo Z formam o plano dessa face

        float cos;

        n.normalFace = normalFace;


        float intensidadev1T=0, intensidadev2T=0, intensidadev1F=0, intensidadev2F=0;
        float especularv1T=0, especularv1F=0, especularv2T=0, especularv2F=0;

        if(this->modelo==FLAT){
            //Flat:
            // meioDaFace reecbe o ponto central da face lateral (da aresta em questao)
            cria_ponto_ou_vetor(meioDaFace, (vr[i][0]+vr[(i+1)%t][0])/2, (vr[i][1]+vr[(i+1)%t][1])/2,  vr[i][2]/2);

            //vetorLuz recebe vetor entre o ponto central e a luz
            forma_vetor(vetorLuz, meioDaFace, luz);
            cos = cos_entre_vetores(normalFace, vetorLuz);

            //calcula intensidade difusa da face
            n.intensidadeFace = intensidadeDifusa(meioDaFace, normalFace, luz, vetorLuz, this->intensidadeLuz);
        }
        else{
            //Gouraud ou Phong:
            //intensidade = intensidade difusa
            //especular = intensidade especular
            //v1 = vertice atual
            //v2 = proximo vertice que faz aresta com o atual
            //T = vertice traseiro (extrapolado)
            //F = vertice frontal

            forma_vetor(v2, vr[(i+t-1)%t], vr[i]);  //v2 recebe vetor entre vertice anterior e vertice atual
            normalFace2 = vetor_normal(eixoZ(), v2);    //vetor normal da face lateral referente a essa aresta (entre vertice anterior e atual)
                                                        //o plano dessa face eh formada pelo eixo Z e pelo vetor v2

            normalv1T = media_vetores(normalFace, normalFace2, menosZ); //vetor normal do vertice atual
                                                                        //esse eh do vertice traseiro do prisma

            normalv1F = normalv1T;  //vetor normal do vertice frontal
            normalv1F[2]*=(-1);     //muda apenas a componente Z

            forma_vetor(v2, vr[(i+1)%t], vr[(i+2)%t]);  //v2 recebe vetor da aresta entre os proximos dois vertices
            normalFace2 = vetor_normal(eixoZ(), v2);    //normal da face da aresta
            cria_ponto_ou_vetor(menosZ, 0, 0, -1);      //vetor menos Z
            normalv2T = media_vetores(normalFace, normalFace2, menosZ); //normal do proximo vertice (vertice que forma aresta com o vertice atual)
            normalv2F = normalv2T;
            normalv2F[2]*=(-1);     //normal do vertice frontal inverte apenas o eixo Z


            //calcula intensidades difusa e especular dos vertices
            vector<GLfloat> vrFront;
            //intensidades do vertice traseiro atual
            intensidadev1T = intensidadeDifusa(vr[i], normalv1T, luz, vetorLuz, this->intensidadeLuz);
            especularv1T = intensidadeEspecular(vr[i], normalv1T, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);

            //intensidades do vertice traseiro posterior
            intensidadev2T = intensidadeDifusa(vr[(i+1)%t], normalv2T, luz, vetorLuz, this->intensidadeLuz);
            especularv2T = intensidadeEspecular(vr[(i+1)%t], normalv2T, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);

            //intensidades do vertice frontal atual
            vrFront = vr[i];
            vrFront[2] = 0;
            intensidadev1F = intensidadeDifusa(vrFront, normalv1F, luz, vetorLuz, this->intensidadeLuz);
            especularv1F = intensidadeEspecular(vrFront, normalv1F, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);

            //intensidades do vertice frontal posterior
            vrFront = vr[(i+1)%t];
            vrFront[2]=0;
            intensidadev2F = intensidadeDifusa(vrFront, normalv2F, luz, vetorLuz, this->intensidadeLuz);
            especularv2F = intensidadeEspecular(vrFront, normalv2F, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);
        }

        vector<GLfloat> Max(2, 0);
        vector<GLfloat> Min(2, 0);
        //verifica qual dos vertices eh min ou max
        if(vr[i][1] > vr[(i+1)%t][1]){
            Max[1] = (vr[i][1]);
            Max[0] = (vr[i][0]);
            Min[1] = (vr[(i+1)%t][1]);
            Min[0] = (vr[(i+1)%t][0]);
            n.normalVmaxT = normalv1T;
            n.normalVminT = normalv2T;
            n.normalVmaxF = normalv1F;
            n.normalVminF = normalv2F;
            n.intensidadeVmaxT = intensidadev1T;
            n.intensidadeVminT = intensidadev2T;
            n.intensidadeVmaxF = intensidadev1F;
            n.intensidadeVminF = intensidadev2F;
            n.especularVmaxF = especularv1F;
            n.especularVminF = especularv2F;
            n.especularVmaxT = especularv1T;
            n.especularVminT = especularv2T;
        } else{
            Max[1] = (vr[(i+1)%t][1]);
            Max[0] = (vr[(i+1)%t][0]);
            Min[1] = (vr[i][1]);
            Min[0] = (vr[i][0]);
            n.normalVmaxT = normalv2T;
            n.normalVminT = normalv1T;
            n.normalVmaxF = normalv2F;
            n.normalVminF = normalv1F;
            n.intensidadeVmaxT = intensidadev2T;
            n.intensidadeVminT = intensidadev1T;
            n.intensidadeVmaxF = intensidadev2F;
            n.intensidadeVminF = intensidadev1F;
            n.especularVmaxF = especularv2F;
            n.especularVminF = especularv1F;
            n.especularVmaxT = especularv2T;
            n.especularVminT = especularv1T;
        }

        n.Ymax = Max[1];
        n.Ymin = Min[1];
        n.Xmax = Max[0];
        n.Xmin = Min[0];    //Xmin eh o X da coordenada do vertice min
        n.X = n.Xmin;       //X eh o X a ser incrementado no scanline
        n.XFrac = 0;
        n.MNum = Max[0] - Min[0];
        n.MDen = Max[1] - Min[1];

        //insere na ET
        ET[Min[1]-Hmin].push_back(n);
    }

    int nivel = 0;

    vector<GLfloat> normal1(3), normal2(3), normal3(3);
    vector<GLfloat> ponto(3);
    vector<GLfloat> normalET(3), normalEF(3), normalDT(3), normalDF(3);
    float intensidade1, intensidade2, intensidade3;
    float especular1, especular2, especular3;
    int min, max;
    float intensidadeMinF, intensidadeMaxF, intensidadeMinT, intensidadeMaxT;
    float especularMinF, especularMaxF, especularMinT, especularMaxT;

    //coloca as arestas na AET e pinta as faces
    while(!AET.empty() or nivel < (int)ET.size()){

        //insere as arestas no nivel atual da AET
        if( nivel < (int) ET.size()){
            for(list<Node>::iterator j=ET[nivel].begin(); j!=ET[nivel].end(); j++){
                AET.push_back(*j);
            }
        }

        //tira da AET as arestas com Ymin igual ao nivel atual
        list<Node>::iterator j = AET.begin();
        while(j != AET.end()){
            if(nivel+Hmin == j->Ymax){
                if(j->Ymin == j->Ymax){ //se for uma aresta completamente horizontal, precisa pintar a face do plano horizontal

                    //dependendo dos valores de Xmin e Xmax, determina os valores dos extremos para a interpolacao
                    //intensidade = intensidade difusa
                    //especular = intensidade especular
                    //F = vertice frontal
                    //T = vertice traseiro
                    if(j->Xmin < j->Xmax){
                        //extremos da posicao em X
                        min = j->Xmin;
                        max = j->Xmax;

                        //extremos da intensidade difusa (frontal e traseira)
                        intensidadeMinF = j->intensidadeVminF;
                        intensidadeMinT = j->intensidadeVminT;
                        intensidadeMaxF = j->intensidadeVmaxF;
                        intensidadeMaxT = j->intensidadeVmaxT;

                        //intensidade especular (frontal e traseira)
                        especularMinF = j->especularVminF;
                        especularMinT = j->especularVminT;
                        especularMaxF = j->especularVmaxF;
                        especularMaxT = j->especularVmaxT;

                        //normais (frontal e traseira)
                        //E = normal do lado esquerdo
                        //D = normal do lado direito
                        normalEF = j->normalVminF;
                        normalET = j->normalVminT;
                        normalDF = j->normalVmaxF;
                        normalDT = j->normalVmaxT;
                    } else {
                        max = j->Xmin;
                        min = j->Xmax;
                        intensidadeMaxF = j->intensidadeVminF;
                        intensidadeMaxT = j->intensidadeVminT;
                        intensidadeMinF = j->intensidadeVmaxF;
                        intensidadeMinT = j->intensidadeVmaxT;

                        especularMaxF = j->especularVminF;
                        especularMaxT = j->especularVminT;
                        especularMinF = j->especularVmaxF;
                        especularMinT = j->especularVmaxT;

                        normalDF = j->normalVminF;
                        normalDT = j->normalVminT;
                        normalEF = j->normalVmaxF;
                        normalET = j->normalVmaxT;
                    }

                    if(this->modelo==FLAT){
                        //intensidade da face eh a mesam para todos os pontos
                        intensidade1 = j->intensidadeFace;
                        glColor3f(R*intensidade1, G*intensidade1, B*intensidade1);
                        glBegin(GL_POINTS);
                        for(int i=min;i<=max;i++){
                            for(int k=0;k>=vr[0][2];k--){
                                glVertex3i(i, nivel+Hmin, k);
                            }
                        }
                        glEnd();
                    }
                    else if(this->modelo == GOURAUD){
                        glBegin(GL_POINTS);
                        for(int i=min;i<=max;i++){
                            //interpola intensidades no eixo X
                            intensidade1 = intensidadeMaxF - ((intensidadeMaxF-intensidadeMinF)*(max-i)/(max-min));
                            especular1 = especularMaxF - ((especularMaxF-especularMinF)*(max-i)/(max-min));
                            intensidade2 = intensidadeMaxT - ((intensidadeMaxT-intensidadeMinT)*(max-i)/(max-min));
                            especular2 = especularMaxT - ((especularMaxT-especularMinT)*(max-i)/(max-min));

                            for(int k=0;k>=vr[0][2];k--){   //vr[0][2] contem o Z da face traseira do prisma
                                //interpola no eixo Z usando as outras duas como extremos
                                intensidade3 = intensidade1 - ((intensidade1-intensidade2)*(0-k)/(0-vr[0][2]));
                                especular3 = especular1 - ((especular1-especular2)*(0-k)/(0-vr[0][2]));
                                glColor3f(R*intensidade3+especular3, G*intensidade3+especular3, B*intensidade3+especular3);
                                glBegin(GL_POINTS);
                                glVertex3i(i, nivel+Hmin, k);
                            }
                        }
                        glEnd();
                    }
                    else if(this->modelo==PHONG){
                        glBegin(GL_POINTS);

                        for(int i=min;i<=max;i++){
                            //interpola as normais no eixo X
                            //normais interpoladas em normal1 e normal2
                            interp_phong(normal1, normalDF, normalEF, i, max, min);
                            interp_phong(normal2, normalDT, normalET, i, max, min);

                            for(int k=0;k>=vr[0][2];k--){
                                //interpola a normal no eixo Z com as normais achadas
                                interp_phong(normal3, normal1, normal2, k, 0, vr[0][2]);
                                cria_ponto_ou_vetor(ponto, i, nivel+Hmin, k);   //ponto atual

                                //calcula intensidades do ponto atual
                                intensidade3 = intensidadeDifusa(ponto, normal3, luz, vetorLuz, this->intensidadeLuz);
                                especular3 = intensidadeEspecular(ponto, normal3, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);
                                glColor3f(R*intensidade3+especular3, G*intensidade3+especular3, B*intensidade3+especular3);
                                glBegin(GL_POINTS);
                                glVertex3i(i, nivel+Hmin, k);
                            }
                        }
                        glEnd();
                    }
                }


                j = AET.erase(j);
            } else {
                j++;
            }
        }

        AET.sort(comp); //ordena os nos pelo X

        int paridade = 0;
        int limiteEsquerdo=0, limiteDireito=0;

        float intensidadeEsquerda,intensidadeDireita;        
        float intensidadeEsquerdaF=0, intensidadeDireitaF=0, intensidadeEsquerdaT=0, intensidadeDireitaT=0;
        float especularEsquerdaF=0, especularDireitaF=0, especularEsquerdaT=0, especularDireitaT=0;        

        //percorre a AET preenchendo as faces frontal, traseira e laterais
        for(list<Node>::iterator i=AET.begin(); i!=AET.end(); i++){

            //se paridade==0, define limite esquerdo (onde comeca a pintar)
            if(paridade == 0){
                limiteEsquerdo = i->X;
                intensidadeEsquerda = i->intensidadeFace;   //intensidade da face esquerda para o flat
                if(this->modelo==GOURAUD){
                    //gouraud:
                    if(i->Ymin == nivel+Hmin){  //se for um vertice, pega intensidades dos vertices
                        intensidadeEsquerdaT = i->intensidadeVminT;
                        intensidadeEsquerdaF = i->intensidadeVminF;

                        especularEsquerdaT = i->especularVminT;
                        especularEsquerdaF = i->especularVminF;
                    }
                    else{   //se nao, interpola as intensidades dos vertices para achar as intensidades laterais
                        intensidadeEsquerdaT = i->intensidadeVmaxT - ((i->intensidadeVmaxT-i->intensidadeVminT)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));
                        intensidadeEsquerdaF = i->intensidadeVmaxF - ((i->intensidadeVmaxF-i->intensidadeVminF)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));

                        especularEsquerdaT = i->especularVmaxT - ((i->especularVmaxT-i->especularVminT)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));
                        especularEsquerdaF = i->especularVmaxF - ((i->especularVmaxF-i->especularVminF)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));
                    }
                }
                else if(this->modelo==PHONG){
                    //phong:
                    //interpola as normais
                    interp_phong(normalEF, i->normalVmaxF, i->normalVminF, nivel+Hmin, i->Ymax, i->Ymin);
                    interp_phong(normalET, i->normalVmaxT, i->normalVminT, nivel+Hmin, i->Ymax, i->Ymin);
                }

                //se a parte fracionaria for maior que 0, aumenta 1 (arredonda pra cima)
                if(i->XFrac > 0){
                    limiteEsquerdo++;
                }
                paridade = 1;
            }
            //se paridade == 1, define limite direito (onde para de pintar)
            else {  //mesma coisa para o lado direito
                limiteDireito = i->X;
                intensidadeDireita = i->intensidadeFace;
                if(this->modelo == GOURAUD){
                    if(i->Ymin == nivel+Hmin){
                        intensidadeDireitaT = i->intensidadeVminT;
                        intensidadeDireitaF = i->intensidadeVminF;

                        especularDireitaT = i->especularVminT;
                        especularDireitaF = i->especularVminF;
                    }
                    else{
                        intensidadeDireitaT = i->intensidadeVmaxT - ((i->intensidadeVmaxT-i->intensidadeVminT)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));
                        intensidadeDireitaF = i->intensidadeVmaxF - ((i->intensidadeVmaxF-i->intensidadeVminF)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));

                        especularDireitaT = i->especularVmaxT - ((i->especularVmaxT-i->especularVminT)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));
                        especularDireitaF = i->especularVmaxF - ((i->especularVmaxF-i->especularVminF)*(i->Ymax-(nivel+Hmin))/(i->Ymax-i->Ymin));
                    }
                }
                else if(this->modelo == PHONG){
                    interp_phong(normalDF, i->normalVmaxF, i->normalVminF, nivel+Hmin, i->Ymax, i->Ymin);
                    interp_phong(normalDT, i->normalVmaxT, i->normalVminT, nivel+Hmin, i->Ymax, i->Ymin);
                }

                //se parte fracionaria menor ou igual a 0, diminui 1 (arredonda pra baixo)
                if(i->XFrac <= 0 and limiteDireito!=limiteEsquerdo){
                    limiteDireito--;
                }

                //pinta o nivel atual de todas as faces
                if(this->modelo==FLAT){
                    //flat:

                    //intensidade da face frontal:
                    cria_ponto_ou_vetor(meioDaFace, (Lmin+Lmax)/2, (Hmin+Hmax)/2, 0);
                    intensidade1 = intensidadeDifusa(meioDaFace, vetorZ, luz, vetorLuz, this->intensidadeLuz);

                    //face traseira
                    vetorZ[2]=-1;
                    meioDaFace[2] = vr[0][2];
                    intensidade2 = intensidadeDifusa(meioDaFace, vetorZ, luz, vetorLuz, this->intensidadeLuz);
                    vetorZ[2]=1;

                    glBegin(GL_POINTS);
                    //pinta faces frontal e traseira
                    for(int k=limiteEsquerdo+1;k<limiteDireito;k++){
                        glColor3f(R*intensidade1, G*intensidade1, B*intensidade1);
                        glVertex3i(k, nivel+Hmin, 0);
                        glColor3f(R*intensidade2, G*intensidade2, B*intensidade2);
                        glVertex3i(k, nivel+Hmin, vr[0][2]);
                    }

                    //pinta faces laterais (pontos extrapolados em Z)
                    for(int k=-1;k>vr[0][2];k--){
                        glColor3f(R*intensidadeEsquerda, G*intensidadeEsquerda, B*intensidadeEsquerda);
                        glVertex3i(limiteEsquerdo, nivel+Hmin, k);
                        glColor3f(R*intensidadeDireita, G*intensidadeDireita, B*intensidadeDireita);
                        glVertex3i(limiteDireito, nivel+Hmin, k);
                    }
                    glEnd();
                }
                else if(this->modelo == GOURAUD){
                    //gouraud:
                    glBegin(GL_POINTS);

                    //pinta faces frontal e traseira
                    for(int k = limiteEsquerdo+1;k<limiteDireito;k++){
                        //interpola intensidades no eixo X
                        //face frontal
                        intensidade1 = intensidadeDireitaF - ((intensidadeDireitaF-intensidadeEsquerdaF)*(limiteDireito-k)/(limiteDireito-limiteEsquerdo));
                        especular1 = especularDireitaF - ((especularDireitaF-especularEsquerdaF)*(limiteDireito-k)/(limiteDireito-limiteEsquerdo));
                        glColor3f(R*intensidade1+especular1, G*intensidade1+especular1, B*intensidade1+especular1);
                        glVertex3i(k, nivel+Hmin, 0);

                        //face traseira
                        intensidade2 = intensidadeDireitaT - ((intensidadeDireitaT-intensidadeEsquerdaT)*(limiteDireito-k)/(limiteDireito-limiteEsquerdo));
                        especular2 = especularDireitaT - ((especularDireitaT-especularEsquerdaT)*(limiteDireito-k)/(limiteDireito-limiteEsquerdo));
                        glColor3f(R*intensidade2+especular2, G*intensidade2+especular2, B*intensidade2+especular2);
                        glVertex3i(k, nivel+Hmin, vr[0][2]);
                    }
                    // Pinta faces laterais (pontos extrapolados em Z)
                    for(int k=-1;k>vr[0][2];k--){   //vr[0][2] contem Z da face traseira
                        //esquerda
                         intensidade1 = intensidadeEsquerdaF - ((intensidadeEsquerdaF-intensidadeEsquerdaT)*(0-k)/(0-vr[0][2]));
                         especular1 = especularEsquerdaF - ((especularEsquerdaF-especularEsquerdaT)*(0-k)/(0-vr[0][2]));
                         glColor3f(R*intensidade1+especular1, G*intensidade1+especular1, B*intensidade1+especular1);
                         glVertex3i(limiteEsquerdo, nivel+Hmin, k);

                         //direita
                         intensidade2 = intensidadeDireitaF - ((intensidadeDireitaF-intensidadeDireitaT)*(0-k)/(0-vr[0][2]));
                         especular2 = especularDireitaF - ((especularDireitaF-especularDireitaT)*(0-k)/(0-vr[0][2]));
                         glColor3f(R*intensidade2+especular2, G*intensidade2+especular2, B*intensidade2+especular2);
                         glVertex3i(limiteDireito, nivel+Hmin, k);
                    }
                    glEnd();

                }
                else if(this->modelo == PHONG){
                    //phong
                    glBegin(GL_POINTS);

                    //faces frontal e traeira
                    for(int k = limiteEsquerdo+1;k<limiteDireito;k++){
                        //interpola normais e calcula as intensidades

                        //normais da face frontal
                        interp_phong(normal1, normalDF, normalEF, k, limiteDireito, limiteEsquerdo);
                        cria_ponto_ou_vetor(ponto, k, nivel+Hmin, 0);
                        //intensidades da face frontal
                        intensidade1 = intensidadeDifusa(ponto, normal1, luz, vetorLuz, this->intensidadeLuz);
                        especular1 = intensidadeEspecular(ponto, normal1, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);
                        glColor3f(R*intensidade1+especular1, G*intensidade1+especular1, B*intensidade1+especular1);
                        glVertex3i(k, nivel+Hmin, 0);

                        //face traseira
                        interp_phong(normal2, normalDT, normalET, k, limiteDireito, limiteEsquerdo);
                        ponto[2] = vr[0][2];

                        intensidade2 = intensidadeDifusa(ponto, normal2, luz, vetorLuz, this->intensidadeLuz);
                        especular2 = intensidadeEspecular(ponto, normal2, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);
                        glColor3f(R*intensidade2+especular2, G*intensidade2+especular2, B*intensidade2+especular2);
                        glVertex3i(k, nivel+Hmin, vr[0][2]);
                    }
                    // pinta faces laterais (pontos extrapolados em Z)
                    for(int k=-1;k>vr[0][2];k--){
                         interp_phong(normal1, normalEF, normalET, k, 0, vr[0][2]);
                         cria_ponto_ou_vetor(ponto, limiteEsquerdo, nivel+Hmin, k);
                         intensidade1 = intensidadeDifusa(ponto, normal1, luz, vetorLuz, this->intensidadeLuz);
                         especular1 = intensidadeEspecular(ponto, normal1, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);

                         glColor3f(R*intensidade1+especular1, G*intensidade1+especular1, B*intensidade1+especular1);
                         glVertex3i(limiteEsquerdo, nivel+Hmin, k);

                         interp_phong(normal2, normalDF, normalDT, k, 0, vr[0][2]);
                         ponto[0] = limiteDireito;
                         intensidade2 = intensidadeDifusa(ponto, normal2, luz, vetorLuz, this->intensidadeLuz);
                         especular2 = intensidadeEspecular(ponto, normal2, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);

                         glColor3f(R*intensidade2+especular2, G*intensidade2+especular2, B*intensidade2+especular2);
                         glVertex3i(limiteDireito, nivel+Hmin, k);
                    }
                    glEnd();
                }
                paridade = 0;
            }
        }

        //muda o X dos pontos
        //pinta o plano do nivel para fechar o prisma
        int Xmin, Xmax, Ymin, Ymax;
        for(list<Node>::iterator i=AET.begin(); i!=AET.end(); i++){
            //soma na parte fracionaria
            i->XFrac += i->MNum;

            if(i->XFrac > 0){   //se a aresta sobe para a direita
                int x1 = i->X;
                //aumenta parte inteira enquanto parte fracionaria eh maior ou igual a 1
                while(i->XFrac >= i->MDen){
                    i->XFrac -= i->MDen;
                    i->X++;
                }

                //define extremos para interpolacao
                //extremos em X e Y
                Xmin = i->Xmin;
                Xmax = i->Xmax;
                Ymin = i->Ymin;
                Ymax = i->Ymax;

                //intervalo a ser pintado
                limiteEsquerdo = x1;
                limiteDireito = i->X;

                //extremos de intensidades para o gouraud
                //F = intensidade do vertice frontal
                //T = intensidade do vertice traseiro
                intensidadeMinF = i->intensidadeVminF;
                intensidadeMinT = i->intensidadeVminT;
                intensidadeMaxF = i->intensidadeVmaxF;
                intensidadeMaxT = i->intensidadeVmaxT;
                especularMinF = i->especularVminF;
                especularMinT = i->especularVminT;
                especularMaxF = i->especularVmaxF;
                especularMaxT = i->especularVmaxT;

                //extremos de normais para o phong
                //E = normal da esquerda
                //D = normal da direita
                normalEF = i->normalVminF;
                normalET = i->normalVminT;
                normalDF = i->normalVmaxF;
                normalDT = i->normalVmaxT;

            } else if(i->XFrac <= 0){   //se a aresta sobe para a esquerda
                int x1 = i->X;
                //diminui a parte inteira enquanto a parte fracionaria for menor ou igual a -1

                while(abs(i->XFrac) >= i->MDen){
                    i->XFrac += i->MDen;
                    i->X--;
                }
                //define os extremos para interpolacao
                Xmax = i->Xmin;
                Xmin = i->Xmax;
                Ymax = i->Ymin;
                Ymin = i->Ymax;
                limiteEsquerdo = i->X;
                limiteDireito = x1;

                intensidadeMaxF = i->intensidadeVminF;
                intensidadeMaxT = i->intensidadeVminT;
                intensidadeMinF = i->intensidadeVmaxF;
                intensidadeMinT = i->intensidadeVmaxT;

                especularMaxF = i->especularVminF;
                especularMaxT = i->especularVminT;
                especularMinF = i->especularVmaxF;
                especularMinT = i->especularVmaxT;

                normalDF = i->normalVminF;
                normalDT = i->normalVminT;
                normalEF = i->normalVmaxF;
                normalET = i->normalVmaxT;
            }

            //pinta o plano do nivel para fechar o prisma
            if(this->modelo==FLAT){
                //flat:
                //mesma intensidade para a face inteira
                glColor3f(R*i->intensidadeFace, G*i->intensidadeFace, B*i->intensidadeFace);
                glBegin(GL_POINTS);
                for(int k=limiteEsquerdo;k<limiteDireito;k++){
                    for(int j=0;j>=vr[0][2];j--){
                        glVertex3i(k, nivel+Hmin, j);
                    }
                }
                glEnd();
            }
            else if(this->modelo == GOURAUD){
                //gouraud: 2 casos
                glBegin(GL_POINTS);
                if(abs(i->Ymax-i->Ymin) < abs(i->Xmax-i->Xmin)){    //se variacao em X maior que variacao em Y
                    for(int k=limiteEsquerdo;k<limiteDireito;k++){
                        //interpola intensidades em X
                        intensidade1 = intensidadeMaxF - ((intensidadeMaxF-intensidadeMinF)*(Xmax-k)/(Xmax-Xmin));
                        intensidade2 = intensidadeMaxT - ((intensidadeMaxT-intensidadeMinT)*(Xmax-k)/(Xmax-Xmin));

                        especular1 = especularMaxF - ((especularMaxF-especularMinF)*(Xmax-k)/(Xmax-Xmin));
                        especular2 = especularMaxT - ((especularMaxT-especularMinT)*(Xmax-k)/(Xmax-Xmin));
                        for(int j=0;j>=vr[0][2];j--){
                            //interpola em Z com as intensidades achadas
                            intensidade3 = intensidade1 - ((intensidade1-intensidade2)*(0-j)/(0-vr[0][2]));
                            especular3 = especular1 - ((especular1-especular2)*(0-j)/(0-vr[0][2]));
                            glColor3f(R*intensidade3+especular3, G*intensidade3+especular3, B*intensidade3+especular3);
                            glVertex3i(k, nivel+Hmin, j);
                        }
                    }
                }
                else{   //se variacao em Y for maior
                    //interpola em Y primeiro
                    intensidade1 = intensidadeMaxF - ((intensidadeMaxF-intensidadeMinF)*(Ymax-(nivel+Hmin))/(Ymax-Ymin));
                    intensidade2 = intensidadeMaxT - ((intensidadeMaxT-intensidadeMinT)*(Ymax-(nivel+Hmin))/(Ymax-Ymin));

                    especular1 = especularMaxF - ((especularMaxF-especularMinF)*(Ymax-(nivel+Hmin))/(Ymax-Ymin));
                    especular2 = especularMaxT - ((especularMaxT-especularMinT)*(Ymax-(nivel+Hmin))/(Ymax-Ymin));
                    for(int k=limiteEsquerdo;k<limiteDireito;k++){
                        for(int j=0;j>=vr[0][2];j--){
                            //ineterpola em Z com as intensidades achadas
                            intensidade3 = intensidade1 - ((intensidade1-intensidade2)*(0-j)/(0-vr[0][2]));
                            especular3 = especular1 - ((especular1-especular2)*(0-j)/(0-vr[0][2]));
                            glColor3f(R*intensidade3+especular3, G*intensidade3+especular3, B*intensidade3+especular3);
                            glVertex3i(k, nivel+Hmin, j);
                        }
                    }
                }
                glEnd();
            }
            else if(this->modelo == PHONG){
                //phong:
                //igual ao gouraud, porem interpola as normais para depois calcular as intensidades
                glBegin(GL_POINTS);
                if(abs(i->Ymax-i->Ymin) < abs(i->Xmax-i->Xmin)){
                    for(int k=limiteEsquerdo;k<=limiteDireito;k++){
                        interp_phong(normal1, normalDF, normalEF, k, Xmax, Xmin);
                        interp_phong(normal2, normalDT, normalET, k, Xmax, Xmin);

                        for(int j=0;j>=vr[0][2];j--){
                            interp_phong(normal3, normal1, normal2, j, 0, vr[0][2]);
                            cria_ponto_ou_vetor(ponto, k, nivel+Hmin, j);
                            intensidade3 = intensidadeDifusa(ponto, normal3, luz, vetorLuz, this->intensidadeLuz);
                            especular3 = intensidadeEspecular(ponto, normal3, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);
                            glColor3f(R*intensidade3+especular3, G*intensidade3+especular3, B*intensidade3+especular3);
                            glVertex3i(k, nivel+Hmin, j);
                        }
                    }
                }
                else{

                    interp_phong(normal1, normalDF, normalEF, nivel+Hmin, Ymax, Ymin);
                    interp_phong(normal2, normalDT, normalET, nivel+Hmin, Ymax, Ymin);

                    for(int k=limiteEsquerdo;k<=limiteDireito;k++){
                        for(int j=0;j>=vr[0][2];j--){
                            interp_phong(normal3, normal1, normal2, j, 0, vr[0][2]);
                            cria_ponto_ou_vetor(ponto, k, nivel+Hmin, j);
                            intensidade3 = intensidadeDifusa(ponto, normal3, luz, vetorLuz, this->intensidadeLuz);
                            especular3 = intensidadeEspecular(ponto, normal3, obs, vetorObs, luz, vetorLuz, this->intensidadeLuz, reflexaoLuz, expoente);
                            glColor3f(R*intensidade3+especular3, G*intensidade3+especular3, B*intensidade3+especular3);
                            glVertex3i(k, nivel+Hmin, j);
                        }
                    }
                }
                glEnd();
            }
        }
        nivel++;    //PROX NIVEL
    }

}

void CanvasOpenGL::paintGL() {

    this->setParameters();

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glRotated(this->xRot, 1.0, 0.0, 0.0); // X
    glRotated(this->yRot, 0.0, 1.0, 0.0); // Y
    glRotated(this->zRot, 0.0, 0.0, 1.0); // Z

    // DESENHA POLIGONOS JA SALVOS
    for (int i = 0; i < (int)this->polygons.size(); i++) {
        Poligono *p = this->polygons[i];
        int n = (int)p->vertices.size();

        scanline(p->vertices, p->color[0], p->color[1], p->color[2]);
    }

    // DESENHA POLIGONO ATUAL
    glColor3d(this->R, this->G, this->B);
    vector<vector<GLfloat>> vertices = this->currentPolygon->vertices;
    int n = (int)vertices.size();
    if(!vertices.empty()){
        vertices.push_back(vertices[0]);    // Reinsere prmieiro vertice para fechar poligono
    }
    for (int i = 0; i < (int)vertices.size(); i++) {
        if(vertices[i][0] <= vertices[(i+1)%n][0]){
            drawLine(vertices[i], vertices[(i+1)%n], this->currentPolygon->color);
        } else {
            drawLine(vertices[(i+1)%n], vertices[i], this->currentPolygon->color);
        }
    }

    // DESENHA PONTOS CLICADOS
    for (int i=0;i<(int)vertices.size();i++) {
        vector<GLfloat> v = vertices[i];
        drawDot(v[0], v[1], v[2]);
        //cout<<this->currentPolygon->vertices[i][0]<<" "<<this->currentPolygon->vertices[i][1]<<endl;
    }

    glFlush();

}

void CanvasOpenGL::createPolygon(){
    if(this->currentPolygon->vertices.size() <= 0){
        return;
    }
    int n =  (int)this->currentPolygon->vertices.size();
    vector<vector<GLfloat>> vertices = this->currentPolygon->vertices;
    vector<vector<GLfloat>> backVertices;
    bool clockwise;

    // Sentido do Poligono
    vector<GLfloat> cornerVertice = {(float)INT_MIN, (float)INT_MAX, 0};
    int cornerIndex = -1;
    for (int i = 0; i < n; i++) {
        if( vertices[i][1] < cornerVertice[1] ||
            (vertices[i][1] == cornerVertice[1] && vertices[i][0] > cornerVertice[0])){
            cornerVertice[0] = vertices[i][0];
            cornerVertice[1] = vertices[i][1];
            cornerIndex = i;
        }
    }
    vector<GLfloat> v1(3);
    forma_vetor(v1, vertices[(cornerIndex+n-1)%n], vertices[cornerIndex]);
    vector<GLfloat> v2(3);
    forma_vetor(v2, vertices[cornerIndex], vertices[(cornerIndex+1)%n]);
    vector<GLfloat> normal = vetor_normal(v1, v2);
    clockwise = (normal[2] < 0);

    if(clockwise){
        for (int i = 0; i < n; i++) {
            vector<GLfloat> aux;
            aux.push_back(vertices[(i)][0]);
            aux.push_back(vertices[(i)][1]);
            aux.push_back((-1)*this->Z);
            backVertices.push_back(aux);
        }
    } else {
        for (int i = n-1; i >= 0; i--) {
            vector<GLfloat> aux;
            aux.push_back(vertices[(i)][0]);
            aux.push_back(vertices[(i)][1]);
            aux.push_back((-1)*this->Z);
            backVertices.push_back(aux);
        }
    }

    // Salva poligono do usuario com coordenada Z indicada
    Poligono *p = new Poligono;
    vector<GLfloat> colors(4, 0);
    colors[0] = this->R;
    colors[1] = this->G;
    colors[2] = this->B;
    p->vertices = backVertices;
    p->color = colors;
    this->polygons.push_back(p);
    this->currentPolygon = new Poligono;

    this->update();
}

void CanvasOpenGL::resizeGL(GLint w, GLint h) {
    this->width = w;
    this->height = h;
    this->aspect = this->width/this->height;

    this->update();
}

void CanvasOpenGL::setParameters () {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (this->isPerspective) {
        this->perspectiveGL();
    } else {
        glOrtho(
            this->hMin,
            this->hMax,
            this->vMin,
            this->vMax,
            this->novoNear,
            this->novoFar
        );
    }
}

void CanvasOpenGL::setR (double arg1){
    this->R = arg1;
    this->update();
}

void CanvasOpenGL::setG (double arg1){
    this->G = arg1;
    this->update();
}

void CanvasOpenGL::setB (double arg1){
    this->B = arg1;
    this->update();
}

void CanvasOpenGL::setZ (double arg1){
    this->Z = arg1;
}

void CanvasOpenGL::resetParameters () {
    this->hMin = -400;
    this->hMax = 400;
    this->vMin = -300;
    this->vMax = 300;
    this->novoNear = -500.0;
    this->novoFar = 500.0;
    this->setDesenhando(2);

    this->xRot = 0.0;
    this->yRot = 0.0;
    this->zRot = 0.0;
}

void CanvasOpenGL::reset() {
    this->resetParameters();

    this->update();
}

void CanvasOpenGL::setDesenhando(int d){
    this->desenhando = d;
    if(d == 0){
        this->currentPolygon->vertices.clear();
    }

    this->update();
}

void CanvasOpenGL::setvMax (GLfloat arg1) {
    this->vMax = arg1;

    this->update();
}

void CanvasOpenGL::setvMin (GLfloat arg1) {
    this->vMin = arg1;

    this->update();
}

void CanvasOpenGL::sethMax (GLfloat arg1) {
    this->hMax = arg1;

    this->update();
}

void CanvasOpenGL::sethMin (GLfloat arg1) {
    this->hMin = arg1;

    this->update();
}

void CanvasOpenGL::setFar (GLfloat arg1) {
    this->novoFar = arg1;
    this->update();
}

void CanvasOpenGL::setNear (GLfloat arg1) {
    this->novoNear = arg1;
    this->update();
}

void CanvasOpenGL::setFovY (GLfloat arg1) {
    this->fovY = arg1;
    this->update();
}

void CanvasOpenGL::setClockwise(bool v){
    this->clockwise = v;
}

void CanvasOpenGL::mouseMoveEvent(QMouseEvent *event) {

    if(this->desenhando==2)
        return;
    GLint dx = event->x() - this->lastPos.x();
    GLint dy = event->y() - this->lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        this->xRot = this->xRot + 2 * dy;
        this->yRot = this->yRot + 2 * dx;
    } else if (event->buttons() & Qt::RightButton) {
        this->xRot = this->xRot + 2 * dy;
        this->zRot = this->zRot + 2 * dx;
    }
    this->lastPos = event->pos();

    this->update();
}

void CanvasOpenGL::mousePressEvent(QMouseEvent *event)
{
    /*
    GLfloat vertice[3];
    vertice[0] = event->x();
    vertice[1] = event->y();
    vertice[2] = 0;
    */
    if(this->desenhando == 2){
        vector<GLint> vertice;
        vertice.push_back(event->x());
        vertice.push_back(event->y());
        vertice.push_back(0);
        this->currentPolygon->vertices.push_back(converte_coord(vertice));
        this->currentPolygon->color[0] = 255;
        this->update();
        //cout<<event->x()<<" "<<event->y()<<endl;
    }
}

void CanvasOpenGL::perspectiveGL() {
    GLfloat fW, fH;

    fH = tan(this->fovY / 360.0 * this->pi) * this->novoNear;
    fW = fH * this->aspect;

    glFrustum(-fW, fW, -fH, fH, this->novoNear, this->novoFar);
}

void  CanvasOpenGL::wheelEvent (QWheelEvent * event)
{
    (void) event;
}

void CanvasOpenGL::undo(){
    if(this->currentPolygon->vertices.size()>0){
        this->currentPolygon->vertices.pop_back();
        this->update();
    }
}

void CanvasOpenGL::resetRot(){
    this->xRot = 0.0;
    this->yRot = 0.0;
    this->zRot = 0.0;
    this->update();
}

void CanvasOpenGL::clear(){
    this->polygons.clear();
    this->currentPolygon = new Poligono;
    this->update();
}

void CanvasOpenGL::setXluz(GLfloat x){
    this->Xluz = x;
    this->update();
}

void CanvasOpenGL::setYluz(GLfloat y){
    this->Yluz = y;
    this->update();
}

void CanvasOpenGL::setZluz(GLfloat z){
    this->Zluz = z;
    this->update();
}

void CanvasOpenGL::set_modelo(int index){
    this->modelo = index;
    this->update();
}

void CanvasOpenGL::setXobs(GLfloat x){
    this->Xobs = x;
    this->update();
}

void CanvasOpenGL::setYobs(GLfloat y){
    this->Yobs = y;
    this->update();
}

void CanvasOpenGL::setZobs(GLfloat z){
    this->Zobs = z;
    this->update();
}

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);

    this->ui->canvasOpenGL->setFormat(format);

    this->updateStatusBar("Use esse método para dar feedback das interações na barra de status.");

    QTimer::singleShot( // o timer é apenas demonstrativo
        5000,
        this,
        SLOT( clearStatusBar() ) // E utilize esse para limpar a barra de status
    );

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateStatusBar(QString message) {
    this->ui->statusBar->showMessage(message);
}

void MainWindow::clearStatusBar() {
    this->ui->statusBar->clearMessage();
}


// OBSERVER
void MainWindow::on_obsX_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setXobs(arg1);
}

void MainWindow::on_obsY_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setYobs(arg1);
}

void MainWindow::on_obsZ_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setZobs(arg1);
}

// LIGHTING
void MainWindow::on_lightingXValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setXluz(arg1);
}

void MainWindow::on_lightingYValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setYluz(arg1);
}

void MainWindow::on_lightingZValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setZluz(arg1);
}

// VIEWING
void MainWindow::on_xMinValue_valueChanged(double arg1)
{
    (void) arg1;
}

void MainWindow::on_xMaxValue_valueChanged(double arg1)
{
    (void) arg1;
}

void MainWindow::on_yMinValue_valueChanged(double arg1)
{
    (void) arg1;
}


void MainWindow::on_yMaxValue_valueChanged(double arg1)
{
    (void) arg1;
}

void MainWindow::on_pertoValue_valueChanged(double arg1)
{
    (void) arg1;
}

void MainWindow::on_longeValue_valueChanged(double arg1)
{
    (void) arg1;
}

void MainWindow::on_fovyValue_valueChanged(double arg1)
{
    (void) arg1;
}

void MainWindow::on_isPerspective_stateChanged(int arg1)
{
    (void) arg1;
}

// DRAWING
void MainWindow::on_drawingRValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setR(arg1);
}

void MainWindow::on_drawingGValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setG(arg1);
}

void MainWindow::on_drawingBValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setB(arg1);
}

void MainWindow::on_drawingZValue_valueChanged(double arg1)
{
    this->ui->canvasOpenGL->setZ(arg1);
}

void MainWindow::on_drawingCheckBox_stateChanged(int arg1)
{
    this->ui->canvasOpenGL->setDesenhando(arg1);
    this->ui->canvasOpenGL->resetRot();
}

void MainWindow::on_undo_clicked()
{
    this->ui->canvasOpenGL->undo();
}

void MainWindow::on_confirm_clicked()
{
    this->ui->canvasOpenGL->createPolygon();
}

void MainWindow::on_toningValue_currentIndexChanged(int index)
{
    (void) index;
}

void MainWindow::on_clear_clicked()
{
    this->ui->canvasOpenGL->clear();
}

void MainWindow::on_reset_clicked()
{

}

void MainWindow::on_clockwiseCheckBox_stateChanged(int arg1)
{
    this->ui->canvasOpenGL->setClockwise(arg1);
}

void MainWindow::on_toningValue_activated(int index)
{
    this->ui->canvasOpenGL->set_modelo(index);
}

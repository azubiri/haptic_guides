#ifndef _GUIDE_H
#define _GUIDE_H

#include "chai3d.h"
#include <iostream>

using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------
#include <vector>

#include <opencv2/opencv.hpp>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include "openni_processor.h"

class CGuide
{

  private:
    char option;

    //------------------------------------------------------------------------------
    // GENERAL SETTINGS
    //------------------------------------------------------------------------------

    // stereo Mode
    /*
        C_STEREO_DISABLED:            Stereo is disabled 
        C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
        C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
        C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
    */
    cStereoMode stereoMode = C_STEREO_DISABLED;

    // fullscreen mode
    bool fullscreen = false;

    // mirrored display
    bool mirroredDisplay = false;


    //------------------------------------------------------------------------------
    // DECLARED VARIABLES
    //------------------------------------------------------------------------------

    // a world that contains all objects of the virtual environment
    cWorld* world;

    // a camera to render the world in the window display
    cCamera* camera;

    // a light source to illuminate the objects in the world
    cDirectionalLight *light;

    // a haptic device handler
    cHapticDeviceHandler* handler;

    // a pointer to the current haptic device
    cGenericHapticDevicePtr hapticDevice;

    // a virtual tool representing the haptic device in the scene
    cToolCursor* tool;

    // a guide object
    cMesh* guide;

    // a label to display the rate [Hz] at which the simulation is running
    cLabel* labelHapticRate;

    // indicates if the haptic simulation currently running
    bool simulationRunning = false;

    // indicates if the haptic simulation has terminated
    bool simulationFinished = true;

    // frequency counter to measure the simulation haptic rate
    cFrequencyCounter frequencyCounter;

    cHapticDeviceInfo hapticDeviceInfo;

    // last mouse position
    int mouseX;
    int mouseY;

     // information about computer screen and GLUT display window
    int screenW;
    int screenH;
    int windowW;
    int windowH;
    int windowPosX;
    int windowPosY;

    // opening of the guide
    float opening;

    // hight of the guide
    float hight;

    // angle of the guide
    double angle;

    // size of the guide
    float sizeguide;

    // vertexs
    int* vertexlow;
    int* vertextop;
    int numVertex;

    // inicialize the points witch represent the edge.
    vector<cVector3d> pos;

    // matrix rotation
    cMatrix3d rotMatrix;

    // inicialize the direction vectors and its orthogonals of each vertexlow
    vector<cVector3d> ort;
    vector<cVector3d> dir;

    // triangles which form each plane
    int indexTriangle;
    int numTriangles;
    int* triangle;

    double maxStiffness;
    double toolRadius;

    // indicates if + or - is pressed to increase or reduce the opening
    bool keypressed = false;

    cThread* guideThread;
    cThread* visionThread;

    // allows to create a default guide
    bool isOption = false;

    // true if uses the kinect, false if not
    bool isKinect = false;

    // allows to process a scene captured by kinect
    SimpleOpenNIProcessor v;

    static CGuide instance;

    double eps;

  public:
    CGuide()
    {
    }

    ~CGuide()
    {
    }

    void try_option();
    void setNumVertex(int numVertex);
    void select_angle(double angle);
    void window_display(int argc, char* argv[]);
    void createScene();
    void createTool();
    void createObject();
    void inicializeGuide();
    void setEdges(const vector<vector<float>> &edges);
    void set_hight(float hight);
    void set_opening(float opening);
    void set_size(float sizeguide);
    void defaultOption();
    void buildGuide();
    void hapticRate();
    void run();


    // callback when the window display is resized
    void resizeWindow(int w, int h);

    // callback when a key is pressed
    void keySelect(unsigned char key, int x, int y);

    // callback to render graphic scene
    void updateGraphics(void) const;

    ///////////////////////////////////////////////////////
    //----------------Provisional--------------------------
    ///////////////////////////////////////////////////////
    void vision();
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////

    // callback of GLUT timer
    void graphicsTimer(int data);

    // callback to handle mouse click
    void mouseClick(int button, int state, int x, int y);

    // callback to handle mouse motion when button is pressed
    void mouseMove(int x, int y);

    // function that closes the application
    void close(void);

    // main haptics simulation loop
    void updateHaptics();

    // update the opening of the guide
    void updateGuide();

    void createRotationMatrix();
    void rotatePoints(std::vector<cVector3d> &points);
    void irotatePoints(std::vector<cVector3d> &points);
    
    // find the vertices of top base
    vector<cVector3d> findVertex(float opening);

    vector<bool> isConcave(const vector<cVector3d> &pos, const vector<cVector3d> &dir);

    double angleins(cVector3d v3d);

    void setCollision();

    // create a new Plane of the guide
    void newPlane(int vertex0, int vertex1, int vertex2, int vertex3);

    // delete the guide build by triangles
    void deleteGuide();

    static CGuide &get_instance();

    static void updateGraphicsCallback();

    static void keySelectCallback(unsigned char key, int x, int y);

    static void mouseClickCallback(int button, int state, int x, int y);

    static void mouseMoveCallback(int x, int y);

    static void closeCallback();

    static void resizeWindowCallback(int w, int h);

    static void updateHapticsCallback();

    static void updateGuideCallback();

    ///////////////////////////////////////////////////////
    //------------------Provisional------------------------
    ///////////////////////////////////////////////////////
    static void visionCallback();
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////

    static void graphicsTimerCallback(int data);
};

#endif

#include "haptic_guides.h"

CGuide CGuide::instance;

void CGuide::try_option()
{

  cout << "Choose the geometric figure: " << endl;
  cout << "----------------------------" << endl;
  cout << "a: convex regular polygon with n vertices." << endl;
  cout << "b: convex irregular polygon with 4 vertices." << endl;
  cout << "c: convex irregular polygon with 6 vertices." << endl;
  cout << "d: concave polygon with 8 vertices." << endl;
  cout << "e: concave polygon with 9 vertices." << endl;
  cout << "f: concave polygon with 11 vertices." << endl;
  cout << "g: U-shaped figure." << endl;
  cout << "h: convex inside concave." << endl;
  cout << "i: rotated object." << endl;
  cout << "j: simple rotated object with 4 vertices" << endl;
  cout << "k: simple rotated object with 3 vertices" << endl;
  cout << "l: rotated concave object with 5 vertices" << endl;
  cout << "m: kinect" << endl;
  cout << endl << endl;

  do
  {
    cout << "Option: ";
    cin >> option;
    cout << endl << endl;
    if(option == 'a')
    {
      cout << "Number of vertices: ";
      cin >> numVertex;
      isOption = true;
      cout << endl << endl;
    }
    else if(option == 'b')
    {
      numVertex = 4;
      isOption = true;
    }
    else if(option == 'c')
    {
      numVertex = 6;
      isOption = true;
    }
    else if(option == 'd')
    {
      numVertex = 8;
      isOption = true;
    }
    else if(option == 'e')
    {
      numVertex = 9;
      isOption = true;
    }
    else if(option == 'f')
    {
      numVertex = 11;
      isOption = true;
    }
    else if(option == 'g')
    {
      numVertex = 8;
      isOption = true;
    }
    else if(option == 'h')
    {
      numVertex = 16;
      isOption = true;
    }
    else if(option == 'i')
    {
      numVertex = 4;
      isOption = true;
    }
    else if(option == 'j')
    {
      numVertex = 4;
      isOption = true;
    }
    else if(option == 'k')
    {
      numVertex = 3;
      isOption = true;
    }
    else if(option == 'l')
    {
      numVertex = 5;
      isOption = true;
    }
    else if(option == 'm')
    {
      ///////////////////////////////////////////////////////
      //-------------------Provisional-----------------------
      ///////////////////////////////////////////////////////
      float alpha;
      // Parameters of the segmentation: k_normal, min_cluster_size, max_cluster_size, neighbour_number, theta
      v.setSegParameters(50, 100, 1000, 10, 5.0);
      // Parameters of the passthrough filter: min, max
      v.setPassParameters(0.5, 1);
      // Parameters of the voxel-grid filter: leafSize
      v.setVoxParameters(0.01f);
      // Paramenters of the border computation: alpha
      cout << "Alpha: ";
      cin >> alpha;
      v.setAlpha(alpha);
      cout << "Epsilon = x*perimeter: ";
      cin >> eps;

      visionThread = new cThread();
      visionThread->start((void(*)(void))CGuide::visionCallback, CTHREAD_PRIORITY_GRAPHICS);
    
      std::vector<std::vector<float>> listPoints;
     
      int error = 0; 
      do
      {
        sleep(3);
        v.numberPointsEdge(numVertex);
        std::cout << "num: " << numVertex << std::endl << std::flush;
        error++;
      }while((numVertex > 500 ||  numVertex < 20) && error < 3);
      if(error == 3)
      {
        cout << "Object does not exist" << endl;
      }
      else
      {
        isKinect = true;
        isOption = true;
      }
      
      visionThread->stop(); 
      v.getEdges(listPoints);

      numVertex = listPoints.size();
      pos.resize(numVertex);
      for(int ii = 0; ii < numVertex; ii++)
      {
        pos[ii].set(listPoints[ii][0], listPoints[ii][1], listPoints[ii][2]);
      }
      ///////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////
    }
    else
    {
      cout << "Repeat" << endl;
      cout << endl << endl;
    }
  }while(!isOption);

}

void CGuide::select_angle(double angle)
{
  this->angle = angle;
}

void CGuide::window_display(int argc, char* argv[])
{
  // initialize GLUT
  glutInit(&argc, argv);

  // retrieve  resolution of computer display and position window accordingly
  screenW = glutGet(GLUT_SCREEN_WIDTH);
  screenH = glutGet(GLUT_SCREEN_HEIGHT);
  windowW = 0.8 * screenH;
  windowH = 0.5 * screenH;
  windowPosY = (screenH - windowH) / 2;
  windowPosX = windowPosY;

  // initialize the OpenGL GLUT window
  glutInitWindowPosition(windowPosX, windowPosY);
  glutInitWindowSize(windowW, windowH);

  if (stereoMode == C_STEREO_ACTIVE)
  {
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
  }
  else
  {
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  }

  // create display context and initialize GLEW library
  glutCreateWindow(argv[0]);

  #ifdef GLEW_VERSION
  // initialize GLEW
  glewInit();
  #endif

  glutDisplayFunc((void(*)(void))CGuide::updateGraphicsCallback);
  glutKeyboardFunc((void(*)(unsigned char, int, int))CGuide::keySelectCallback);
  glutMouseFunc((void(*)(int, int, int, int))CGuide::mouseClickCallback);
  glutMotionFunc((void(*)(int, int))CGuide::mouseMoveCallback);
  glutReshapeFunc((void(*)(int, int))CGuide::resizeWindowCallback);
  glutSetWindowTitle("GuidePoly");

  // set fullscreen mode
  if (fullscreen)
  {
    glutFullScreen();
  }
std::cout << "Window displayed" << std::endl << std::flush;
}

void CGuide::createScene()
{
  // create a new world.
  world = new cWorld();

  // set the background color of the environment
  world->m_backgroundColor.setBlack();

  // create a camera and insert it into the virtual world
  camera = new cCamera(world);
  world->addChild(camera);

  camera->setSphericalReferences(cVector3d(0,0,0),    // origin
                                 cVector3d(0,0,0),    // zenith direction
                                 cVector3d(0,0,0));   // azimuth direction

  camera->setSphericalDeg(6,    // spherical coordinate radius
                          0,     // spherical coordinate azimuth angle
                          0);    // spherical coordinate polar angle

  //camera->set(cVector3d(0, 0, 0),
  //            cVector3d(0, 0, 0),
  //            cVector3d(0, 0, 0));


  // set the near and far clipping planes of the camera
  // anything in front or behind these clipping planes will not be rendered
  camera->setClippingPlanes(0.01, 100.0);

  // set stereo mode
  camera->setStereoMode(stereoMode);

  // set stereo eye separation and focal length (applies only if stereo is enabled)
  camera->setStereoEyeSeparation(0.03);
  camera->setStereoFocalLength(3.0);

  // set vertical mirrored display mode
  camera->setMirrorVertical(mirroredDisplay);

  // enable multi-pass rendering to handle transparent objects
  camera->setUseMultipassTransparency(true);

  // create a light source
  light = new cDirectionalLight(world);

  // add light to world
  world->addChild(light);

  // enable light source
  light->setEnabled(true);


  // define the direction of the light beam
  light->setDir(-1.0, -1.0, -1.0);
  light->setLocalPos(1.0, 1.0, 1.0);


  // define the direction of the light beam
  light->setDir(-3.0,-0.5, 0.0);

  // set lighting conditions
  light->m_ambient.set(0.4f, 0.4f, 0.4f);
  light->m_diffuse.set(0.8f, 0.8f, 0.8f);
  light->m_specular.set(1.0f, 1.0f, 1.0f);
std::cout << "Scene" << std::endl << std::flush;
}

void CGuide::createTool()
{
  // create a haptic device handler
  handler = new cHapticDeviceHandler();

  // get access to the first available haptic device found
  handler->getDevice(hapticDevice, 0);

  // retrieve information about the current haptic device
  hapticDeviceInfo = hapticDevice->getSpecifications();

  // create a tool (cursor) and insert into the world
  tool = new cToolCursor(world);
  world->addChild(tool);

  // connect the haptic device to the virtual tool
  tool->setHapticDevice(hapticDevice);

  // define a radius for the virtual tool (sphere)
  toolRadius = 0.1;
  tool->setRadius(toolRadius);

  // map the physical workspace of the haptic device to a larger virtual workspace.
  tool->setWorkspaceRadius(1.0);

  // oriente tool with camera
  tool->setLocalRot(camera->getLocalRot());

  // haptic forces are enabled only if small forces are first sent to the device;
  // this mode avoids the force spike that occurs when the application starts when 
  // the tool is located inside an object for instance. 
  tool->setWaitForSmallForce(true);

  // start the haptic tool
  tool->start();
std::cout << "Tool created" << std::endl << std::flush;
}

void CGuide::createObject()
{
  // read the scale factor between the physical workspace of the haptic
  // device and the virtual workspace defined for the tool
  double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

  // get properties of haptic device
  double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
  maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
  double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;


std::cout << "Features of tool" << std::endl << std::flush;
}

void CGuide::setNumVertex(int numVertex)
{
  this->numVertex = numVertex;
}

void CGuide::inicializeGuide()
{
  // create a mesh
  guide = new cMesh();

  // add mesh to world
  world->addChild(guide);
  // assign the vertexs
  vertexlow = new int[numVertex];
  vertextop = new int[numVertex];
  for(unsigned int ii = 0; ii < numVertex; ii++)
  {
    vertexlow[ii] = guide->newVertex();
    vertextop[ii] = guide->newVertex();
  }
std::cout << "Guide Inicialized" << std::endl << std::flush;
}

void CGuide::set_hight(float hight)
{
  this->hight = hight;
  opening = hight / tan(angle*C_PI/180);
}

void CGuide::set_opening(float opening)
{
  this->opening = opening;
  hight = opening * tan(angle*C_PI/180);
}

void CGuide::set_size(float sizeguide)
{
  this->sizeguide = sizeguide;
}

void CGuide::defaultOption()
{
  if(isOption)
  {
    switch(option)
    {
      case 'a':
      {
        // regular polygon with 'numVertex' vertices
        for(int ii = 0; ii < numVertex; ii++)
        {
          double ang = (C_PI/180)*(360/numVertex)*ii;
          guide->m_vertices->setLocalPos(vertexlow[ii], sizeguide*cos(ang), sizeguide*sin(ang), sizeguide);
        }
        break;
      }
      case 'b':
      {
        // irregular polygon with 4 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 0*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 1*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 2*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], -1*sizeguide, 1*sizeguide, 1*sizeguide);
        break;
      }
      case 'c':
      {
        // irregular polygon with 6 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 0*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 1*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 1*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], 0*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[4], -1*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[5], -2*sizeguide, 2*sizeguide, 1*sizeguide);
        break;
      }
      case 'd':
      {
        // concave polygon with 8 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 0*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 1*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 1*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], 2*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[4], 2*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[5], -1*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[6], 0*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[7], -1*sizeguide, 1*sizeguide, 1*sizeguide);
        break;
      }
      case 'e':
      {
        // concave polygon with 9 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 3*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 2*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 0*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], -2*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[4], -2*sizeguide, -1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[5], -1*sizeguide, -1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[6], -1*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[7], 0*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[8], 2*sizeguide, 1*sizeguide, 1*sizeguide);
        break;
      }
      case 'f':
      {
        // concave polygon with 11 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 2*sizeguide, 4*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 1*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], -1*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], 2*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[4], 0*sizeguide, -1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[5], 2*sizeguide, -3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[6], 2*sizeguide, -2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[7], 5*sizeguide, -3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[8], 4*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[9], 6*sizeguide, 2*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[10], 4*sizeguide, 2*sizeguide, 1*sizeguide);
        break;
      }
      case 'g':
      {
        // concave polygon with 8 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 0*sizeguide-1+0.25, 0*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[1], 3*sizeguide-1+0.25, 0*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[2], 3*sizeguide-1+0.25, 3*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[3], 2*sizeguide-1+0.25, 3*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[4], 2*sizeguide-1+0.25, 1*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[5], 1*sizeguide-1+0.25, 1*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[6], 1*sizeguide-1+0.25, 3*sizeguide-0.5, 1*sizeguide-1);
        guide->m_vertices->setLocalPos(vertexlow[7], 0*sizeguide-1+0.25, 3*sizeguide-0.5, 1*sizeguide-1);
        break;
      }
      case 'h':
      {
        // concave polygon with 16 vertices
        guide->m_vertices->setLocalPos(vertexlow[0], 0*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 10*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 10*sizeguide, 10*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], 3*sizeguide, 10*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[4], 3*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[5], 6*sizeguide, 3*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[6], 6*sizeguide, 7*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[7], 5*sizeguide, 7*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[8], 5*sizeguide, 4*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[9], 4*sizeguide, 4*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[10], 4*sizeguide, 9*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[11], 8*sizeguide, 9*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[12], 8*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[13], 1*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[14], 1*sizeguide, 10*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[15], 0*sizeguide, 10*sizeguide, 1*sizeguide);
        break;
      }
      case 'i':
      {
        // rotate object
        guide->m_vertices->setLocalPos(vertexlow[0], 1*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], -1*sizeguide, 1*sizeguide, 0*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], -1*sizeguide, -1*sizeguide, -1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], 1*sizeguide, -1*sizeguide, 0*sizeguide);
        break;
      }
      case 'j':
      {
        // simple rotate object
        guide->m_vertices->setLocalPos(vertexlow[0], 0*sizeguide, 0*sizeguide, 0*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 1*sizeguide, 0*sizeguide, 0*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 1*sizeguide, 1*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], 0*sizeguide, 1*sizeguide, 1*sizeguide);
        break;
      }
      case 'k':
      {
        // simple rotate object
        guide->m_vertices->setLocalPos(vertexlow[0], 2*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 2*sizeguide, 2*sizeguide, -1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 1*sizeguide, 1*sizeguide, 1*sizeguide);
        break;
      }
      case 'l':
      {
        // simple rotate object
        guide->m_vertices->setLocalPos(vertexlow[0], 2*sizeguide, 0*sizeguide, 1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[1], 5*sizeguide, -5*sizeguide, 3*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[2], 2*sizeguide, 2*sizeguide, -1*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[3], -5*sizeguide, 5*sizeguide, 3*sizeguide);
        guide->m_vertices->setLocalPos(vertexlow[4], 1*sizeguide, 1*sizeguide, 1*sizeguide);
        break;
std::cout << "Que hago mal?" << std::endl << std::flush;
      }
      case 'm':
      {
        for(int ii = 0; ii < numVertex; ii++)
        {
          guide->m_vertices->setLocalPos(vertexlow[ii], pos[ii].x()*sizeguide, pos[ii].y()*sizeguide, pos[ii].z()-2);
        }
        cout << "done" << endl;
      }
    }
  }
  else
  {
    cout << "Error: choose a default guide (use try_option())" << endl << endl;
  }
}

void CGuide::setEdges(const vector<vector<float>> &edges)
{
  for(int ii = 0; ii < edges.size(); ii++)
  {
    guide->m_vertices->setLocalPos(vertexlow[ii], edges[ii][0]*sizeguide, edges[ii][1]*sizeguide, edges[ii][2]*sizeguide);
  }

}

void CGuide::createRotationMatrix()
{
  bool write = false;

  pos.resize(numVertex);
  // -------------------------------------------------
  // -----------------Low Base------------------------
  // -------------------------------------------------
  // get the position of each vertex of the low base
  pos[0] = guide->m_vertices->getLocalPos(vertexlow[0]);
  if(write)
  {
    cout << "Point 0: " << pos[0].str(5) << endl;
  }
  for(int ii = 1; ii < numVertex; ii++)
  {
    pos[ii] = guide->m_vertices->getLocalPos(vertexlow[ii]);

    if(write)
    {
      cout << "Point " << ii << ": " << pos[ii].str(5) << endl;
    }
  }
  
  // ------------------------------------------------
  // ---------------Find the plane-------------------
  // ------------------------------------------------
  // based on the first three points of the edge
  // find the equation of the plane Ax+By+Cz+D=0
  // A, B, C and D are found by implicit way of the plane:
  // |  x-x0  y-y0  z-z0  |
  // |  x1-x0 y1-y0 z1-z0 | = 0 we resolve the determinant of this matrix
  // |  x2-x0 y2-y0 z2-z0 |
  // where (x, y, z) is a point of the plane
  // and (x0, y0, z0), (x1, y1, z1) and (x2, y2, z2) are used to find the equation of the plane.
  
  double A, B, C, D;

  // point p = (x0, y0, z0)
  cVector3d p(pos[0]);
  // row u = (x1-x0 y1-y0 z1-z0) 
  cVector3d u(pos[1].x()-pos[0].x(), pos[1].y()-pos[0].y(), pos[1].z()-pos[0].z());
  // row v = (x2-x0 y2-y0 z2-z0) 
  cVector3d v(pos[2].x()-pos[0].x(), pos[2].y()-pos[0].y(), pos[2].z()-pos[0].z());

  A = u.y()*v.z() - u.z()*v.y();
  B = -u.x()*v.z() + u.z()*v.x();
  C = u.x()*v.y() - u.y()*v.x();
  D = -p.x()*A -p.y()*B -p.z()*C;

  // -------------------------------------------------
  // ---------------Normal----------------------------
  // -------------------------------------------------
  cVector3d normal(A, B, C);
  
  if(write)
  {
    cout << endl;
    cout << "Normal: " << normal.str(5) << endl;
  }

  // normalize the normal of the plane
  cVector3d nNormal;
  normal.normalizer(nNormal);

  if(write)
  {
    cout << "Normalized normal: " << nNormal.str(5) << endl << endl;
  }

  // --------------------------------------------------
  // -----------Compute the rotation matrix------------
  // --------------------------------------------------
  // we align the normal vector of the plane with the vector (0,0,1)
  cVector3d axis_z(0,0,1);
  
  if(nNormal.equals(axis_z))
  {
    rotMatrix.identity();
  }
  else
  {
    // -------------Rotation Matrix 2D-------------------
    // create rotation matrix in 2D
    double cos, sin;
    cVector3d crossAB, proj, rej, crossBA;

    cos = nNormal.dot(axis_z);
    
    nNormal.crossr(axis_z, crossAB);

    if(write)
    {
      cout << "Cross product between nNormal and axis_z: " << crossAB.str(5) << endl;
    }

    sin = crossAB.length();

    cMatrix3d rot2D(cos,  -sin,  0,
                    sin,   cos,  0,
                     0 ,    0 ,  1);

    if(write)
    {
      cout << "Rotation Matrix 2D: " << rot2D.str(5) << endl << endl;
    }

    // ------------Basis Change Matrix-----------------
    // create the basis change matrix
    proj.copyfrom(nNormal);

    cVector3d scnNormal(nNormal);
    scnNormal.mul(cos);
    axis_z.subr(scnNormal, rej);
    rej.normalize();

    // cross product between axis_z x nNormal
    axis_z.crossr(nNormal, crossBA);
    crossBA.normalize();

    // the inverted basis change matrix
    cMatrix3d iBasChange(proj, rej, crossBA);
    if(write)
    {
      cout << "Inverted Basis Change Matrix: " << iBasChange.str(5) << endl << endl;
    }

    // the basis change matrix
    cMatrix3d basChange;
    basChange = iBasChange;
    basChange.trans();
    if(write)
    {
      cout << "Basis Change Matrix: " << basChange.str(5) << endl << endl;
    }
   
    // --------------Matrix Rotation--------------------
    // compute the matrix rotation
    
    cMatrix3d tmp;

    rot2D.mulr(basChange, tmp);

    iBasChange.mulr(tmp, rotMatrix);

    if(write)
    {
      cout << "Rotation Matrix: " << rotMatrix.str(5) << endl;
    }
  }
  
}

void CGuide::rotatePoints(vector<cVector3d> &points)
{ 
  for(int jj = 0; jj < numVertex; jj++)
  {
    rotMatrix.mul(points[jj]);
  }
}

void CGuide::irotatePoints(vector<cVector3d> &points)
{ 
  cMatrix3d irotMatrix;
  rotMatrix.transr(irotMatrix);
  for(int jj = 0; jj < numVertex; jj++)
  {
    irotMatrix.mul(points[jj]);
  }
}


vector<cVector3d> CGuide::findVertex(float opening)
{
  // original position of each vertex of the low base represented by vectors
  //vector<cVector3d> pos(numVertex);
  // direction vector of each segment between two vertices
  //vector<cVector3d> dir(numVertex);
  // orthogonal vector of each direction vector
  //vector<cVector3d> ort(numVertex);
  // straight represented by a vector
  //vector<cVector3d> str(numVertex);
  // vertices of top base
  //vector<cVector3d> postop(numVertex);
  // OpenCV position
  vector<cv::Point2f> pos2d(numVertex);
  // Smoothed line
  vector<cv::Point2f> posmoothed;
  
  bool write = false;

  this->rotatePoints(pos);

  if(isKinect)
  {
    for(int kk = 0; kk < numVertex; kk++)
    {
      pos2d[kk].x = pos[kk].x();
      pos2d[kk].y = pos[kk].y();
    }

    // Simplify the borders
    // By Douglas-Peuker algorithm 
    if(true)
    { 
      cv::approxPolyDP(pos2d, posmoothed, eps*cv::arcLength(pos2d, true), true);
      //cv::approxPolyDP(pos2d, posmoothed, eps, true);
      numVertex = posmoothed.size();
      pos.resize(numVertex);

      for(int ll = 0; ll < numVertex; ll++)
      {
        pos[ll].x(posmoothed[ll].x);
        pos[ll].y(posmoothed[ll].y);
      }
    }
    // By Hough Transform
    else
    {
      vector<cv::Vec4i> lines;

      //cv::Mat image32f(pos2d, true);
      //cout << image32f.channels() << endl;
      //cv::namedWindow("lala");
      //cv::imshow("lala", image32f);
      //cv::waitKey(0);
      //cv::Mat image8u;
      //image32f.convertTo(image8u, CV_8UC1);


      cv::Mat image(100, 100, CV_8U);
      image.setTo(0);

      std::vector<cv::KeyPoint> kp_pos2d;
      for(vector<cv::Point2f>::const_iterator it = pos2d.begin(); it != pos2d.end(); it++)
      {
        cv::KeyPoint kp(*it, 50);
        kp_pos2d.push_back(kp);
      }

      cv::Mat pointmat;
      cv::drawKeypoints(image, kp_pos2d, pointmat, CV_RGB(255, 255, 255));

      cv::Mat grayimage;
      cv::cvtColor(grayimage, pointmat, CV_GRAY2RGB);

      cout << "Channels mat: " << grayimage.channels() << endl;

      //cv::imshow("Window", pointmat);
      //cv::waitKey(0);
      
      HoughLinesP( grayimage, lines, 1, CV_PI/180, 80, 30, 10);
      numVertex = lines.size();
      pos.resize(numVertex);

      //cout << endl << "Channels 8u:"<< image8u.channels() << endl;
      //HoughLinesP( image8u, lines, 1, CV_PI/180, 80, 30, 10);
      //numVertex = lines.size();
      //pos.resize(numVertex);
      
      for(size_t ll = 0; ll < numVertex; ll++)
      {
        pos[ll].x(lines[ll][0]);
        pos[ll].y(lines[ll][1]);
      } 
    }  
  }

  if(write)
  {
    /////////////
    cout << "1: Smoothed edge: "<< numVertex << endl;
    ////////////
    for(int ii = 0; ii<numVertex; ii++)
    {
      cout << pos[ii].str() << endl;
    }
    double dist1;
    dist1 = sqrt(pow(pos[0].x()-pos[1].x(),2)+pow(pos[0].y()-pos[1].y(),2)+pow(pos[0].z()-pos[1].z(),2));
    cout << "Euclidean distance 1: " << dist1 << endl;
  }

  // direction vector of each segment between two vertices
  vector<cVector3d> dir(numVertex);
  // orthogonal vector of each direction vector
  vector<cVector3d> ort(numVertex);
  // straight represented by a vector
  vector<cVector3d> str(numVertex);
  // vertices of top base
  vector<cVector3d> postop(numVertex);

  // --------Direction Vector & Orthogonal------------    
  // Direction vector calculated by vertex[i] and its adyacent
  // And next it calculates its normalized orthogonal
  for(int ii = 0; ii < numVertex-1; ii++)
  {
    dir[ii].set(pos[ii+1].x() - pos[ii].x(), pos[ii+1].y() - pos[ii].y(), 0);
  }
  // The last direction vector calculated by the first and the last vertices.
  dir[numVertex-1].set(pos[0].x() - pos[numVertex-1].x(), pos[0].y() - pos[numVertex-1].y(), 0);

  vector<bool> conc = this->isConcave(pos, dir);

  if(!(conc[1] && conc[2]) && !(conc[0] && conc[1]) && !(conc[numVertex-1] && conc[0]))
  {
    ort[0].set(pos[1].y() - pos[0].y(), pos[0].x() - pos[1].x(), 0);
    ort[0].normalize();
  }
  else
  {
    ort[0].set(0, 0, 0);
  }

  for(int ii = 1; ii < numVertex-2; ii++)
  {
    if(!(conc[ii+1] && conc[ii+2]) && !(conc[ii] && conc[ii+1]) && !(conc[ii-1] && conc[ii]))
    {
      ort[ii].set(pos[ii+1].y() - pos[ii].y(), pos[ii].x() - pos[ii+1].x(), 0);
      ort[ii].normalize();
    }
    else
    {
      ort[ii].set(0, 0, 0);
    }
  }

  if(!(conc[numVertex-2+1] && conc[0]) && !(conc[numVertex-2] && conc[numVertex-2+1]) && !(conc[numVertex-2-1] && conc[numVertex-2]))
  {
    ort[numVertex-2].set(pos[numVertex-2+1].y() - pos[numVertex-2].y(), pos[numVertex-2].x() - pos[numVertex-2+1].x(), 0);
    ort[numVertex-2].normalize();
  }
  else
  {
    ort[numVertex-2].set(0, 0, 0);
  }
  if(!(conc[0] && conc[1]) && !(conc[numVertex-1] && conc[0]) && !(conc[numVertex-1-1] && conc[numVertex-1]))
  {
    ort[numVertex-1].set(pos[0].y() - pos[numVertex-1].y(), pos[numVertex-1].x() - pos[0].x(), 0);
    ort[numVertex-1].normalize();
  }
  else
  {
    ort[numVertex-1].set(0, 0, 0);
  }

  // -----------------Straights-----------------------
  // straight str[i]--> Ax+By+C=0 which passes through the point
  // (x,y,z)=(pos[i].x()+opening*ort[i].x(), pos[i].y()+opening*ort[i].y(), pos[i].z()+hight)
  // with the direction vector dir[i].
  // each straight is represented as a vector.
  for(int ii = 0; ii < numVertex; ii++)
  {
    str[ii].set(dir[ii].y(), -dir[ii].x(), dir[ii].x()*(pos[ii].y() + opening*ort[ii].y()) - dir[ii].y()*(pos[ii].x() + opening*ort[ii].x()));
  }

  // -------------------Top Base----------------------
  // build the top base finding the intersection points by Cramer's rule 
  // str[i-1].x()x + srt[i-1].y()y + srt[i-1].z() = 0   (Ax + By + C = 0)
  // str[i].x()x + srt[i].y()y + srt[i].z() = 0    (Dx + Ey + F = 0)

  // Cramer's rule to find the (x, y) coordinates:
  // first vertex
  postop[0].x(( -str[numVertex-1].z()*str[0].y() + str[numVertex-1].y()*str[0].z()  ) / ( str[numVertex-1].x()*str[0].y() - str[numVertex-1].y()*str[0].x() ));
  postop[0].y(( -str[numVertex-1].x()*str[0].z() + str[numVertex-1].z()*str[0].x()  ) / ( str[numVertex-1].x()*str[0].y() - str[numVertex-1].y()*  str[0].x() ));
  postop[0].z(hight + pos[0].z());
  // rest of vertices
  for(int ii = 1; ii < numVertex; ii++)
  {
    postop[ii].x(( -str[ii-1].z()*str[ii].y() + str[ii-1].y()*str[ii].z()  ) / ( str[ii-1].x()*str[ii].y() - str[ii-1].y()*str[ii].x() ));
    postop[ii].y(( -str[ii-1].x()*str[ii].z() + str[ii-1].z()*str[ii].x()  ) / ( str[ii-1].x()*str[ii].y() - str[ii-1].y()*str[ii].x() ));
    postop[ii].z(hight + pos[ii].z());
  }

  if(write)
  {
    cout << "2: Pos top" << endl;
    for(int ii = 0; ii<numVertex; ii++)
    {
      cout << postop[ii].str() << endl;
    }
    double dist2;
    dist2 = sqrt(pow(postop[0].x()-postop[1].x(),2)+pow(postop[0].y()-postop[1].y(),2)+pow(postop[0].z()-postop[1].z(),2));
    cout << "Euclidean distance 2: " << dist2 << endl;
  }

  this->irotatePoints(pos);
  this->irotatePoints(postop);

  if(write)
  {
    cout << "3: Pos rotated" << endl;
    for(int ii = 0; ii<numVertex; ii++)
    {
      cout << pos[ii].str() << endl;
    }
    double dist3;
    dist3 = sqrt(pow(pos[0].x()-pos[1].x(),2)+pow(pos[0].y()-pos[1].y(),2)+pow(pos[0].z()-pos[1].z(),2));
    cout << "Euclidean distance 3: " << dist3 << endl;
    
    cout << "4: Pos top rotated" << endl;
    for(int ii = 0; ii<numVertex; ii++)
    {
      cout << postop[ii].str() << endl;
    }
    double dist4;
    dist4 = sqrt(pow(postop[0].x()-postop[1].x(),2)+pow(postop[0].y()-postop[1].y(),2)+pow(postop[0].z()-postop[1].z(),2));
    cout << "Euclidean distance 4: " << dist4 << endl; 
    /////////////
    cout << "Postop: "<< postop.size() << endl;
    ////////////
  }

  return postop;

}

void CGuide::newPlane(int vertex0, int vertex1, int vertex2, int vertex3)
{
  cMultiSegment segment;

  segment.newSegment(vertex0, vertex1);
  segment.newSegment(vertex0, vertex2);
  segment.newSegment(vertex2, vertex3);
  segment.newSegment(vertex1, vertex3);

  segment.setLineWidth(2);

  cColorf colseg(0.2, 0.2, 1);
  segment.setLineColor(colseg);


  // assign color value to each vertex
  cColorf colour(1.0, 0.2, 0.2);
  cColorf colour1(0.2, 1, 0.2);
  guide->m_vertices->setColor(vertex0, colour);
  guide->m_vertices->setColor(vertex1, colour);
  guide->m_vertices->setColor(vertex2, colour1);
  guide->m_vertices->setColor(vertex3, colour1);

  // create new triangle from vertices
  triangle[indexTriangle] = guide->newTriangle(vertex2, vertex3, vertex1);
  indexTriangle++;
  triangle[indexTriangle] = guide->newTriangle(vertex1, vertex0, vertex2);
  indexTriangle++;

  // enable vertex colours
  guide->setUseVertexColors(true);

  // compute surface normals
  guide->computeAllNormals();
}


void CGuide::buildGuide()
{

std::cout << "Started?" << std::endl << std::flush;
  // compute the rotation matrix
  this->createRotationMatrix();
std::cout << "rotated" << std::endl << std::flush;

  // find the vertices of the top base 
  vector<cVector3d> postop = this->findVertex(opening);
std::cout << "vertex found" << std::endl << std::flush;

  // set the position calculated of each vertex, except for points from kinect
  if(isKinect)
  {
    for(int ii = 0; ii < numVertex; ii++)
    {
      guide->m_vertices->setLocalPos(vertexlow[ii], pos[ii].x(), pos[ii].y(), pos[ii].z());
    }
  }
  for(int ii = 0; ii < numVertex; ii++)
  {
    guide->m_vertices->setLocalPos(vertextop[ii], postop[ii].x(), postop[ii].y(), postop[ii].z());
  }


  // inicialize the triangles
  indexTriangle = 0;
  numTriangles = 2*numVertex;

  // create the triangles which forming the faces of the guide
  triangle = new int[numTriangles];

  // draw the faces of the guide
  for(int ii = 0; ii < numVertex - 1; ii++)
  {
    this->newPlane(vertexlow[ii], vertexlow[ii+1], vertextop[ii], vertextop[ii+1]);
  }
  this->newPlane(vertexlow[numVertex-1], vertexlow[0], vertextop[numVertex-1], vertextop[0]);
std::cout << "Printed" << std::endl << std::flush;
}

void CGuide::setCollision()
{
  // compute a boundary box
  guide->computeBoundaryBox(true);

  // show/hide boundary box
  guide->setShowBoundaryBox(false);

  // compute collision detection algorithm
  guide->createAABBCollisionDetector(toolRadius);

  // define a default stiffness for the object
  guide->setStiffness(0.2 * maxStiffness, true);

  // define some haptic friction properties
  guide->setFriction(0.1, 0.2, true);

  // enable display list for faster graphic rendering
  guide->setUseDisplayList(true);

  //transparence in object
  guide->setTransparencyLevel(0.5);

  // center object in scene
  //guide->setLocalPos(-1.0 * guide->getBoundaryCenter());
}

void CGuide::hapticRate()
{
  // create a font
  cFont *font = NEW_CFONTCALIBRI20();

  // create a label to display the haptic rate of the simulation
  labelHapticRate = new cLabel(font);
  camera->m_frontLayer->addChild(labelHapticRate);
}


void CGuide::run()
{
  // create a thread which starts the main haptics rendering loop
  cThread* hapticsThread = new cThread();
  hapticsThread->start((void(*)(void))CGuide::updateHapticsCallback, CTHREAD_PRIORITY_HAPTICS);

  // create a thread which starts the main haptics rendering loop
  guideThread = new cThread();
  guideThread->start((void(*)(void))CGuide::updateGuideCallback, CTHREAD_PRIORITY_GRAPHICS);


  // setup callback when application exits
  atexit((void(*)(void))CGuide::closeCallback);

  // start the main graphics rendering loop
  glutTimerFunc(50, (void(*)(int))CGuide::graphicsTimerCallback, 0);
  glutMainLoop();
}

void CGuide::updateHaptics(void)
{
  // simulation in now running
  simulationRunning  = true;
  simulationFinished = false;

  // main haptic simulation loop
  while(simulationRunning)
  {
    // compute global reference frames for each object
    world->computeGlobalPositions(true);

    // update position and orientation of tool
    tool->updateFromDevice();

    // compute interaction forces
    tool->computeInteractionForces();

    // send forces to haptic device
    tool->applyToDevice();

    // update frequency counter
    frequencyCounter.signal(1);
  }

  // exit haptics thread
  simulationFinished = true;
}

void CGuide::updateGuide(void)
{
  // when '+' or '-' is pressed
  while(true)
  {
    if(keypressed)
    {
      this->deleteGuide();
      vector<cVector3d> postop;

      postop = this->findVertex(opening);

      // set the position calculated of each vertex
      for(int ii = 0; ii < numVertex; ii++)
      {
        guide->m_vertices->setLocalPos(vertextop[ii], postop[ii].x(), postop[ii].y(), postop[ii].z());
      }

      for(int ii = 0; ii < numVertex - 1; ii++)
      {
        this->newPlane(vertexlow[ii], vertexlow[ii+1], vertextop[ii], vertextop[ii+1]);
      }
      this->newPlane(vertexlow[numVertex-1], vertexlow[0], vertextop[numVertex-1], vertextop[0]);
      this->setCollision();
      keypressed = false;
    }
    glutPostRedisplay();
  }
}

////////////////////////////////////////////////////////////////
//---------------Provisional-----------------------------------
////////////////////////////////////////////////////////////////
void CGuide::vision()
{
  v.run();
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void CGuide::resizeWindow(int w, int h)
{
  windowW = w;
  windowH = h;
}

void CGuide::keySelect(unsigned char key, int x, int y)
{
  // option ESC: exit
  if ((key == 27) || (key == 'q'))
  {
    // exit application
    guideThread->stop();
    exit(0);
  }

  // option f: toggle fullscreen
  if (key == 'f')
  {
    if (fullscreen)
    {
      windowPosX = glutGet(GLUT_INIT_WINDOW_X);
      windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
      windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
      windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
      glutPositionWindow(windowPosX, windowPosY);
      glutReshapeWindow(windowW, windowH);
      fullscreen = false;
    }
    else
    {
      glutFullScreen();
      fullscreen = true;
    }
  }
  float step = 0.25;
  // option "+" or "-": increase or reduce the opening
  if (key == '+' || key == '-')
  {
    if(key == '+' && angle < 90)
    {
      angle = angle + step;
    }
    else if (key == '-' && angle > step)
    {
      angle = angle - step;
    }
    opening = hight / tan(angle*C_PI/180);
    keypressed = true;
  }

  if (key == 'a')
  {
    cout << "Set the angle: ";
    cin >> angle;
    cout << endl << endl;
    keypressed = true;
  }
  // option m: toggle vertical mirroring
  if (key == 'm')
  {
    mirroredDisplay = !mirroredDisplay;
    camera->setMirrorVertical(mirroredDisplay);
  }
}

void CGuide::mouseClick(int button, int state, int x, int y)
{
  mouseX = x;
  mouseY = y;
}

void CGuide::mouseMove(int x, int y)
{
  // compute mouse motion
  int dx = x - mouseX;
  int dy = y - mouseY;
  mouseX = x;
  mouseY = y;

  // compute new camera angles
  double azimuthDeg = camera->getSphericalAzimuthDeg() + (0.5 * dy);
  double polarDeg = camera->getSphericalPolarDeg() + (-0.5 * dx);

  // assign new angles
  camera->setSphericalAzimuthDeg(azimuthDeg);
  camera->setSphericalPolarDeg(polarDeg);

  // line up tool with camera
  tool->setLocalRot(camera->getLocalRot());
}

void CGuide::close(void)
{
  // stop the simulation
  simulationRunning = false;

  // wait for graphics and haptics loops to terminate
  while (!simulationFinished)
  {
    cSleepMs(100);
  }

  // close haptic device
  tool->stop();
}

void CGuide::graphicsTimer(int data)
{
  // inform the GLUT window to call updateGraphics again (next frame)
  if (simulationRunning)
  {
      glutPostRedisplay();
  }

  //glutTimerFunc(50, graphicsTimer, 0);
}

void CGuide::updateGraphics(void) const
{
  /////////////////////////////////////////////////////////////////////
  // UPDATE WIDGETS
  /////////////////////////////////////////////////////////////////////

  // update haptic rate data
  labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz       " + cStr(angle) + " degrees");

  // update position of label
  labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


  /////////////////////////////////////////////////////////////////////
  // RENDER SCENE
  /////////////////////////////////////////////////////////////////////

  // update shadow maps (if any)
  world->updateShadowMaps(false, mirroredDisplay);

  // render world
  camera->renderView(windowW, windowH);

  // swap buffers
  glutSwapBuffers();

  // wait until all GL commands are completed
  glFinish();

  // check for any OpenGL errors
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

vector<bool> CGuide::isConcave(const vector<cVector3d> &pos, const vector<cVector3d> &dir)
{
  bool isshown = false;
  string str;
  vector<cVector3d> copdir = dir;
  vector<cVector3d> ndir(numVertex);
  vector<double> alpha(numVertex);
  vector<double> beta(numVertex);
  vector<double> gamma(numVertex);
  vector<bool> conc(numVertex);

  if(isshown)
  {
    cout <<"------alpha-------" << endl;
  }

  for(int ii = 0; ii < numVertex; ii++)
  {
    copdir[ii].normalize();

    ndir[ii] = cNegate(copdir[ii]);
    alpha[ii] = this->angleins(copdir[ii]);

    if(isshown)
    {
      str = copdir[ii].str(5);
      cout << str << endl;
      cout << alpha[ii] << endl << endl;
    }
  }

  if(isshown)
  {
    cout << "-----beta-----" << endl;
    str = ndir[numVertex-1].str(5);
    cout << str << endl;
  }

  beta[0] = this->angleins(ndir[numVertex-1]);

  if(isshown)
  {
    cout << beta[0] << endl;
  }

  for(int ii = 0; ii < numVertex - 1; ii++)
  {
    beta[ii+1] = this->angleins(ndir[ii]);

    if(isshown)
    {
      str = ndir[ii].str(5);
      cout << str << endl;

      cout << beta[ii+1] << endl;
    }
  }

  if(isshown)
  {
    cout << "-----gamma-----" << endl;
  }

  for(int ii = 0; ii < numVertex; ii++)
  {
    if(alpha[ii] < beta[ii])
    {
      gamma[ii] = beta[ii] - alpha[ii];
    }
    else
    {
      gamma[ii] = 360 - (alpha[ii] - beta[ii]);
    }

    if(gamma[ii] <= 180)
    {
      conc[ii] = false;
    }
    else
    {
      conc[ii] = true;
    }

    if(isshown)
    {
      cout << gamma[ii] << "  condition: " << conc[ii]<< endl;
    }
  }

  return conc;
}

double CGuide::angleins(cVector3d v3d)
{

  double angins;

  if(v3d.x()>=-1 && v3d.x()<=1 && v3d.y()>=0 && v3d.y()<=1)
  {
    angins = (180/C_PI)*cAngle(v3d, cVector3d(1, 0, v3d.z()));
  }
  else if(v3d.x()>-1 && v3d.x()<=0 && v3d.y()>=-1 && v3d.y()<=0)
  {
    angins = (180/C_PI)*cAngle(v3d, cVector3d(1, 0, v3d.z())) + 2*(180 - (180/C_PI)*cAngle(v3d, cVector3d(1, 0, v3d.z())));
  }
  else if(v3d.x()>0 && v3d.x()<=1 && v3d.y()>=-1 && v3d.y()<=0)
  {
    angins = 360 - (180/C_PI)*cAngle(v3d, cVector3d(1, 0, v3d.z()));
  }

  return angins;
}

void CGuide::deleteGuide()
{
  for(int ii = 0; ii < numTriangles; ii++)
  {
    guide->removeTriangle(ii);
  }

  indexTriangle = 0;
}

CGuide &CGuide::get_instance()
{
  return instance;
}

void CGuide::updateGraphicsCallback()
{
  instance.updateGraphics();
}

void CGuide::keySelectCallback(unsigned char key, int x, int y)
{
  instance.keySelect(key, x, y);
}

void CGuide::mouseClickCallback(int button, int state, int x, int y)
{
  instance.mouseClick(button, state, x, y);
}

void CGuide::mouseMoveCallback(int x, int y)
{
  instance.mouseMove(x, y);
}

void CGuide::closeCallback()
{
  instance.close();
}

void CGuide::resizeWindowCallback(int w, int h)
{
  instance.resizeWindow(w, h);
}

void CGuide::updateHapticsCallback()
{
  instance.updateHaptics();
}

void CGuide::updateGuideCallback()
{
  instance.updateGuide();
}

/////////////////////////////////////////////////////
//------------------Provisional----------------------
/////////////////////////////////////////////////////
void CGuide::visionCallback()
{
  instance.vision();
}
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

void CGuide::graphicsTimerCallback(int data)
{
  instance.graphicsTimer(data);
}

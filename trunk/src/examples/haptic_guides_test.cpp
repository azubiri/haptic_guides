//#include "choose_filter.h"
//#include "segmentation.h"
#include "openni_processor.h"
#include "haptic_guides.h"
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{

//  SimpleOpenNIProcessor v;
//  v.run ();

  //--------------------------------------------------------------------------
  // INITIALIZATION
  //--------------------------------------------------------------------------

  cout << endl;
  cout << "-----------------------------------" << endl;
  cout << "CHAI3D" << endl;
  cout << "GUIDE" << endl;
  cout << "Copyright 2003-2016" << endl;
  cout << "-----------------------------------" << endl << endl << endl;
  cout << "Keyboard Options:" << endl << endl;
  cout << "[f] - Enable/Disable full screen mode" << endl;
  cout << "[m] - Enable/Disable vertical mirroring" << endl;
  cout << "[x] - Exit application" << endl;
  cout << endl << endl;

  CGuide& guide = CGuide::get_instance();
  double angle;

  guide.try_option();

  cout << "Opening in degrees: ";
  cin >> angle;
  cout << endl << endl;

  guide.select_angle(angle);

  //--------------------------------------------------------------------------
  // OPEN GL - WINDOW DISPLAY
  //--------------------------------------------------------------------------

  guide.window_display(argc, argv);    

  //--------------------------------------------------------------------------
  // WORLD - CAMERA - LIGHTING
  //--------------------------------------------------------------------------

  guide.createScene();

  //--------------------------------------------------------------------------
  // HAPTIC DEVICES / TOOLS
  //--------------------------------------------------------------------------

  guide.createTool();

  //--------------------------------------------------------------------------
  // CREATING OBJECTS
  //--------------------------------------------------------------------------

  guide.createObject();

  /////////////////////////////////////////////////////////////////////////
  // OBJECT: "GUIDE"
  /////////////////////////////////////////////////////////////////////////

  guide.inicializeGuide();

  // assign the opening and the hight
  guide.set_hight(2.5);

  // assign the size of the object
  guide.set_size(4.50);

  // set the local position where the base of guide(information given by PCL)
  guide.defaultOption();   
std::cout << "Sales del default?" << std::endl << std::flush;
  // find the vertices of the top base and draw the faces of the guide
  guide.buildGuide();

  // set the collision between the guide and the tool
  guide.setCollision();

  //--------------------------------------------------------------------------
  // WIDGETS
  //--------------------------------------------------------------------------

  //display the haptic rate
  guide.hapticRate();

  //--------------------------------------------------------------------------
  // START SIMULATION
  //--------------------------------------------------------------------------

  guide.run();        


  // exit
  return (0);





}

//------------------------------------------------------------------------------


//------------------------------------------------------------------------------





//------------------------------------------------------------------------------

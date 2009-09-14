#include <iostream>
#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <FGState.h>

using namespace std;

#if !defined(__GNUC__) && !defined(sgi) && !defined(_MSC_VER)
#  include <time>
#else
#  include <time.h>
#endif

#if defined(__BORLANDC__) || defined(_MSC_VER) || defined(__MINGW32__)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#  include <mmsystem.h>
#  include <regstr.h>
#  include <sys/types.h>
#  include <sys/timeb.h>
#else
#  include <sys/time.h>
#endif

// Function Definitions
void publish_config(JSBSim::FGFDMExec *FDMExec);

bool pfms_init(JSBSim::FGFDMExec *FDMExec);

void pfms_event(JSBSim::FGFDMExec *FDMExec);
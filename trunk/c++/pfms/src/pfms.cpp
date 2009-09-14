//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// INCLUDES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include "pfms.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// DEFINITIONS
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// GLOBAL DATA
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string RootDir = "";
string ScriptName;
string ResetName;
string LogOutputName;
string LogDirectiveName;
JSBSim::FGFDMExec* FDMExec;
bool realtime;
bool suspend;
bool catalog;

double end_time = -1.0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FORWARD DECLARATIONS
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#if defined(__BORLANDC__) || defined(_MSC_VER) || defined(__MINGW32__)
  double getcurrentseconds(void)
  {
    struct timeb tm_ptr;
    ftime(&tm_ptr);
    return tm_ptr.time + tm_ptr.millitm*0.001;
  }
#else
  double getcurrentseconds(void)
  {
    struct timeval tval;
    struct timezone tz;

    gettimeofday(&tval, &tz);
    return (tval.tv_sec + tval.tv_usec*1e-6);
  }
#endif

#if defined(__BORLANDC__) || defined(_MSC_VER) || defined(__MINGW32__)
  void sim_nsleep(long nanosec)
  {
    Sleep(nanosec*1e-6); // convert nanoseconds (passed in) to milliseconds for Win32.
  }
#else
  void sim_nsleep(long nanosec)
  {
    struct timespec ts, ts1;

    ts.tv_sec = 0;
    ts.tv_nsec = nanosec;
    nanosleep(&ts, &ts1);
  }
#endif

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// TO DO
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Fix FGState dt problem
// Initilise atmosphere/weather

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// IMPLEMENTATION
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

using namespace JSBSim;

int main(int argc, char* argv[])
{
  // *** INITIALIZATIONS *** //

  ScriptName = "scripts/c3104";  
  ResetName = "";
  LogOutputName = "output_file.csv";
  LogDirectiveName = "";
  bool result = false, success;
  bool was_paused = false;
  
  double frame_duration;

  double new_five_second_value = 0.0;
  double actual_elapsed_time = 0;
  double initial_seconds = 0;
  double current_seconds = 0.0;
  double paused_seconds = 0.0;
  double sim_time = 0.0;
  double sim_lag_time = 0;
  double cycle_duration = 0.0; // 
  long sleep_nseconds = 0;

  realtime = false;  
  suspend = false;
  catalog=false;

  // SET UP JSBSIM
  FDMExec = new JSBSim::FGFDMExec();
  FDMExec->SetAircraftPath(RootDir + "aircraft");
  FDMExec->SetEnginePath(RootDir + "engine");
  FDMExec->SetSystemsPath(RootDir + "systems");
  //FDMExec->GetPropertyManager()->Tie("simulation/frame_start_time", &actual_elapsed_time);
  //FDMExec->GetPropertyManager()->Tie("simulation/cycle_duration", &cycle_duration);  
  
  // LOAD A SCRIPT, WHICH LOADS EVERYTHING ELSE
  /*ScriptName = RootDir + ScriptName;
  result = FDMExec->LoadScript(ScriptName); 

  if (!result) {
      cerr << "Script file " << ScriptName << " was not successfully loaded" << endl;
      delete FDMExec;
      exit(-1);
  }*/

  // LOAD PFMS INITIAL CONDITIONS AND SETTINGS  
  result = pfms_init(FDMExec);
  if (!result) {
      cerr << "The PFMS was not successfully loaded" << endl;
      delete FDMExec;
      exit(-1);
  }

  // Load output directives file, if given
  if (!LogDirectiveName.empty()) {
    if (!FDMExec->SetOutputDirectives(LogDirectiveName)) {
      cout << "Output directives not properly set" << endl;
      delete FDMExec;
      exit(-1);
    }
  }

  // OVERRIDE OUTPUT FILE NAME. THIS IS USEFUL FOR CASES WHERE MULTIPLE
  // RUNS ARE BEING MADE (SUCH AS IN A MONTE CARLO STUDY) AND THE OUTPUT FILE
  // NAME MUST BE SET EACH TIME TO AVOID THE PREVIOUS RUN DATA FROM BEING OVER-
  // WRITTEN. THIS OVERRIDES ONLY THE FILENAME FOR THE FIRST FILE.
  if (!LogOutputName.empty()) {
    string old_filename = FDMExec->GetOutputFileName();
    if (!FDMExec->SetOutputFileName(LogOutputName)) {
      cout << "Output filename could not be set" << endl;
    } else {
      cout << "Output filename change from " << old_filename << " from aircraft"
              " configuration file to " << LogOutputName << " specified on"
              " command line" << endl;
    }
  }  

  cout << endl << JSBSim::FGFDMExec::fggreen << JSBSim::FGFDMExec::highint
       << "---- JSBSim Execution beginning ... --------------------------------------------"
       << JSBSim::FGFDMExec::reset << endl << endl;  

  result = FDMExec->Run();  // MAKE AN INITIAL RUN  

  if (suspend) FDMExec->Hold();

  JSBSim::FGJSBBase::Message* msg;  

  // Print actual time at start
  char s[100];
  time_t tod;
  time(&tod);
  strftime(s, 99, "%A %B %d %Y %X", localtime(&tod));
  cout << "Start: " << s << " (HH:MM:SS)" << endl;

  frame_duration = FDMExec->GetDeltaT();
  if (realtime) sleep_nseconds = (long)(frame_duration*1e9);
  else          sleep_nseconds = (10000000);           // 0.01 seconds

  tzset(); 
  current_seconds = initial_seconds = getcurrentseconds();  

  int cerr_count = 0;

  // CYCLIC EXECUTION LOOP, AND MESSAGE READING
  while (result) {
    while (FDMExec->SomeMessages()) {
      msg = FDMExec->ProcessMessage();
      switch (msg->type) {
      case JSBSim::FGJSBBase::Message::eText:
        cout << msg->messageId << ": " << msg->text << endl;
        break;
      case JSBSim::FGJSBBase::Message::eBool:
        cout << msg->messageId << ": " << msg->text << " " << msg->bVal << endl;
        break;
      case JSBSim::FGJSBBase::Message::eInteger:
        cout << msg->messageId << ": " << msg->text << " " << msg->iVal << endl;
        break;
      case JSBSim::FGJSBBase::Message::eDouble:
        cout << msg->messageId << ": " << msg->text << " " << msg->dVal << endl;
        break;
      default:
        cerr << "Unrecognized message type." << endl;
        break;
      }
    }

    // if running realtime, throttle the execution, else just run flat-out fast
    // unless "playing nice", in which case sleep for a while (0.01 seconds) each frame.
    // If suspended, then don't increment cumulative realtime "stopwatch".

    if ( ! FDMExec->Holding()) {
      if ( ! realtime ) {         // ------------ RUNNING IN BATCH MODE

        result = FDMExec->Run();
		
		/*if ( cerr_count == 100 )
		{
		cerr << "h-agl-ft :: " << FDMExec->GetPropertyValue("position/h-agl-ft") << " ap/altitude_setpoint :: " <<  FDMExec->GetPropertyValue("ap/altitude_setpoint") <<  " ap/altitude_hold :: " <<  FDMExec->GetPropertyValue("ap/altitude_hold") << endl;
		cerr_count = 0;
		}
		else
		{
			cerr_count = cerr_count+1;
		}*/
 
		// PFMS Event function
		pfms_event(FDMExec);		

      } else {                    // ------------ RUNNING IN REALTIME MODE

        // "was_paused" will be true if entering this "run" loop from a paused state.
        if (was_paused) {
          initial_seconds += paused_seconds;
          was_paused = false;
        }
        current_seconds = getcurrentseconds();                      // Seconds since 1 Jan 1970
        actual_elapsed_time = current_seconds - initial_seconds;    // Real world elapsed seconds since start
        sim_lag_time = actual_elapsed_time - FDMExec->GetSimTime(); // How far behind sim-time is from actual
                                                                    // elapsed time.
        for (int i=0; i<(int)(sim_lag_time/frame_duration); i++) {  // catch up sim time to actual elapsed time.
          result = FDMExec->Run();
          cycle_duration = getcurrentseconds() - current_seconds;   // Calculate cycle duration
          current_seconds = getcurrentseconds();                    // Get new current_seconds
          if (FDMExec->Holding()) break;
        }        

        if (FDMExec->GetSimTime() >= new_five_second_value) { // Print out elapsed time every five seconds.
          cout << "Simulation elapsed time: " << FDMExec->GetSimTime() << endl;
          new_five_second_value += 5.0;
        }
      }
    } else { // Suspended
      was_paused = true;
      paused_seconds = getcurrentseconds() - current_seconds;
      sim_nsleep(sleep_nseconds);
      result = FDMExec->Run();
    }

  }

  // PRINT ENDING CLOCK TIME
  time(&tod);
  strftime(s, 99, "%A %B %d %Y %X", localtime(&tod));
  cout << "End: " << s << " (HH:MM:SS)" << endl;

  // CLEAN UP
  delete FDMExec;  

  return 0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool 
pfms_init(JSBSim::FGFDMExec *FDMExec)//, JSBSim::FGInitialCondition *IC)
{
	// Load an aircraft
    // NOTE:: Model will eventually only require loading a single time
	string AircraftName = "c310";
	if ( ! FDMExec->LoadModel( RootDir + "aircraft", RootDir + "engine", RootDir + "systems", AircraftName)) 
	{
      cerr << "JSBSim Flight model could not be loaded." << endl << endl;
	  return false;
    }

	// Define simulation timing 	
	//JSBSim::FGState *State = FDMExec->GetState(); // Creates a pointer to the State object of FDMExec	   
    //State->Setdt(0.02); 
	// Included FGState.h however dt did not assign for FDMExec

	// Assign inital conditions
	JSBSim::FGInitialCondition *IC = FDMExec->GetIC(); // Creates a pointer to the IC object of FDMExec	

	IC->SetLatitudeDegIC(29.593978); // Initial Latitude
	IC->SetLongitudeDegIC(-95.163839); // Initial Longitude
	IC->SetTerrainElevationFtIC(32.0); // Inital terrain elevation
	IC->SetAltitudeAGLFtIC(1000); // Initial altitude above ground level

	IC->SetUBodyFpsIC(175.8402231); // Inital x-axis body velocity
	IC->SetVBodyFpsIC(1.154094211); // Inital y-axis body velocity
	IC->SetWBodyFpsIC(-8.954631464); // Initial z-axis body velocity
	
	IC->SetPhiRadIC(0); // Initial roll angle
	IC->SetThetaRadIC(0); // Initial pitch angle
	IC->SetPsiRadIC(0); // Initial heading angle

	/* // Assign wind conditions to model
	IC->SetWindDirDegIC(0);
	IC->SetWindMagKtsIC(0);
	// OR
	IC->SetWindNEDFpsIC(0);
	*/

	// Engage engines if required to initially be in running state	
	// Works but not required
	/*JSBSim::FGPropulsion *propulsion = FDMExec->GetPropulsion();
	for(int i=0; i<propulsion->GetNumEngines(); i++) {
		propulsion->GetEngine(i)->SetRunning(true);		
    }*/
	
	/*while (running_elements) {
		n = int(running_elements->GetDataAsNumber());
		propulsion->InitRunning(n);
		running_elements = document->FindNextElement("running");
	}*/

	// Run IC and reset sim to 0.0
	FDMExec->RunIC();

	// Assign flight properties
	FDMExec->SetPropertyValue("fcs/mixture-cmd-norm[0]", 1);
	FDMExec->SetPropertyValue("fcs/mixture-cmd-norm[1]", 1);
	FDMExec->SetPropertyValue("fcs/advance-cmd-norm[0]", 1);
	FDMExec->SetPropertyValue("fcs/advance-cmd-norm[1]", 1);
	FDMExec->SetPropertyValue("propulsion/magneto_cmd", 3);
	FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[0]", 1);
	FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[1]", 1);
	FDMExec->SetPropertyValue("propulsion/starter_cmd", 1);
	FDMExec->SetPropertyValue("ap/altitude_setpoint", 1000);
	FDMExec->SetPropertyValue("ap/attitude_hold", 0);
	FDMExec->SetPropertyValue("guidance/target_wp_latitude_rad", 0.507481);
	FDMExec->SetPropertyValue("guidance/target_wp_longitude_rad", -1.660062);
	FDMExec->SetPropertyValue("ap/heading_setpoint", 0);
	// Assigned ap/heading-setpoint-select to '1'
	FDMExec->SetPropertyValue("ap/heading-setpoint-select", 1);
	FDMExec->SetPropertyValue("ap/heading_hold", 1);
	FDMExec->SetPropertyValue("ap/active-waypoint", 1);	

	// Raise landing gear
	FDMExec->SetPropertyValue("gear/gear-cmd-norm", 0);

	// Assign altitude hold
	FDMExec->SetPropertyValue("ap/altitude_hold", 1);	

	/*
	// Trim the Aircraft
	JSBSim::FGTrim fgt(FDMExec, JSBSim::tGround);
    if( !fgt.DoTrim() ) 
	{      
	  fgt.Report();
	  fgt.TrimStats();
	  exit(-1);
    }	
	*/
    
	cerr << "~~~~~~~~~~~~~~ End of IC function ~~~~~~~~~~~~~~" << endl;

	return true;
}

void 
pfms_event(JSBSim::FGFDMExec *FDMExec)
{	
	// Head to first Waypoint
	if ( (FDMExec->GetPropertyValue("guidance/wp-distance") < 200) && (FDMExec->GetPropertyValue("ap/active-waypoint") == 1) ) 
	{
		FDMExec->SetPropertyValue("guidance/target_wp_latitude_rad", 0.511661);
		FDMExec->SetPropertyValue("guidance/target_wp_longitude_rad", -1.653510);
		FDMExec->SetPropertyValue("ap/active-waypoint", 2);		
		cout << "First waypoint conditions met." << endl;
		return;
	}

	// Head to second Waypoint
	if ( (FDMExec->GetPropertyValue("guidance/wp-distance") < 200) && (FDMExec->GetPropertyValue("ap/active-waypoint") == 2) ) 
	{
		FDMExec->SetPropertyValue("guidance/target_wp_latitude_rad", 0.516512);
		FDMExec->SetPropertyValue("guidance/target_wp_longitude_rad", -1.660922);
		FDMExec->SetPropertyValue("ap/active-waypoint", 3);		
		cout << "Second waypoint conditions met." << endl;
		return;
	}	

	// Terminate
	if ( FDMExec->GetPropertyValue("guidance/wp-distance") < 200 && FDMExec->GetPropertyValue("ap/active-waypoint") == 3 ) 
	{
		FDMExec->SetPropertyValue("simulation/terminate", 1);
		//cout << "Terminate conditions met." << endl;
	}
}

void
publish_config(JSBSim::FGFDMExec *FDMExec)
{
	//%% WORKING %%
	/*
    double value = 0;
	double value1 = 10000;
	double value2 = 30;

  	FDMExec->SetPropertyValue("ic/alpha-rad", value);
	FDMExec->SetPropertyValue("ic/beta-rad", value);
	FDMExec->SetPropertyValue("ic/gamma-rad", value);
	
	FDMExec->SetPropertyValue("ic/lat-gc-deg", 29.59397397704);	// use rad // work sort of, dont know what this is? ground control?
	FDMExec->SetPropertyValue("ic/long-gc-deg", -95.16382606877);
	FDMExec->SetPropertyValue("ic/h-agl-ft", value1); // working

	FDMExec->SetPropertyValue("ic/p-rad_sec", value);
	FDMExec->SetPropertyValue("ic/q-rad_sec", value);
	FDMExec->SetPropertyValue("ic/r-rad_sec", value);

	FDMExec->SetPropertyValue("ic/u-fps", value2);
	FDMExec->SetPropertyValue("ic/v-fps", value;
	FDMExec->SetPropertyValue("ic/w-fps", value);

	
	*/
	//FDMExec->SetPropertyValue("ic/h-agl-ft", 10000); // working

	//cout << "Vel:: " << FDMExec->GetPropertyValue("velocities/vc-fps") << endl;

	//FDMExec->SetPropertyValue("ic/alpha-deg", 50);
	//FDMExec->SetPropertyValue("ic/beta-deg", 50);
	//FDMExec->SetPropertyValue("ic/gamma-deg", 50);

	//FDMExec->SetPropertyValue("ic/p-rad_sec", 50);
	//FDMExec->SetPropertyValue("ic/q-rad_sec", 50);
	//FDMExec->SetPropertyValue("ic/r-rad_sec", 50);

	//FDMExec->SetPropertyValue("ic/u-fps", 50);
	//FDMExec->SetPropertyValue("ic/v-fps", 50);
	//FDMExec->SetPropertyValue("ic/w-fps", 50);

	//FDMExec->SetPropertyValue("position/lat-gc-deg", 21);
	//FDMExec->SetPropertyValue("velocities/u-aero-fps", 25);
	//FDMExec->SetPropertyValue("velocities/u-fps", 25);

	//FDMExec->SetPropertyValue("simulation/cycle_duration", 2); 
  // %%  END WORKING %%

	//cout << "Simulation Time :: " << FDMExec->GetSimTime() << endl;
	//cout << "Current Time :: " << getcurrentseconds() << endl;

	// ResetToInitialConditions function may be useful
	// Difference between tie and setpropertyvalue
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Includes
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include "pfms.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Global Data
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
string RootDir = "";
string LogOutputName;
JSBSim::FGFDMExec* FDMExec;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Forward Declarations
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
// Work to do
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Fix FGState dt problem
// Initilise atmosphere/weather
// Create xml options file
// IC read from ap
// Create Trim
// Confirm requirements of propulsion
// Event requires dynamic changing of waypoints - needs initiate wps function
// Work on publish config function, and determine if its required
// Make loop calculate at a certain frequency

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Implementation
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

using namespace JSBSim;

int main(int argc, char* argv[])
{
	// Initialisations  	
	double initial_seconds = 0;
	double current_seconds = 0.0;	
	bool result = false;
	// Should be in xml options file
	string AircraftName = "c310"; 	
	LogOutputName = "../pfms/logfiles/output_file.csv";
	double frame_duration = 0.02;
	double prediction_horizon = 10.0;
	double start_time = 0;
	// RootDir = "";

	// Set up JSBSim
	FDMExec = new FGFDMExec();	
	FDMExec->SetAircraftPath(RootDir + "aircraft");
	FDMExec->SetEnginePath(RootDir + "engine");
	FDMExec->SetSystemsPath(RootDir + "systems"); 

	// Assign debug level
	FDMExec->SetDebugLevel(0);

	// Load Aircraft	
	if ( ! FDMExec->LoadModel( RootDir + "aircraft", RootDir + "engine", RootDir + "systems", AircraftName)) 
	{
		cerr << "JSBSim Flight model could not be loaded." << endl << endl;
		return false;
	}

	// Below will iterate
	// while(active) {

	// Load output file
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

	// Load PFMS initial settings - NOTE:: dont know if bool is required
	result = pfms_init(FDMExec);
	if (!result) {
		cerr << "PFMS initialisation was not successfully loaded." << endl;
		delete FDMExec;
		exit(-1);
	}   

	cout << endl << FGFDMExec::fggreen << FGFDMExec::highint
	<< "---------- JSBSim Execution beginning ----------"
	<< FGFDMExec::reset << endl << endl;  
    
	// Make an initial run , assign predict true if model is running properly NOTE:: could be used for error checking
	result = FDMExec->Run();    

	// Print actual time at start
	char s[100];
	time_t tod;
	time(&tod);
	strftime(s, 99, "%A %B %d %Y %X", localtime(&tod));
	cout << "Start: " << s << " (HH:MM:SS)" << endl;	

	tzset(); 
	current_seconds = initial_seconds = getcurrentseconds(); 

	// Cyclic execution loop, and message reading to prediction horizon
	while (result) {

		// Read output messages		
		pfms_readJSBSimMessages(FDMExec);
		
		// Run model
		result =  FDMExec->Run();	

		// Determine if current state meets pfms event states
		pfms_event(FDMExec);

		if (( FDMExec->GetSimTime() - start_time) >= (prediction_horizon - frame_duration))
			result = false;

		cerr << "Sim Time:: " << FDMExec->GetSimTime() << endl;
	}

	// Print ending clock time
	time(&tod);
	strftime(s, 99, "%A %B %d %Y %X", localtime(&tod));
	cout << "End: " << s << " (HH:MM:SS)" << endl;

	// Sleep for a period as require

	// Determine if still should be active 
	// end of while

	// Clean up
	delete FDMExec;  

	return 0;
}

bool 
pfms_init(JSBSim::FGFDMExec *FDMExec)
{	
	// Required to link to ap

	// Define simulation timing 	
	//JSBSim::FGState *State = FDMExec->GetState(); // Creates a pointer to the State object of FDMExec	   
	//State->Setdt(0.02); 
	//State->Setsim_time(StartTime);
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

	/*// Engage engines if required to initially be in running state	
	// Works but not required
	JSBSim::FGPropulsion *propulsion = FDMExec->GetPropulsion();
	for(int i=0; i<propulsion->GetNumEngines(); i++) {
		propulsion->GetEngine(i)->SetRunning(true);		
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
	FDMExec->SetPropertyValue("ap/heading-setpoint-select", 1);
	FDMExec->SetPropertyValue("ap/heading_hold", 1);
	FDMExec->SetPropertyValue("ap/active-waypoint", 1);
	// Raise landing gear
	FDMExec->SetPropertyValue("gear/gear-cmd-norm", 0);
	// Assign altitude hold
	FDMExec->SetPropertyValue("ap/altitude_hold", 1);	

	/*
	// Trim the Aircraft required eventually
	JSBSim::FGTrim fgt(FDMExec, JSBSim::tLongitudinal);
	if( !fgt.DoTrim() ) 
	{      
	fgt.Report();
	fgt.TrimStats();
	exit(-1);
	}	
	*/

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
	}
}

void 
pfms_readJSBSimMessages(JSBSim::FGFDMExec* FDMExec)
{
	FGJSBBase::Message* msg = NULL;

	while (FDMExec->SomeMessages()) {
		msg = FDMExec->ProcessMessage();
		switch (msg->type) {
		case FGJSBBase::Message::eText:
			cout << msg->messageId << ": " << msg->text << endl;
			break;
		case FGJSBBase::Message::eBool:
			cout << msg->messageId << ": " << msg->text << " " << msg->bVal << endl;
			break;
		case FGJSBBase::Message::eInteger:
			cout << msg->messageId << ": " << msg->text << " " << msg->iVal << endl;
			break;
		case FGJSBBase::Message::eDouble:
			cout << msg->messageId << ": " << msg->text << " " << msg->dVal << endl;
			break;
		default:
			cerr << "Unrecognized message type." << endl;
			break;
		}
	}
}

void
pfms_publish_config(JSBSim::FGFDMExec *FDMExec)
{	
}
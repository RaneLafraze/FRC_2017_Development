
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Notice that sometimes '.h' files are included, and other times '.hpp' files
// are included. My convention for using both naming schemes is to identify
// classes that contain all (or most) of their method definitions in the
// header file (.hpp) versus having just declarations in the header (.h) and
// putting definitions in an accompanying .cpp file.
//................................................................................

#include "./GM_Code/gmCamera.h"
#include "./GM_Code/gmDriveControl.h"
#include "./GM_Code/gmJoystick.hpp"
#include "./GM_Code/gmServer.hpp"
#include "./GM_Code/gmDedRec.hpp"
using namespace GM_Code;
#include "CANTalon.h"
#include "WPILib.h"
#include <SampleRobot.h>
#include <DriverStation.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cmath>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
using namespace std;
#include <chrono>
using namespace std::chrono;

deque<json> cmdDeque;
std::mutex cmdMutex;

void addRobotCommand(json cmd)
{
	unique_lock<mutex> lockit(cmdMutex);
	cmdDeque.push_back(cmd);

	char str[80];
	sprintf(str, "Added Robot Command");
	DriverStation::ReportError(str);
}

bool isRobotCommand()
{
	unique_lock<mutex> lockit(cmdMutex);
	return !cmdDeque.empty();
}

json getRobotCommand()
{
	unique_lock<mutex> lockit(cmdMutex);
	json j;
	if (!cmdDeque.empty())
		j = cmdDeque.front();
	cmdDeque.pop_front();
	return j;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Robot -- Main class; runs the robot.
// -----
// See RobotBase and SampleRobot class documentation for
// methods that we override.
//............................................................
class Robot : public frc::SampleRobot
{

	static const Robot* this_robot;

	CANTalon* leftDriveMaster = new CANTalon(1);
	CANTalon* rightDriveMaster = new CANTalon(4);
	CANTalon* leftDriveSlave1 = new CANTalon(2);
	CANTalon* leftDriveSlave2 = new CANTalon(3);
	CANTalon* rightDriveSlave1 = new CANTalon(5);
	CANTalon* rightDriveSlave2 = new CANTalon(6);
	CANTalon* shooterMotor = new CANTalon(7);

	VictorSP* intakeMotor1 = new VictorSP(9);
	VictorSP* intakeMotor2 = new VictorSP(5);
	VictorSP* intakeMotor3 = new VictorSP(7);
	VictorSP* climbMotor = new VictorSP(8);

	Compressor* compressor = new Compressor(0);
	Solenoid* shifterSolenoid = new Solenoid(0);

	RobotDrive* drive = new RobotDrive(leftDriveMaster, rightDriveMaster); //CANTalon

	//frc::RobotDrive drive{1 /*front left*/, 0 /*rear left*/, 2 /*front right*/, 3 /*rear right*/}; // HGM's 4 motor drive.

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// In general it's wise to allocate memory on the 'heap'
	// rather than on the stack or in 'static'' memory (wherever
	// that is (?)). Especially true for objects that consume lots
	// of memory and are instanced many times during execution.
	//
	// In this case, static variables (like 'drive') must live
	// until the program is terminated so it doesn't matter.
	// My understanding of what's best here is not perfect. (!)
	//............................................................

	std::unique_ptr<gmJoystick> driveStick {new gmJoystick(0)};
	std::unique_ptr<gmJoystick> shootStick {new gmJoystick(1)};

	std::unique_ptr<gmDriveControl> driveControl {new gmDriveControl()};
    std::unique_ptr<gmDedRec> dedRec;

    std::mutex m;
    std::condition_variable cv;

    bool action_running = false;
    int lastButtons = 0;;
    int recording = 0;
    high_resolution_clock::time_point macroBgn, macroEnd;

    double autoDistance = 95.3;
    double autoDistance2 = 35.25;
    int autoCounter = 0;
	bool ticksReached = false;
	bool angleReached = false;

    void print(string s)
	{
    	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    	// For testing, this seems like the most sure way to
    	// display something on the Driver Station.
    	//............................................................
		DriverStation::ReportError(s);
    }
    friend json getDriveMacros();
    friend void putDriveMacros(json &macros);
    friend json readDriveMacros();
    friend void saveDriveMacros(json &macros);
    friend bool isButtonChange();

  public:


    Robot()
    {
    	this_robot = this;
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// Set the timeout for the MotorSafetyHelper. Be sure to
		// update the motor outputs more frequently than this. (!)
		//............................................................

		drive->SetExpiration(0.1); // Every 1/10th second.
    }

    void RobotInit()
    {

        json macros = getDriveMacros();
    	leftDriveSlave1->SetControlMode(CANSpeedController::kFollower); //change control mode to follow mode
    	leftDriveSlave2->SetControlMode(CANSpeedController::kFollower); //change control mode to follow mode
    	rightDriveSlave1->SetControlMode(CANSpeedController::kFollower); //change control mode to follow mode
    	rightDriveSlave2->SetControlMode(CANSpeedController::kFollower); //change control mode to follow mode
    	leftDriveSlave1->Set(1); //follow left motor #1 Talon SRX ID=1
    	leftDriveSlave2->Set(1); //follow left motor #1 Talon SRX ID=1
    	rightDriveSlave1->Set(4); //follow right motor #1 Talon SRX ID=4
    	rightDriveSlave2->Set(4); //follow right motor #1 Talon SRX ID=4

    	leftDriveMaster->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
    	leftDriveMaster->ConfigEncoderCodesPerRev(360);
    	leftDriveMaster->SetPosition(0); //set encoder to 0
    	rightDriveSlave2->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
    	rightDriveSlave2->ConfigEncoderCodesPerRev(360);
    	rightDriveSlave2->SetPosition(0);
    	shooterMotor->SetControlMode(CANSpeedController::kVoltage);

		this->print("RobotInit");

		std::string s = macros.dump();
		if (s.length() > 4)
		{
			char str[80];
			sprintf(str, "Macros size = %d", s.length());
			DriverStation::ReportError(str);

			putDriveMacros(macros);
		}
		else
		{
			DriverStation::ReportError("Macro file doesn't exist!");
		}

		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// It takes a bit before the navX-MXP is fully running, so
		// wait a couple of secs before resetting it.
		//............................................................
		std::thread t_1([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
			// ahrs->ZeroYaw();

			dedRec = unique_ptr<gmDedRec>(new gmDedRec());
		});
		t_1.detach();

#if 0
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// Okay... First off, this code seems totally sufficient for
		// running one or more camera servers automatically...
		//............................................................
	    CameraServer::GetInstance()->StartAutomaticCapture(0).SetResolution(640, 360);
	    CameraServer::GetInstance()->StartAutomaticCapture(1).SetResolution(640, 480);
#else
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// ... But to add OpenCV image processing, it *is* more
	    // complicated. See the gmCamera class for more info. (!)
		//............................................................

		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// Run video capture(s) asynchronously.
		//........................................
		std::thread t_2([]() {
			// Logitech C270 in (outer) USB port. 1280 x 720 is possible but too slow.
			gmCamera::StartCapturing("1st Camera", 0, 640, 360, 0/*delay_ms*/, gmCamera::toEdges/*or nullptr*/);
		});
		t_2.detach();

		std::thread t_4([]() {
			// Other camera in (inner) USB port...
			gmCamera::StartCapturing("2nd Camera", 1, 320, 240, 25/*delay_ms*/, gmCamera::toBW);
		});
		t_4.detach();
#endif

        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Run a TCP/IP server asynchronously.
        //........................................
        std::thread t_3([]() {
            // Wait for the robot code to fully initialize...
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // ...  Then start the server.
			gmServer::StartListening();
        });
        t_3.detach();
    }

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Below we override each of the three operational modes
    // selectable from the FRC Driver Station.
    //
    // (>>>) Note that 'Practice' mode (on the Driver Station)
    //       controls the auto/teleop timing to simulate a match.
    //............................................................

    void Autonomous()
    {
		this->print("Autonomous Begin");
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// In a match (according to 'Practice') motor safety might be
		// forced on during autonomous. Better stay 'responsive'!
		//............................................................
		drive->SetSafetyEnabled(true);
		shifterSolenoid->Set(true);
		// float yawAngle = 0.0;   // Not used
		float yawThreshold = .5;
		// yawAngle = dedRec->getYawAngle();
		leftDriveMaster->SetPosition(0);
		rightDriveSlave2->SetPosition(0);
		double ratio = 0.1300752291666667;
		double error = 0.2930555;
		double autoTickGoal = ratio * autoDistance;
		double autoTickGoal2 = ratio * autoDistance2;
		double autoAngleGoal = 0.0;
		int nloops = 0;
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// The robot heading is relative to its position/orientation
		// at the start of autonomous. So reset it here.
		//............................................................

		while (IsAutonomous() && IsEnabled() && autoCounter < 1)
		{


			/*if (-leftDriveMaster->GetPosition() < autoTickGoal)
			{
				drive->SetLeftRightMotorOutputs(-.2, -.2);

				if (dedRec->getYawAngle() < yawAngle - yawThreshold)
				{
					//increase left motor speed
					drive->SetLeftRightMotorOutputs(-.3, -.2);
				}
				else if (dedRec->getYawAngle() > yawAngle + yawThreshold)
				{
					//increase right motor speed
					drive->SetLeftRightMotorOutputs(-.2, -.3);
				}
			}
			else if (-leftDriveMaster->GetPosition() > autoTickGoal)
			{
				drive->SetLeftRightMotorOutputs(0,0);
			}*/

			if (-leftDriveMaster->GetPosition() < autoTickGoal && ticksReached == false)
			{
				drive->SetLeftRightMotorOutputs(-.2, -.2);

				// Here, autoAngleGoal = 0.0
				if (dedRec->getYawAngle() < autoAngleGoal - yawThreshold)
				{
					//increase left motor speed
					drive->SetLeftRightMotorOutputs(-.3, -.2);
				}
				else if (dedRec->getYawAngle() > autoAngleGoal + yawThreshold)
				{
					//increase right motor speed
					drive->SetLeftRightMotorOutputs(-.2, -.3);
				}

				else if (-leftDriveMaster->GetPosition() > autoTickGoal)
				{
					ticksReached = true;
					autoAngleGoal = 60.0; // Set to 60 degrees for the next part of Autonomous
				}
			}
			if (ticksReached == true && dedRec->getYawAngle() < autoAngleGoal && angleReached == false)
			{
					drive->SetLeftRightMotorOutputs(-.1,.1);
			}
			else if (dedRec->getYawAngle() > autoAngleGoal)
			{
				// Loop until the robot is at 60 degrees
				while(dedRec->getYawAngle() < autoAngleGoal + yawThreshold || dedRec->getYawAngle() > autoAngleGoal - yawThreshold)
				{
					if (dedRec->getYawAngle() < autoAngleGoal - yawThreshold)
					{
						//increase left motor speed
						drive->SetLeftRightMotorOutputs(-.3, -.2);
					}
					else if (dedRec->getYawAngle() > autoAngleGoal + yawThreshold)
					{

					//increase right motor speed
						drive->SetLeftRightMotorOutputs(-.2, -.3);
					}
					// Once the robot is at 60 degrees, the loop will terminate
				}

				angleReached = true;
				leftDriveMaster->SetPosition(0);
			}


			if (angleReached == true && -leftDriveMaster->GetPosition() < autoTickGoal2)
			{
					drive->SetLeftRightMotorOutputs(-.2, -.2);

					// Here, autoAngleGoal = 60 degrees
					if (dedRec->getYawAngle() < autoAngleGoal - yawThreshold)
					{
						//increase left motor speed
						drive->SetLeftRightMotorOutputs(-.3, -.2);
					}
					else if (dedRec->getYawAngle() > autoAngleGoal + yawThreshold)
					{
					//increase right motor speed
						drive->SetLeftRightMotorOutputs(-.2, -.3);
					}

					else if (angleReached == true && -leftDriveMaster->GetPosition() > autoTickGoal2)
				{
					drive->SetLeftRightMotorOutputs(0,0);
				}
			}
			if (0 == (nloops++ % 200))
			{
				char str[80];
				sprintf(str,"Yaw %f", dedRec->getYawAngle());
				DriverStation::ReportError(str);
			}

			/*int macro_id = 1;
			if (driveControl->isMacro(macro_id)) // ... Else, play the macro, if it exists.
			{
				char str[80];
				sprintf(str, "Playing macro %d", macro_id);
				DriverStation::ReportError(str);

				// Here's an example of a concurrent process; for example
				// shooting while continuing to to drive or whatever.

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				// As we execute the macro, the teleop loop will block
				// because we're waiting for this thread by using join().
				//............................................................
				std::thread macro_thread([this, macro_id]() {
					driveControl->playMacro(macro_id, drive);
					autoCounter++;
				});
				macro_thread.join();

				sprintf(str, "Done playing macro %d", macro_id);
				this->print(autoCounter + " times");
				DriverStation::ReportError(str);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5));*/
		}
		this->print("Autonomous End");
    }

    void OperatorControl() override
    {
		this->print("TeleOp Begin");

		drive->SetSafetyEnabled(true);
		int nloops = 0;
		int reset = 0;
		while (IsOperatorControl() && IsEnabled())
		{

			//This is for the commands sent from the Jetson!
			if(!action_running && isRobotCommand())
			{
				char str[80];
				sprintf(str, "Command Executing");
				DriverStation::ReportError(str);

				while(isRobotCommand())
				{
					json cmd = getRobotCommand();
					int ms = cmd["duration"];
					float motorL = cmd["motorL"];
					float motorR = cmd["motorR"];

					driveControl->setMotors(drive, motorL, motorR);
					std::this_thread::sleep_for(std::chrono::milliseconds(ms));
				}
			}

			compressor->Start();
			//compressor->SetClosedLoopControl(true);
			int buttons = driveStick->getJoystickButtons();

			int shiftButtonDriveStick = driveStick->getJoystickButton(5);
			int shootButtonShootStick = shootStick->getJoystickButton(1);
			int intakeButtonShootStick = shootStick->getJoystickButton(2);
			int climbButtonDriveStick = driveStick->getJoystickButton(3);
			int roll = shootStick->getJoystickButton(4);
			int driveStraightButtonDriveStick = driveStick->getJoystickButton(1);

			if (driveStraightButtonDriveStick == 1)
			{
				float yawAngle = 0.0;
				float yawThreshold = 1.0;

				if (reset < 1)
					{
						yawAngle = dedRec->getYawAngle();
						reset++;
					}

				drive->SetLeftRightMotorOutputs(-.2, -.225);

				if (dedRec->getYawAngle() < yawAngle - yawThreshold)
				{
					//increase left motor speed
					drive->SetLeftRightMotorOutputs(-.3, -.2);

				}
				else if (dedRec->getYawAngle() > yawAngle + yawThreshold)
				{
					//increase right motor speed
					drive->SetLeftRightMotorOutputs(-.2, -.3);
				}

			}
			else
			{
				reset = 0;
			}
			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Activate solenoid if button 2 on driveStick
			// is pressed. On release deactivate solenoid.
			//..................................................

			if (shiftButtonDriveStick == 1)
			{

				shifterSolenoid->Set(true);
			}
			else if (shiftButtonDriveStick == 0)
			{
				shifterSolenoid->Set(false);
			}

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Run shooter motor and intake motors to shoot balls
			// if button 1 on shootStick (XBOX) is pressed
			//....................................................

			if (shootButtonShootStick == 1)
			{

				shooterMotor->Set(9.0);
				//shooterMotor->Set(0.9);
				intakeMotor1->Set(-1);
				intakeMotor2->Set(-1);
				intakeMotor3->Set(-.90);
			}

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Run intake motors if button 2 on either joystick is pressed
			//....................................................
			if (roll == 1)
			{

				shooterMotor->Set(9.0);
			}

			if (intakeButtonShootStick == 1)
			{
				intakeMotor1->Set(-1);
				intakeMotor2->Set(-1);
				intakeMotor3->Set(1);
			}

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Run climb motor if button 3 on driveStick is pressed or
			// button 4 on shootStick is pressed.
			//...........................................................

			if (climbButtonDriveStick == 1)
			{
				climbMotor->Set(1);
			}

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Check if button is pressed, if not set motors to 0
			//....................................................

			if ((shootButtonShootStick == 0 && intakeButtonShootStick == 0) && (climbButtonDriveStick == 0) && (roll == 0))
			{
				shooterMotor->Set(0);
				intakeMotor1->Set(0);
				intakeMotor2->Set(0);
				intakeMotor3->Set(0);
				climbMotor->Set(0);
			}

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Just for testing, report encoder positions every ~1 sec
			// Motors are powered while button is held.
			//........................................................

			if (0 == (nloops++ % 200))
			{
				char str[80];
				sprintf(str, "Left Encoder position %f", leftDriveMaster->GetPosition());
				DriverStation::ReportError(str);
				sprintf(str, "Right Encoder position %f", rightDriveSlave2->GetPosition());
				DriverStation::ReportError(str);
			}

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Just for testing, report heading every ~1 sec.
			//..................................................
			/*if (0 == (nloops++ % 200))
			{
				char str[80];
				float yaw = dedRec->getYawAngle();
				yaw = (float)(int)(yaw * 100 + 0.5) / 100.0;
				sprintf(str, "yaw = %f", yaw);
				this->print(str);
			}
			*/

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Macro recording and playback handling...
			//==================================================

			if (buttons != 0 && buttons != lastButtons)
			{
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				// 'macro_mask' is the bit-mask of possible macro buttons.
				// Because it depends on specific knowledge about the joystick
				// that isn't presented here, I consider this a hack that
				// should generally be avoided. (!!!)
				//............................................................
				const int macro_mask = 0xFC0;
				int macro_id = (buttons & macro_mask) >> 6;

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				// (1) One of the macro buttons is (newly) pressed. See (2)...
				//............................................................
				if (macro_id != 0)
				{
					if ((buttons & 2) != 0) // If 'shift' is also pressed, record a macro...
					{
						char str[80];
						sprintf(str, "Recording macro %d", macro_id);
						DriverStation::ReportError(str);

						macroBgn = high_resolution_clock::now();
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						// 'recording' is both a "recording-in-progress" flag,
						// and the identifier of the macro being recorded.
						//............................................................
						recording = macro_id;
					}
					else if (recording != 0) // ... Else, if recording, stop recording...
					{
						macroEnd = high_resolution_clock::now();
						bool err = driveControl->saveMacro(recording, macroBgn, macroEnd);
						recording = 0;

						if (err)
						{
							DriverStation::ReportError("Macro not recorded.");
						}
						else
						{
							char str[80];
							sprintf(str, "Done recording macro %d", macro_id);
							DriverStation::ReportError(str);
						}
					}
					else if (driveControl->isMacro(macro_id)) // ... Else, play the macro, if it exists.
					{
						char str[80];
						sprintf(str, "Playing macro %d", macro_id);
						DriverStation::ReportError(str);

						// Here's an example of a concurrent process; for example
						// shooting while continuing to to drive or whatever.

						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						// As we execute the macro, the teleop loop will block
						// because we're waiting for this thread by using join().
						//............................................................
						lastButtons = buttons; // (!) Important.
						std::thread macro_thread([this, macro_id]() {
							driveControl->playMacro(macro_id, drive);
						});
						macro_thread.join();
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						// If a button was pressed to terminate the macro, don't
						// let it trigger something else. So, update 'buttons',
						// which, in turn, will update 'lastButtons'. See (L)...
						//............................................................
						buttons = driveStick->getJoystickButtons();

						sprintf(str, "Done playing macro %d", macro_id);
						DriverStation::ReportError(str);
					}
				}
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				// ... (2) Some other button was newly pressed. (Not 'shift'.)
				//............................................................
				else if (!action_running && 0 == (buttons & 2))
				{
					//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
					// Here's an example of a concurrent process; for example
					// shooting while continuing to to drive or whatever.
					//............................................................
					action_running = true;
					std::thread auto_thread([this]() {
				    	const float in_per_m = 39.3701f;
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						// In this case, just count down for 10 secs and report the
				    	// position returned from 'dedRec' every second.
				    	// (Just a test; not practical for anything.)
						//............................................................
						dedRec->setPosition(point_t(0, 0, 0));

						char str[80];
						int n = 10;
						while (n >= 0)
						{
							point_t pos = dedRec->getPosition();

							float d = std::sqrt(pos.x * pos.x + pos.y * pos.y/* + pos.z * pos.z*/);

							sprintf(str, "%d : %f %f %f -- %f", n--, pos.x * in_per_m, pos.y * in_per_m, pos.z * in_per_m, d);
							DriverStation::ReportError(str);
							if (n < 0) break;

							std::this_thread::sleep_for(std::chrono::seconds(1));
						}
						action_running = false;
					});
					auto_thread.detach();
				}


			}
			lastButtons = buttons; // ... (L)
			//========================================
			// ... End of macro-handling code.
			//........................................

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Drive the robot... drive the robot...
			//==================================================
			float x, y, z, throttle;
			driveStick->getJoystickValues(x, y, z, throttle);

			driveControl->addInput(x, y, z, throttle);

			float motorR, motorL;
			if (driveControl->getMotor(motorR, motorL) && driveStraightButtonDriveStick == 0)
			{
				driveControl->setMotors(drive, motorL, motorR);
			}
			//==============================
			// ... End of driving code.
			//..............................

			// const gmDriveControl::dspy_s& dspyValues = driveControl->getDspyValues();

			//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			// Note that the motors are updated every 5ms; and that
			// new joystick values are updated every 20ms. Hmm...
			//----
			// Since we won't get new inputs faster than every 20ms,
			// I guess we should wait that long until our next pass
			// through the loop. (?)
			//----
			// (!) If this delay is changed, it will affect the
			//     amount of available macro recording.
			//............................................................
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		this->print("TeleOp End");
    }

    void Test() override
    {
		this->print("Test Begin");
		drive->SetSafetyEnabled(false);

		while (IsTest() && IsEnabled())
		{
			// ...

			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		this->print("Test End");
    }

    void Disabled() override
    {
		//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// During a match, disabled is called after both autonomous
		// and after teleop... according to 'Practice' simulation.
		//............................................................
		this->print("Disabled");

		driveControl->setMotors(drive, 0, 0);
		driveControl->clear();
    }
};

const Robot* Robot::this_robot;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Below are some 'global' functions that interact with
// elements from our Robot class.
//............................................................

json getDriveMacros()
{
	json macroS_j;

	vector<int> ids = Robot::this_robot->driveControl->getMacros();
	for (auto i : ids)
	{
		json macro_j;
		macro_j["id"] = i;
		macro_j["data"] = json::array();

		auto macro = Robot::this_robot->driveControl->getMacro(i);

	    for (shared_ptr<gmDriveControl::motors_s> &item : *macro)
	    {
	        json item_j = {
	        		{ "ts", item->ts },
					{ "l", (int)(item->l * 10000 + 0.5f) },
					{ "r", (int)(item->r * 10000 + 0.5f) }
	        };
	        macro_j["data"].push_back(item_j);
	    }
	    macroS_j.push_back(macro_j);
	}
	return macroS_j;
}

void putDriveMacros(json &macros_j)
{
	for (auto i : macros_j)
	{
		shared_ptr<deque<shared_ptr<gmDriveControl::motors_s>>> macro(new deque<shared_ptr<gmDriveControl::motors_s>>);

		for (auto j : i["data"])
		{
			float motorL = j["l"], motorR = j["r"], ts = j["ts"];
			motorL /= 10000.0f; motorR /= 10000.0f;
			macro->push_back(unique_ptr<gmDriveControl::motors_s>(new gmDriveControl::motors_s(motorR, motorL, ts)));
		}
		Robot::this_robot->driveControl->putMacro(i["id"], macro);
	}
}

const string filename = "./robot_macros.json";

json readDriveMacros()
{
	json macros_j;
	ifstream file(filename);
	if (file.is_open())
	{
		file >> macros_j;
		file.close();
	}
	return macros_j;
}

void writeDriveMacros(json &macros_j)
{
	ofstream file(filename);
	if (file.is_open())
	{
		file << macros_j;
		file.close();
	}
}


bool isButtonChange()
{
	int buttons = Robot::this_robot->driveStick->getJoystickButtons();

	if (buttons != 0 && buttons != Robot::this_robot->lastButtons)
		return true;
	return false;
}

START_ROBOT_CLASS(Robot)




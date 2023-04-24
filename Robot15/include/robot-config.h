using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;

extern smartdrive Drivetrain;
extern motor LeftFrontMotor;
extern motor LeftMiddleMotor;
extern motor LeftBackMotor;
extern motor RightFrontMotor;
extern motor RightMiddleMotor;
extern motor RightBackMotor;
extern inertial DrivetrainInertial;

extern motor FlywheelDown;
extern motor FlywheelUp;
extern motor_group Flywheel;

extern motor Expansor;
extern motor_group Intake_roller;
extern pneumatics Indexer;

/* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * This should be called at the start of your int main function. */
// Main functions
extern int rc_auto_loop_function_Controller1();
void vexcodeInit( void );
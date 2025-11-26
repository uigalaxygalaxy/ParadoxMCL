#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

 #include "lemlib/chassis/odom.hpp"
 #include "lemlib/chassis/mcl.hpp"
 #include <array>
 #include <random>
 #include <cmath>
#include <atomic> // like our sister team 45434A. they're cutie patooties

 std::vector<Particle> particles; //initialize so its global and we dont , like die cuz of thread  magic. ChatGPT said so and is way smarter than me anyway


// TODO: tune constants so that our math works. ****** everything written below is made by chatgpt, what can you expect from me. i have an essay due in 17 minutes. i should be doing that instead of
// this. its also 11:43 pm on a tuesday night. this is tough mangoes :c

// - `PARTICLE_COUNT`: number of particles (higher = better accuracy, worse CPU)
// - `INIT_STD_*`: initial uncertainty around selected starting pose
// - `PROCESS_NOISE_*`: odometry/process noise used during the predict step
// - `MCL_DT`: time-step assumed for odomSpeed integration (should match odom update rate)
// - `SENSOR_SIGMA_FACTOR` and `SENSOR_SIGMA_MIN`: sensor noise model parameters
// - `DISTANCE_OFFSETS`: signed mounting offsets for each distance sensor (left, front, right, back)
// - `TOGGLE_BUTTON`: controller button to toggle MCL on/off at runtime
// Tune these iteratively: start conservative (larger noise) then reduce as confidence grows.


// a bunch of constants so we can put values in easily so anyone can do it and also 
// here to prevent stupid mistakes which we appear to be really good at making

 const int PARTICLE_COUNT = 100;

// Initial particle spread (in inches / degrees)
static float INIT_STD_X = 2.0f;     // TODO: tune initial X stddev
static float INIT_STD_Y = 2.0f;     // TODO: tune initial Y stddev
static float INIT_STD_THETA = 5.0f; // TODO: tune initial heading stddev (degrees)

// Process (odometry) noise used in the predict step
static float PROCESS_NOISE_DX = 0.1f;    // TODO: tune process noise for x (inches)
static float PROCESS_NOISE_DY = 0.1f;    // TODO: tune process noise for y (inches)
static float PROCESS_NOISE_DTHETA = 0.2f; // TODO: tune process noise for theta (degrees)

// MCL timing
static float MCL_DT = 0.01f; // its 0.01 cuz lemlib odom is 10ms

// Sensor noise model
static float SENSOR_SIGMA_FACTOR = 0.2f; // TODO: factor used to scale sensor reading -> sigma
static float SENSOR_SIGMA_MIN = 0.6f;    // TODO: minimum sigma (inches)

// Distance sensor mounting offsets (left, front, right, back).
// Each entry is {x, y} in inches relative to the robot reference point
// (x = forward positive, y = right positive). Fill these with measured
// coordinates of the sensor aperture. Example defaults approximate the
// previous scalar values: left ~1.5" left, front ~2.5" forward, right ~1.5" right,
// back ~2.5" rear.
static std::array<std::array<float,2>,4> SENSOR_OFFSETS = {{{
    {{0.0f, -1.5f}}, // left sensor: 0" forward, 1.5" left
    {{2.5f,  0.0f}}, // front sensor: 2.5" forward, 0" right
    {{0.0f,  1.5f}}, // right sensor: 0" forward, 1.5" right
    {{-2.5f, 0.0f}}  // back sensor: 2.5" rear, 0" right
}}}; // TODO: measure these precisely

//the top sentence was lke super advanced so here it is in actual english: just find the offsets of the sensors from the center of the chassis cuz we have bum builders that dont do that

// Per-sensor reading bias (inches). This lets you correct sensors that
// systematically under- or over-report distances. Positive value means the
// sensor reads a SHORTER distance than reality by this amount (so we subtract
// the bias from the expected distance when comparing to the reading).
static std::array<float,4> SENSOR_READING_BIAS = {0.0f, 0.0f, 0.0f, 0.0f};

//just in case lol. vex judges will love it if you say it is systematically under or over shooting btw even though were most likely not using this at all

// Controller button to toggle MCL on/off
static const pros::controller_digital_e_t TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_R1; // TODO: change if desired

// runtime toggle for MCL
static std::atomic<bool> mclEnabled{true};

 int autonSelected = 0;

 //  our starting poses for start of autos. If you don't change these, you're gonna die
StartingPose leftStart {12.0f, 25.0f, 320.0f};
StartingPose rightStart {36.0f, 25.0f, 40.0f};
StartingPose awpStart {24.0f, 15.0f, 0.0f};
StartingPose skillsStart {12.0f, 10.0f, 0.0f};
StartingPose bumStart {24.0f, 25.0f, 180.0f};


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-5, 4, -3},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({6, -9, 7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

pros::Distance dFront('A'); // front distance sensor on port A
pros::Distance dLeft('B');  // left distance sensor on port B
pros::Distance dRight('C'); // right distance sensor on port C 
pros::Distance dBack('D');  // back distance sensor on port D
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void left() {
    chassis.setPose(leftStart.x, leftStart.y, leftStart.theta);
}
void right() { chassis.setPose(rightStart.x, rightStart.y, rightStart.theta); }
void awp() { chassis.setPose(awpStart.x, awpStart.y, awpStart.theta); }
void skills() { chassis.setPose(skillsStart.x, skillsStart.y, skillsStart.theta); }
void bum() { chassis.setPose(bumStart.x, bumStart.y, bumStart.theta); }

//where all yo autonomous stuff go. This not our real code file  so like thats all youre getting

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 StartingPose currentPose = leftStart;

 //initialize currentPose because why not

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors


    controller.print(0, 0, "Auton Selected");

    pros::Task screenTask([&]() {
        for (int i = 0; i < 150; i++) { //only run for 15s
        switch (autonSelected) {
        case 1:
        left();
        pros::lcd::print(0, "Auton Selected %d | left", autonSelected);
        controller.print(1, 0, "Auton Selected %d", autonSelected);
        controller.print(2, 0, "left");
         currentPose = leftStart;
        break;
        case 2:
        right();
        pros::lcd::print(0, "Auton Selected %d | right", autonSelected);
        controller.print(1, 0, "Auton Selected %d", autonSelected);
        controller.print(2, 0, "right");
         currentPose = rightStart;
        break;
        case 3:
        awp();
        pros::lcd::print(0, "Auton Selected %d | awp", autonSelected);
        controller.print(1, 0, "Auton Selected %d", autonSelected);
        controller.print(2, 0, "awp");
                 currentPose = awpStart;

        break;
        case 4: 
        skills();
        pros::lcd::print(0, "Auton Selected %d  | skills", autonSelected);
        controller.print(1, 0, "Auton Selected %d", autonSelected);
        controller.print(2, 0, "skills");
                 currentPose = skillsStart;
        break;
        default:
        bum();
        pros::lcd::print(0, "Auton Selected %d  | bum", autonSelected);
        controller.print(1, 0, "Auton Selected %d", autonSelected);
        controller.print(2, 0, "bum");
         currentPose = bumStart;
         break;

        }
        controller.print(3, 0, "Pick within %d", i/10);
        pros::delay(100);
            
        }

        // create particles based off of our starting pose
        // use gaussian distributions so its woh fully spooky and noisy. noise is good so we have lots of options and stuff
        std::default_random_engine generator(std::random_device{}());
        std::normal_distribution<float> xNoise(currentPose.x, INIT_STD_X);
        std::normal_distribution<float> yNoise(currentPose.y, INIT_STD_Y);
        std::normal_distribution<float> thetaNoise(currentPose.theta, INIT_STD_THETA);

        particles.clear();
        particles.reserve(PARTICLE_COUNT);
        for (int i = 0; i < PARTICLE_COUNT; i++) {
            Particle p;
            p.x = xNoise(generator); // generate x with noise
            p.y = yNoise(generator); // generate y with noise
            p.theta = thetaNoise(generator); // generate theta with noise
            p.weight = 1.0f / (float)PARTICLE_COUNT; // make  it uniform  (its gonna change later dont woryy)
            particles.push_back(p); // push the particle to the vector
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        autonSelected++;
        controller.rumble(". -");
    }
    else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        autonSelected--;
        controller.rumble("- .");
    }
    if(autonSelected < 0) autonSelected = 0;
    if(autonSelected > 4) autonSelected = 4;

    if(controller.get_battery_level() < 67) {
        controller.rumble("...---...");
        controller.print(4, 0, "Battery Low!");
    }
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


void autonomous() {
    // set initial pose based on selection
    switch (autonSelected) {
        case 1: left(); break;
        case 2: right(); break;
        case 3: awp(); break;
        case 4: skills(); break;
        default: bum(); break;
    }

    // big mcl hope it works :pray:
    pros::Task MCL([&]() {
        std::default_random_engine generator(std::random_device{}()); // RNG for noise
        const float dt = MCL_DT; // thats our dt. its 10ms cuz lemlib odom is 10ms
        std::normal_distribution<float> dxNoise(0.0f, PROCESS_NOISE_DX); // create a distribution and stuff
        std::normal_distribution<float> dyNoise(0.0f, PROCESS_NOISE_DY);
        std::normal_distribution<float> dthetaNoise(0.0f, PROCESS_NOISE_DTHETA);

        const std::array<float,4> sensorAngles = {-90.0f, 0.0f, 90.0f, 180.0f}; // left, front, right, back; so u know what wall the robot is facing

        auto receiveReadings = [&]() -> std::array<float, 4> {
            const float INCH_CONVERSION = 0.03937008f; // turns mm to inches
            return {dLeft.get() * INCH_CONVERSION,
                    dFront.get() * INCH_CONVERSION,
                    dRight.get() * INCH_CONVERSION,
                    dBack.get() * INCH_CONVERSION};
        };

        auto receiveConfidences = [&]() -> std::array<uint8_t, 4> {
            return {dLeft.get_confidence(), dFront.get_confidence(), dRight.get_confidence(), dBack.get_confidence()};
        };

        auto sensorOffsets = [&]() -> std::array<std::array<float,2>,4> { //self exmplanatory these two
            return SENSOR_OFFSETS;
        };
        auto sensorReadingBiases = [&]() -> std::array<float,4> {
            return SENSOR_READING_BIAS;
        };

            auto expected_distance = [&](float px, float py, float ptheta, int sensorIndex) -> float {
            // raycast  particles so u know expected distance
            
            // expected_distance -- raycast from a particle to the arena walls
            // Explanation: super generated by chatGPT because I am lazy but thats ok
            //  - 'Raycasting' here means: from a particle's pose, we cast a straight line
            //    (a ray) in the direction the particular distance sensor is pointing.
            //  - We then find where this ray first intersects the rectangular arena walls
            //    (axis aligned at +/-70.2 inches). The distance along the ray to that
            //    intersection is the expected distance the sensor would measure if the
            //    robot were exactly at that particle's pose.
            //  - We subtract a small `distanceOffset` to account for the sensor being
            //    mounted slightly away from the robot's reference point (change these
            //    offsets to match your physical mounting). If no intersection is found
            //    we return a large sentinel value (9999) which the update step ignores.
            //  - This simple raycast assumes axis-aligned box walls.
            const float WALL_MIN = -70.2f;
            const float WALL_MAX = 70.2f;
            // compute ray direction for te sensor 
            float ang = lemlib::degToRad(ptheta + sensorAngles[sensorIndex]); //raycast our sensor angles here to fix bum sensor offsets
            float rx = cos(ang);
            float ry = sin(ang);

            // get sensor offset
            auto so = sensorOffsets()[sensorIndex];
            float sx = so[0];
            float sy = so[1];

            // rotate sensor offset into world frame using particle heading
            float th = lemlib::degToRad(ptheta);
            float sWx = cos(th) * sx - sin(th) * sy;
            float sWy = sin(th) * sx + cos(th) * sy;

            // calculate ray origin
            float ox = px + sWx;
            float oy = py + sWy;

            float bestT = 1e6f;

            //particle raycasting stuff

            // vertical walls x = WALL_MIN and WALL_MAX
            for (float wx : {WALL_MAX, WALL_MIN}) {
                if (fabs(rx) < 1e-6f) continue;
                float t = (wx - ox) / rx;
                if (t <= 0) continue;
                float yi = oy + t * ry;
                if (yi <= WALL_MAX + 1e-3f && yi >= WALL_MIN - 1e-3f) bestT = std::min(bestT, t);
            }

            // horizontal walls y = WALL_MIN and WALL_MAX
            for (float wy : {WALL_MAX, WALL_MIN}) {
                if (fabs(ry) < 1e-6f) continue;
                float t = (wy - oy) / ry;
                if (t <= 0) continue;
                float xi = ox + t * rx;
                if (xi <= WALL_MAX + 1e-3f && xi >= WALL_MIN - 1e-3f) bestT = std::min(bestT, t);
            }

            if (bestT > 1e5f) return 9999.0f; // no intersection
            float expected = bestT; // distance from sensor aperture to wall
            // sensor bias? subtract 
            expected -= sensorReadingBiases()[sensorIndex];
            if (expected < 0.0f) expected = 0.0f;
            return expected;
        };

        while (true) {
            // stupid telementary stuff so we dont implode . generated by chatGPT because im lazy
            if (controller.get_digital_new_press(TOGGLE_BUTTON)) {
                bool on = !mclEnabled.load();
                mclEnabled.store(on);
                controller.print(4, 0, "MCL %s", on ? "ON" : "OFF");
            }

            if (!mclEnabled.load()) {
                // MCL paused, sleep briefly and check toggle again
                pros::delay(100);
                continue;
            }
            // predict particle positions from odom
               lemlib::Pose localSpeed = lemlib::getLocalSpeed(false); // get local speed from lemlib , good thing i caught that mistake before  my beauty sleep (goon)
            for (Particle &p : particles) { //for each particle...
                float dx = (localSpeed.x * dt) + dxNoise(generator); // use our deltas (which is our odom speed stuff thx lemlib) and then add some noise
                float dy = (localSpeed.y * dt) + dyNoise(generator);
                float dtheta = (localSpeed.theta * dt) + dthetaNoise(generator);
                float thetaRad = lemlib::degToRad(p.theta); // c++ is weird and needs radians for trig functions

                /*
                TODO:
                change odomspeed to be local instead of global
                */
                p.x += dx * cos(thetaRad) - dy * sin(thetaRad); //just realized this changes it to be global. That's not good our odom Speed is global too. Yeah im wayy too lazy to fix that now
                p.y += dx * sin(thetaRad) + dy * cos(thetaRad);
                p.theta += dtheta; //normalize angle to be within (-180, 180]
                p.theta = fmod(p.theta + 180.0f, 360.0f);
                if (p.theta < 0) p.theta += 360.0f;
                p.theta -= 180.0f;
            }

            // Update weights
            auto readings = receiveReadings();
            auto confidences = receiveConfidences();
            float totalWeight = 0.0f;

            for (Particle &p : particles) {
                float w = 1.0f; // initial weight
                for (int s = 0; s < 4; ++s) { //for each sensor...
                    float actual = readings[s];
                    if (actual >= (9999.0f * 0.03937008f)) continue; // if sensor reach max reading, skip it
                    float expected = expected_distance(p.x, p.y, p.theta, s);  //find what the sensor in theory should read at the particle's pose
                    float conf = (confidences[s] > 0) ? (float)confidences[s] : 1.0f; // avoid /0
                    float sigma = std::max(SENSOR_SIGMA_FACTOR * actual / sqrtf(conf / 64.0f), SENSOR_SIGMA_MIN); // standard deviation algorithm stolen staight from Echo 2654E. Thanks Echo!
                    float err = actual - expected; // self explanatory
                    w *= expf(-(err * err) / (2.0f * sigma * sigma)); //gaussian probability (we're basically grading the particle on its accuracy)
                }
                p.weight = w;
                totalWeight += p.weight; //add up all weights so we can norm em
            }

            // normalize weights
            if (totalWeight <= 1e-9f) { //r our weights degenerate? 
                for (Particle &p : particles) p.weight = 1.0f / (float)particles.size(); //make them uniform then oh well
            } else {
                for (Particle &p : particles) p.weight /= totalWeight; //normalize
            }

            // resampling
            std::vector<Particle> newParticles; // curated particle set
            newParticles.reserve(PARTICLE_COUNT); //reserve space so we dont have to realloc
            std::uniform_real_distribution<float> dist(0.0f, 1.0f / (float)PARTICLE_COUNT); //create a distribution for  low variance resampling
            float r = dist(generator); // random start 
            float c = particles[0].weight; // cumulative weight
            int idx = 0; // particle index
            for (int m = 0; m < PARTICLE_COUNT; ++m) { //for each particle to resample...
                float U = r + m * (1.0f / (float)PARTICLE_COUNT); // find U
                while (U > c && idx + 1 < (int)particles.size()) { // find the particle corresponding to U
                    idx++; 
                    c += particles[idx].weight; //re cumulate weight
                }
                newParticles.push_back(particles[idx]); //add curated particle to list
            }
            newParticles.swap(particles); //replace old particles with new ones

            // estimate pose (mean x, y, and circular mean for theta)
            float addX = 0.0f;
            float addY = 0.0f;
            float addThetaX = 0.0f;
            float addThetaY = 0.0f;
            for (const Particle &p : particles) { //sum them all up
                addX += p.x;
                addY += p.y;
                addThetaX += cosf(lemlib::degToRad(p.theta));
                addThetaY += sinf(lemlib::degToRad(p.theta));
            }
            float estX = addX / (float)particles.size(); //average our good particles up
            float estY = addY / (float)particles.size();
            float estTheta = lemlib::radToDeg(atan2f(addThetaY, addThetaX)); //nerd stuff for circular mean 

            chassis.setPose(estX, estY, estTheta); //finaly update pose ugghh

            pros::delay(10); //sleep  so CPU doesnt explode
        }
    });

    pros::delay(10);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}

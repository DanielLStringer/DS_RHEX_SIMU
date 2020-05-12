/*
DS's version of test
*/

#include <iostream>
#include <cstdlib>
#include <rhex_dart/DS_fixed_rhex_dart_simu.hpp>
#define CTRL_SIZE 28

struct Params {
    static constexpr double radius() { return 0.01; }

    static Eigen::Vector3d head() { return Eigen::Vector3d(1, 1, 0.5); }

    static Eigen::Vector3d tail() { return Eigen::Vector3d(1, 0, 0.5); }

    static Eigen::Vector4d color() { return Eigen::Vector4d(1, 0, 0, 1); }

    static std::string skel_name() { return "floor"; }

    static std::string body_name() { return "BodyNode"; }
};

int main(int argc, char** argv)
{
    // using the same model as the hexapod and so the robot has a damages parameter but is set to 0
    std::vector<rhex_dart::DS_RhexDamage> brk = std::vector<rhex_dart::DS_RhexDamage>();
    assert(argc == 4);

    // loads the robot with name Rhex tels it that it is not a URDF file and give it the blank damages
    // possible models: raised.skel, skinny.skel, Rhex8.skel, raised_loosehind.skel
    auto global_robot = std::make_shared<rhex_dart::DS_Rhex>(std::string(std::getenv("RESIBOTS_DIR")) + "/share/rhex_models/SKEL/" + argv[3], "Rhex", false, brk);


//////////// ./waf && ./build/test_damage 0 1 raised.skel 
	
	


    using desc_t = boost::fusion::vector<rhex_dart::descriptors::DutyCycle,
					rhex_dart::descriptors::Contact,
                    rhex_dart::descriptors::BodyOrientation,
                    rhex_dart::descriptors::SpecificResistance,
                    rhex_dart::descriptors::AvgCOMVelocities,
                    rhex_dart::descriptors::PositionTraj,
					rhex_dart::descriptors::RotationTraj>;

     // using safe_t = boost::fusion::vector<rhex_dart::safety_measures::BodyColliding, rhex_dart::safety_measures::MaxHeight, rhex_dart::safety_measures::TurnOver>;
     using viz_t = boost::fusion::vector<rhex_dart::visualizations::HeadingArrow, rhex_dart::visualizations::RobotTrajectory>;
     rhex_dart::DSFixedRhexDARTSimu<rhex_dart::desc<desc_t>, rhex_dart::viz<viz_t>> simu( global_robot, atof(argv[1]), atof(argv[2]));

#ifdef GRAPHIC
    simu.fixed_camera(Eigen::Vector3d(3, 0, 0.5));
    simu.follow_rhex();
#endif
    simu.run(30);

    std::cout << "Covered distance | Arrival angle | Body avg height" <<std::endl;
    std::cout << simu.covered_distance() << " " << simu.arrival_angle() << " " << simu.body_avg_height() << std::endl;

    std::cout << "Energy" << std::endl;
    std::cout << simu.energy() << std::endl;

    std::vector<double> v;
    simu.get_descriptor<rhex_dart::descriptors::DutyCycle>(v);
    std::cout << "Duty Cycle:" << std::endl;
    for (size_t i = 0; i < v.size(); i++) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> vv;
    simu.get_descriptor<rhex_dart::descriptors::BodyOrientation>(vv);
    std::cout << "Body Orientation" << std::endl;
    for (size_t i = 0; i < vv.size(); i++) {
        std::cout << vv[i] << " ";
    }
    std::cout << std::endl;

    double sr;
    simu.get_descriptor<rhex_dart::descriptors::SpecificResistance>(sr);
    std::cout << "Specific Resistance: " << sr << std::endl;

    Eigen::Vector3d vels;
    simu.get_descriptor<rhex_dart::descriptors::AvgCOMVelocities>(vels);

    std::cout << "Avg COM Velocities" << std::endl;
    for (size_t i = 0; i < vels.size(); i++) {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    global_robot.reset();
    return 0;
}

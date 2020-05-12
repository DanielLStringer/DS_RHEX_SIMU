#ifndef RHEX_DART_RHEX_CONTROL
#define RHEX_DART_RHEX_CONTROL

#include <rhex_dart/rhex.hpp>
#include <rhex_dart/pid_control.hpp>
#include <rhex_controller/rhex_controller_simple.hpp>
#include <rhex_controller/rhex_controller_cpg.hpp>
#define PI 3.14159265

// this file is a work in progress

namespace rhex_dart {

    class RhexControl {
    public:
        using robot_t = std::shared_ptr<Rhex>;

        RhexControl() {}

        RhexControl(const std::vector<double>& ctrl, robot_t robot)
            : _cpg(ctrl), _robot(robot),  _pid(2, 4, 0.01)
        {
            _feedback = std::vector<double>(6, 0.0);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();

            std::vector<double> start_phase(6,0);

            for (size_t i = 0; i < 6; ++i){
                start_phase[i] = current_positions[i+6];
            }

            _cpg.set_parameters(ctrl);
        }

        const std::vector<double> parameters() const
        {
            return _cpg.parameters();
        }

        robot_t robot()
        {
            return _robot;
        }

        void update(double t)
        {
            if (_robot == nullptr)
                return;

            _target_positions = _cpg.pos(t); // get phase from cpg, monotous value

            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();
            Eigen::VectorXd ctrl(54);

            ctrl << 0,0,0,0,0,0, - PI,PI/2,-PI,PI/2,-PI,PI/2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

            _robot->skeleton()->setPositions(ctrl);
            return;
            std::vector<double> feedback(6,0);

            for (size_t i = 0; i < 6; ++i){
                feedback[i] = current_positions[i+6];
            }

            // convert it
            for (size_t i = 0; i < 6; ++i){
                _target_positions[i] = _cpg.mono_transform(_target_positions[i]);
                feedback[i] = _cpg.mono_transform(feedback[i]);
            }

            std::cout << "Target/cpg/setpoint positions: " ;
            for (size_t i =0; i < 6; ++i){
                std::cout << _target_positions[i] << " ";
            }
            std::cout << std::endl;

            std::cout << "feedback/current positions: " ;
            for (size_t i =0; i < 6; ++i){
                std::cout << feedback[i] << " ";
            }
            std::cout << std::endl;

            _pid.set_points(_target_positions);
            _pid.update(feedback, t);

            std::vector<double> pid_output = _pid.get_output();

            Eigen::VectorXd commands = Eigen::VectorXd::Zero(54);

            for (size_t i = 0; i < 6; ++i){
                // std::cout << "Inputting to mono transform: " << pid_output[i] << std::endl;
                // commands[i+6] = _cpg.mono_transform(pid_output[i]); // transform the monotous input into the cycle
                if (pid_output[i] > 6)
                    commands[i+6] = 6;
                else if (pid_output[i] < -6)
                    commands[i+6] = -6;
                else
                    commands[i+6] = pid_output[i];

//                if (commands[i+6] < 0)
//                    commands[i+6] = commands[i+6] * -1;
            }

            std::cout << "Setting commands: ";
            for (size_t i = 0; i < 6; ++i){
                std::cout << commands[i + 6] << " ";
            }
            std::cout << std::endl;
            _robot->skeleton()->setCommands(commands);

//            std::cout << "Current leg positions are: " ;
//            for ( size_t i = 0; i < 6; ++i){
//                std::cout << _target_positions[i+6] << " ";
//            }
//            std::cout << std::endl;
            // it looks like you can give the joint any value and it will translate it into an appropriate range for itself to work with.
//            std::cout << "Current positions: ";
//            std::cout << current_positions << std::endl;

            // feedback to be put into PID
            // CPG phase is the new setpoint for PID

//            // setpoint PID from CPG
//            std::cout << "Setting PID set points/CPG target: " <<std::endl;
//            for ( size_t i = 0; i < _target_positions.size(); ++i){
//                std::cout << _target_positions[i] << " ";
//            }
//            std::cout << std::endl;


            // _pid.set_delta_time(t);
            // update pid with current joint positions
//            std::cout << "This is the current_positions array: ";
//            for (size_t i = 0; i < current_positions.size(); ++i){
//                std::cout <<  current_positions[i] << " ";
//            }
            //std::cout << std::endl;



//            // the pid controller updates its values with the new setpoint
//            std::cout << "Feedback is: " <<std::endl;
//            for ( size_t i = 0; i < feedback.size(); ++i){
//                std::cout << feedback[i] << " ";
//              }
//            std::cout << std::endl;



            // updates the robot with the new forces for the joints



//            std::cout << "PID output is: " <<std::endl;

//            for ( size_t i = 0; i < pid_output.size(); ++i){
//                std::cout << pid_output[i] << " ";
//            }
//            std::cout << std::endl;

            // for each value in pid_output, transform it accordingly using cpg function
            // first 6 are COM, the next six are hipjoint positions, the rest are
            // joints which are not real DOFs but require an input


//            std::cout << "Setting commands: ";
//            for (size_t i = 0; i < commands.size(); ++i){
//                std::cout << commands[i] << " ";
//                // commands[i] = _cpg.transform(commands[i] - PI/2);
//            }

         //   std::cout<< std::endl;

//            for (size_t i = 0; i < 6; ++i){
//                commands[i+6] = commands[i+6] - PI/2;
//            }

//            std::cout << std::endl;
//            Eigen::VectorXd ctrl(54);
//            ctrl << 0,0,0,0,0,0,50,50,50,50,50,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

//            for (size_t i = 0; i < 6; ++i){
//                positions[i+6] = 0;
//            }
//            std::cout << std::endl;

            // _robot->skeleton()->setPositions(commands);
            //_robot->skeleton()->setPositions(commands);
        }

    protected:
        rhex_controller::RhexControllerCPG _cpg;
        rhex_dart::PIDControl _pid;
        robot_t _robot;
        std::vector<double> _feedback;
        std::vector<double> _target_positions;


    };
}

#endif

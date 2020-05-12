#ifndef RHEX_DART_RHEX_CONTROL_HOPF
#define RHEX_DART_RHEX_CONTROL_HOPF

#include <algorithm>
#include <rhex_dart/rhex.hpp>
#include <rhex_dart/pid_control.hpp>
#include <rhex_controller/rhex_controller_hopf.hpp>
#define PI 3.14159265

#define PROP 5
#define INTEG 20
#define DIFF 0.0
#define CPG_SIZE 6
#define FORCE_LIMIT 3
#define IGNORE 1000

// this file is a work in progress


namespace rhex_dart {

    class RhexControlHopf {
    public:
        using robot_t = std::shared_ptr<Rhex>;

        RhexControlHopf() {}

        RhexControlHopf(const std::vector<double>& ctrl, robot_t robot)
            : _robot(robot)
        {
            set_parameters(ctrl);
            _leg_count = CPG_SIZE;

            _feedback = std::vector<double>(_leg_count, 0.0);
            _compensatory_count = std::vector<int>(_leg_count, 0);
        }


        RhexControlHopf(const std::vector<double>& ctrl, robot_t robot, std::vector<rhex_dart::RhexDamage> damages)
            : _robot(robot), _damages(damages)
        {
            set_parameters(ctrl);
            _leg_count = CPG_SIZE;

            // need to know which, and the no. of legs removed.
            int leg = 0;
            for (auto dmg : _damages) {
                if (dmg.type == "leg_removal") {
                    _leg_count -= 1;
                    _removed_legs.push_back(stoi(dmg.data));
                }

                leg+=1;
            }

            _feedback = std::vector<double>(_leg_count, 0.0);
            _compensatory_count = std::vector<int>(_leg_count, 0);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();

            _pid.clear();
            _pid.set_Kp(ctrl[0] * PROP);
            _pid.set_Ki(ctrl[1] * INTEG);
            _pid.set_Kd(DIFF);

            std::vector<double>::const_iterator first = ctrl.begin() + 3;
            std::vector<double>::const_iterator last = ctrl.begin() + ctrl.size();
            std::vector<double> cpg_ctrl(first, last);

            _cpg.set_parameters(cpg_ctrl);
        }

        const std::vector<std::vector<double> > parameters() const
        {
            return _cpg.parameters();
        }

        robot_t robot()
        {
            return _robot;
        }

        // legs on the body is aligned differently to that of the cpg, so correct
        void correct_legs(std::vector<double>& legs) {
            // switch legs 1 and 3
            double temp = legs[0];
            legs[0] = legs[2];
            legs[2] = temp;

            // switch legs 4 and 6
            temp = legs[3];
            legs[3] = legs[5];
            legs[5] = temp;
        }

        void update(double t)
        {
            std::cout<<"3"<<std::endl;

            if (_robot == nullptr)
                return;

            _target_positions = _cpg.pos(t);

            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();
//            std::cout<< "current positions: " ;
//            for (size_t i = 0; i < current_positions.size(); ++i){
//                std::cout << current_positions[i] << " ";
//            }
//            std::cout<<std::endl;

            std::vector<double> feedback(CPG_SIZE, 0);
            int skip_count = 0;
            for (size_t i = 0; i < CPG_SIZE; ++i)
            {
                // _removed_legs is sorted list so binary search
                if (std::binary_search(_removed_legs.begin(), _removed_legs.end(), i+1))
                {
                    feedback[i] = 0;
                    skip_count+=1;
                }

                else
                    feedback[i] = current_positions[i + 6 - skip_count];
            }

//            std::cout << "feedback before correction: " ;

//            for (size_t i = 0; i < CPG_SIZE; ++i){
//                std::cout << feedback[i] << " ";
//            }
//            std::cout<<std::endl;

            // correct_legs(feedback);

//            std::cout << "feedback after correction: " ;

//            for (size_t i = 0; i < CPG_SIZE; ++i){
//                std::cout << feedback[i] << " ";
//            }
//            std::cout << std::endl;

            // if the target is one or more full rotations ahead, subtract the appropriate amount of rotations.
            // this needs to be sustained for the rest of the simulation as the signal will never decrease.
            for (size_t i = 0; i < CPG_SIZE; ++i)
            {
                double diff = _target_positions[i] - 2 * PI * _compensatory_count[i] - feedback[i];
                
                while(diff >= 2*PI)
                {
                    _compensatory_count[i] += 1;
                    diff = _target_positions[i] - 2 * PI * _compensatory_count[i] - feedback[i];
                }
                
                _target_positions[i] -= 2 * PI * _compensatory_count[i];

                // similarly, if the feedback is more than 2 PI ahead of the signal, we dont want the leg to wait
                // which influences other legs in a negative way because of phase.

                diff = feedback[i] - _target_positions[i];
                
                if (diff > 2*(3*PI)/4)
                    _target_positions[i] +=  2 * PI;
            }

//            std::cout << "Target/cpg/setpoint positions: " ;
//            for (size_t i = 0; i < 6; ++i){
//                std::cout << _target_positions[i] << " ";
//            }
//            std::cout << std::endl;

//            std::cout << "feedback/current positions: " ;

//            for (size_t i = 0; i < CPG_SIZE; ++i){
//                std::cout << feedback[i] << " ";
//            }
//            std::cout << std::endl;

            _pid.set_points(_target_positions);
            _pid.update(feedback, t);

            std::vector<double> pid_output = _pid.get_output();

            std::vector<double> tcommands(6,0);
            //skip_count = 0;
            for (size_t i = 0; i < CPG_SIZE; ++i){
                if (!std::binary_search(_removed_legs.begin(), _removed_legs.end(), i))
                {
                    if (pid_output[i] > FORCE_LIMIT)
                        tcommands[i] = FORCE_LIMIT;
                    else if (pid_output[i] < -FORCE_LIMIT)
                        tcommands[i] = -FORCE_LIMIT;
                    else
                        tcommands[i] = pid_output[i];
                }

                else
                {
                    //skip_count += 1;
                    tcommands[i] = IGNORE;
                }
                std::cout<<"3"<<std::endl;

            }

            // correct_legs(tcommands);

            // remove ignored values to get the right number of dof commands for the present body
            int j = 0;
            size_t dof = _robot->skeleton()->getNumDofs();
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(dof);

            for(size_t i = 0; i < CPG_SIZE; ++i){
                if(tcommands[i] != IGNORE){
                    commands[j + 6] = tcommands[i];
                    ++j;
                }
            }

//            std::cout << "Setting commands: ";
//            for (size_t i = 0; i < _leg_count; ++i){
//                std::cout << commands[i + CPG_SIZE] << " ";
//            }
//            std::cout << std::endl;

            _robot->skeleton()->setCommands(commands);

        }

    protected:
        rhex_controller::RhexControllerHopf _cpg;
        rhex_dart::PIDControl _pid;
        robot_t _robot;
        std::vector<double> _feedback;
        std::vector<double> _target_positions;
        std::vector<int> _compensatory_count;
        std::vector<RhexDamage> _damages;
        std::vector<int> _removed_legs;
        int _leg_count;
    };
}

#endif

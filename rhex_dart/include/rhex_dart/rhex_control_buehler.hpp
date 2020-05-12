/*
Aran Smith's code
*/

#ifndef RHEX_DART_RHEX_CONTROL_BUEHLER
#define RHEX_DART_RHEX_CONTROL_BUEHLER

#include <string>
#include <fstream>
#include <streambuf>

#include <algorithm>
#include <rhex_dart/rhex.hpp>
#include <rhex_dart/pid_control.hpp>
#include <rhex_controller/rhex_controller_buehler.hpp>
#define PI 3.14159265

#define PROP 15         // proportional gain
#define INTEG 15        // integral gain
#define DIFF 0          // differential gain

#define CPG_SIZE 6      // no. outputs from the controller
#define FORCE_LIMIT 4   // +- torque limit
#define IGNORE 1000     // term used to ignore legs

namespace rhex_dart {

    class RhexControlBuehler {
    public:
        using robot_t = std::shared_ptr<Rhex>;

        RhexControlBuehler() {}

        RhexControlBuehler(const std::vector<double>& ctrl, robot_t robot, std::vector<rhex_dart::RhexDamage> damages = {})
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

            _feedback = std::vector<double>(CPG_SIZE, 0.0);
            _compensatory_count = std::vector<int>(CPG_SIZE, 0);
			//std::cout << "Rhex_control_buehler" << std::endl;
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();

            _pid.clear();
            _pid.set_Kp(PROP);
            _pid.set_Ki(INTEG);
            _pid.set_Kd(DIFF);

            std::vector<double>::const_iterator first = ctrl.begin();
            std::vector<double>::const_iterator last = ctrl.begin() + ctrl.size();
            std::vector<double> cpg_ctrl(first, last);

            _cpg.set_parameters(cpg_ctrl);
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

            _target_positions = _cpg.pos(t);
            
            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();

            std::vector<double> feedback(CPG_SIZE, 0);
            int skip_count = 0;
            for (size_t i = 0; i < CPG_SIZE; ++i)
            {
                // _removed_legs is sorted list so binary search
                if (std::binary_search(_removed_legs.begin(), _removed_legs.end(), i))
                {
                    feedback[i] = 0;
                    skip_count+=1;
                }

                else
                    feedback[i] = current_positions[i + 6 - skip_count];
            }

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
                diff = feedback[i] - _target_positions[i];
                if (diff > (3 * PI) / 2){
                    _target_positions[i] +=  2 * PI;
                }
            }

            _pid.set_points(_target_positions);
            _pid.update(feedback, t);

            std::vector<double> pid_output = _pid.get_output();

            std::vector<double> tcommands(6,0);
            for (size_t i = 0; i < CPG_SIZE; ++i)
            {
                // for those legs not removed, add the command
                if (!std::binary_search(_removed_legs.begin(), _removed_legs.end(), i))
                {
                    if (pid_output[i] > FORCE_LIMIT)
                        tcommands[i] = FORCE_LIMIT;
                    else if (pid_output[i] < -FORCE_LIMIT)
                        tcommands[i] = -FORCE_LIMIT;
                    else
                        tcommands[i] = pid_output[i];
                }

                // indicate removed legs with an ignore flag
                else
                {
                    tcommands[i] = IGNORE;
                }
            }

            // remove ignored values to get the right number of dof commands for the present body
            int j = 0;
            size_t dof = _robot->skeleton()->getNumDofs();
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(dof);

            for(size_t i = 0; i < CPG_SIZE; ++i)
            {
                if(tcommands[i] != IGNORE)
                {
                    commands[j + 6] = tcommands[i];
                    ++j;
                }
            }
			//std::cout<<"DOFs:   "<<dof<<std::endl;
			
			/*std::cout << "Rhex_control_buehler   commands:" <<std::endl;
			std::cout << "Commands 1:  " << commands[0] <<std::endl;
			std::cout << "Commands 2:  " << commands[1] <<std::endl;
			std::cout << "Commands 3:  " << commands[2] <<std::endl;
			std::cout << "Commands 4:  " << commands[3] <<std::endl;
			std::cout << "Commands 5:  " << commands[4] <<std::endl;
			std::cout << "Commands 6:  " << commands[5] <<std::endl;
			std::cout << "Commands 7:  " << commands[6] <<std::endl;
			std::cout << "Commands 8:  " << commands[7] <<std::endl;
			std::cout << "Commands 9:  " << commands[8] <<std::endl;
			std::cout << "Commands 10:  " << commands[9] <<std::endl;
			std::cout << "Commands 11:  " << commands[10] <<std::endl;
			std::cout << "Commands 12:  " << commands[11] <<std::endl;*/
            _robot->skeleton()->setCommands(commands);

        }

    protected:
        rhex_controller::RhexControllerBuehler _cpg;
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

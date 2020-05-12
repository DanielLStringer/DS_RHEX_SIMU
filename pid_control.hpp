/*
Aran Smith's code
*/

#ifndef RHEX_DART_PID_CONTROL
#define RHEX_DART_PID_CONTROL

#define PI 3.14159265

namespace rhex_dart {

    class PIDControl {
    public:

        PIDControl() {}

        PIDControl(double p, double i, double d)
        {
            _Kp = p;
            _Ki = i;
            _Kd = d;

            clear();
        }

        void clear() // Clears PID computations and coefficients
        {
            _set_point = std::vector<double>(6, 0.0);

            _Pterm =  std::vector<double>(6, 0.0);
            _Iterm =  std::vector<double>(6, 0.0);
            _Dterm =  std::vector<double>(6, 0.0);
            _last_error = std::vector<double>(6, 0.0);

            _windup_guard = PI;

            _output = std::vector<double>(6, 0.0);
            _last_time = 0.0;
        }

        void update(const std::vector<double>& feedback_values, double time)
        {
            // Calculates PID value for given reference feedback
            // u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
            std::vector<double> error(6, 0);
            std::vector<double> delta_error(6, 0);

            _delta_time = time - _last_time;
            if (_delta_time < 0)
                _delta_time = 0; // cant be less than zero, happens at start of sim.
            _last_time = time;

            for(size_t i = 0; i < 6; ++i)
            {

                error[i] = _set_point[i] - feedback_values[i];

                delta_error[i] = error[i] - _last_error[i];

                _Pterm[i] = _Kp * error[i];

                _Iterm[i] += error[i] * _delta_time;

                if (_Iterm[i] < -_windup_guard)
                {
                    //std::cout<< "hit the negative iterm windup guard" << std::endl;
                    _Iterm[i] = -_windup_guard;
                }
                else if (_Iterm[i] > _windup_guard)
                {
                    //std::cout<< "hit the positive iterm windup guard" << std::endl;
                    _Iterm[i] = _windup_guard;
                }

                _Dterm[i] = 0.0;
                if (_delta_time > 0)
                    _Dterm[i] = delta_error[i] / _delta_time;

                // Remember last error for next calculation
                _last_error[i] = error[i];
                // std::cout << str(_Pterm) + " " + str(_Iterm) + " " + str(_Dterm) << std::endl;

                _output[i] = _Pterm[i] + (_Ki * _Iterm[i]) + (_Kd * _Dterm[i]);

            }
        }

        void set_points(const std::vector<double>& targets)
        {
            _set_point = targets;
        }

        void set_Kp(double proportional_gain)
        {
            // Determines how aggressively the PID reacts to the current error with setting Proportional Gain
            _Kp = proportional_gain;
        }

        void set_Ki(double integral_gain)
        {
            // Determines how aggressively the PID reacts to the current error with setting Integral Gain
            _Ki = integral_gain;
        }

        void set_Kd(double derivative_gain)
        {
            // Determines how aggressively the PID reacts to the current error with setting Derivative Gain
            _Kd = derivative_gain;
        }

        void set_windup(double windup)
        {
            _windup_guard = windup;
        }

        void set_delta_time(double dt){
            _delta_time = dt;
        }

        const std::vector<double>& get_output()
        {
            return _output;
        }

    protected:
        double _Kp;
        double _Ki;
        double _Kd;
        double _delta_time;
        double _last_time;

        std::vector<double> _set_point;
        std::vector<double> _Pterm;
        std::vector<double> _Iterm;
        std::vector<double> _Dterm;
        std::vector<double> _last_error;

        double _windup_guard = PI;

        std::vector<double> _output;
    };
}


#endif // PID_CONTROL


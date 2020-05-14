/*

DS Rhex dart simu, my version of the code
FIXED so no Adaption
*/

#ifndef DS_FIXED_RHEX_DART_SIMU_HPP
#define DS_FIXED_RHEX_DART_SIMU_HPP

#include <boost/parameter.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/find.hpp>

#include <dart/dart.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/math/Constants.hpp>

#include <Eigen/Core>
//#include <rhex_dart/rhex.hpp>
#include <rhex_dart/DS_rhex.hpp>
#include <rhex_dart/safety_measures.hpp>
#include <rhex_dart/descriptors.hpp>
#include <rhex_dart/visualizations.hpp>
#include <rhex_dart/osc_control.hpp>
#include <rhex_dart/Adapt.hpp>
#include <cmath>
#include <cstdlib>

#include <string>
#include <fstream>
#include <streambuf>

#ifdef GRAPHIC
#include <dart/gui/osg/osg.hpp>
#endif


namespace rhex_dart {

    BOOST_PARAMETER_TEMPLATE_KEYWORD(rhex_control)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(safety)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(desc)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(viz)

    typedef boost::parameter::parameters<boost::parameter::optional<tag::rhex_control>,
        boost::parameter::optional<tag::safety>,
        boost::parameter::optional<tag::desc>,
        boost::parameter::optional<tag::viz>> class_signature;

    template <typename DS_FIXED_Simu, typename robot>
    struct Refresh {
        Refresh(DS_FIXED_Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            : _simu(simu), _robot(rob), _init_trans(init_trans) {}

        DS_FIXED_Simu& _simu;
        std::shared_ptr<robot> _robot;
        Eigen::Vector6d _init_trans;

        template <typename T>
        void operator()(T& x) const { x(_simu, _robot, _init_trans); }
    };

    template <class A1 = boost::parameter::void_, class A2 = boost::parameter::void_, class A3 = boost::parameter::void_, class A4 = boost::parameter::void_>
    class DSFixedRhexDARTSimu {
    public:
		
        using robot_t = std::shared_ptr<DS_Rhex>;
        // defaults
        struct defaults {
            //using safety_measures_t = boost::fusion::vector<safety_measures::MaxHeight, safety_measures::BodyColliding, safety_measures::TurnOver>;
            using safety_measures_t = boost::fusion::vector<safety_measures::BodyColliding, safety_measures::TurnOver>;
			//using safety_measures_t = boost::fusion::vector<safety_measures::TurnOver>;
            using descriptors_t = boost::fusion::vector<descriptors::DutyCycle, descriptors::Contact, descriptors::SpecificResistance, descriptors::AvgCOMVelocities>;
            using viz_t = boost::fusion::vector<visualizations::HeadingArrow>;
        };

        // extract the types
        using args = typename class_signature::bind<A1, A2, A3, A4>::type;
        using SafetyMeasures = typename boost::parameter::binding<args, tag::safety, typename defaults::safety_measures_t>::type;
        using Descriptors = typename boost::parameter::binding<args, tag::desc, typename defaults::descriptors_t>::type;
        using Visualizations = typename boost::parameter::binding<args, tag::viz, typename defaults::viz_t>::type;
        using safety_measures_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<SafetyMeasures>, SafetyMeasures, boost::fusion::vector<SafetyMeasures>>::type;
        using descriptors_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Descriptors>, Descriptors, boost::fusion::vector<Descriptors>>::type;
        using viz_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Visualizations>, Visualizations, boost::fusion::vector<Visualizations>>::type;

        DSFixedRhexDARTSimu( robot_t robot, int world_option = 0, double friction = 1.0, std::vector<rhex_dart::DS_RhexDamage> damages = {}) : _covered_distance(0.0),
                                                                          _energy(0.0),
                                                                          _world(std::make_shared<dart::simulation::World>()),                                                                         
                                                                          _world_option(world_option),
                                                                          _old_index(0),
                                                                          _desc_period(2),
                                                                          _break(false)
        {
			_world->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
			//_world->setGravity(Eigen::Vector3d(0.0, 9.81, 0.0));
            _robot = robot;
			std::cout<<"DS_FIXED_RHEX_DART_SIMU_HPP"<<std::endl;
            // set position of rhex
            _robot->skeleton()->setPosition(6, 0.1);
            switch(_world_option) {
                case 0: // just a flat world
                    _add_floor(friction);
                    break;
                case 1: // flat world with a round hill, make sure to turn off height safety measure!
                    _add_hill(friction);
                    break;
                case 2:
                    _add_stairs();
                    break;
                case 3:
                    _add_slope(friction);
                    break;
                case 4:
                    _add_rugged();
                    break;
                case 5:
                    _add_rugged_ditch();
                    break;
                case 6:
                    _add_pipes();
                    break;
                case 7:
                    _add_ditch();
                    break;
                case 8:
                    _add_thick_pipe();
                    break;
                case 9:
                    _add_thin_pipe();
                    break;
				case 10:
                    _add_later_hill();
                    break;
				case 11:
                    _add_thin_pipe();
					_add_later_hill();
                    break;
				case 12:
                    _add_rugged();
                    _add_stairs();
                    break;
				case 13:
					_add_slope();
					_add_three_pipes();
					break;
            }
			

            _world->addSkeleton(_robot->skeleton());
            _world->setTimeStep(0.0025);


            _world->setTime(0.0);

#ifdef GRAPHIC
            _fixed_camera = false;
            _osg_world_node = new dart::gui::osg::WorldNode(_world);
            _osg_viewer.addWorldNode(_osg_world_node);
            _osg_viewer.setUpViewInWindow(0, 0, 640, 480);

            // full-screen
            // _osg_viewer.setUpViewOnSingleScreen();
#endif
        }

        ~DSFixedRhexDARTSimu() {}

        void run(double duration = 15.0, bool continuous = false, bool chain = false)
        {
            _break = false;
            robot_t rob = this->robot();
            double old_t = _world->getTime();
			double Pi= 3.1415926535;
			
			std::vector <double> phases(6,0);
			double t_switch=200;
			//double t_switch=0.2;
			
			std::vector<double> Ep_phases(7,0);
			
			//Tripod gait
			double Ep=0.5;
			phases[0]=-1*Pi;			
			phases[1]=0;
			phases[2]=1*Pi;
			phases[3]=0;
			phases[4]=-1*Pi;
			phases[5]=1*Pi;
			
			
			//Quadruped gait
			/*double Ep=2.0/3.0;	
			phases[0]=(-2.0/3.0)*Pi;				
			phases[1]=(-2.0/3.0)*Pi;						
			phases[2]=(4.0/3.0)*Pi;
			phases[3]=(-2.0/3.0)*Pi;
			phases[4]=(-2.0/3.0)*Pi;
			phases[5]=(4.0/3.0)*Pi;*/
			
			//Hill climbing (AS)
			/*double Ep=0.5;
			phases[0]=Pi;				
			phases[1]=-1*Pi;						
			phases[2]=Pi;
			phases[3]=-1*Pi;
			phases[4]=Pi;
			phases[5]=-1*Pi;*/
			
			//Stair climbing (AS)
			/*double Ep=0.5;	
			phases[0]=0;				
			phases[1]=(-2.0/3.0)*Pi;						
			phases[2]=0;
			phases[3]=(-2.0/3.0)*Pi;
			phases[4]=0;
			phases[5]=(4.0/3.0)*Pi;*/
			
			//Wave gait
			//double Ep=0.833333;
			/*double Ep=5.0/6.0;
			phases[0]=-1*Pi;
			phases[1]=(-1.0/3.0)*Pi;
			phases[2]=1*Pi;
			phases[3]=(-1.0/3.0)*Pi;
			phases[4]=-1*Pi;
			phases[5]=(5.0/3.0)*Pi;*/
			
            size_t index = _old_index;
            Eigen::Vector3d init_pos = rob->pos();
			
			rhex_dart::OscControl _controller(_robot);
			double move_camera;
			move_camera=0;
			double old_COM;
			auto COM = rob->skeleton()->getCOM();
			old_COM=COM(0);
			

			//declaration of sensor variables
			std::vector<long unsigned int> contacts;
			Eigen::Vector3d now_pos;
			Eigen::Vector3d now_rot;
			
            static Eigen::Vector6d init_trans = rob->pose();

#ifdef GRAPHIC
            while ((_world->getTime() - old_t) < duration && !_osg_viewer.done())
#else
            while ((_world->getTime() - old_t) < duration)
#endif
            {
				
				if((_world->getTime() - old_t)>t_switch){
					Ep_phases =adpt.InToOut(contacts, now_rot);				
					Ep=Ep_phases[0];
					phases[0]=Ep_phases[1];
					phases[1]=Ep_phases[2];				
					phases[2]=Ep_phases[3];
					phases[3]=Ep_phases[4];				
					phases[4]=Ep_phases[5];
					phases[5]=Ep_phases[6];						
				}
				
				/*if((_world->getTime() - old_t)>t_switch){
					double Ep=5.0/6.0;
					phases[0]=-1*Pi;
					phases[1]=(-1.0/3.0)*Pi;
					phases[2]=1*Pi;
					phases[3]=(-1.0/3.0)*Pi;
					phases[4]=-1*Pi;
					phases[5]=(5.0/3.0)*Pi;				
				}
				
				if((_world->getTime() - old_t)>2*t_switch){
					double Ep=2.0/3.0;
					phases[0]=(-2.0/3.0)*Pi;				
					phases[1]=(-2.0/3.0)*Pi;							
					phases[2]=(4.0/3.0)*Pi;
					phases[3]=(-2.0/3.0)*Pi;
					phases[4]=(-2.0/3.0)*Pi;
					phases[5]=(4.0/3.0)*Pi;					
				}	

				if((_world->getTime() - old_t)>2.1*t_switch){
					double Ep=0.5;	
					phases[0]=0;				
					phases[1]=(-2.0/3.0)*Pi;						
					phases[2]=0;
					phases[3]=(-2.0/3.0)*Pi;
					phases[4]=0;
					phases[5]=(4.0/3.0)*Pi;				
				}	*/
							
				//std::cout<<Ep<<std::endl;			
				//std::cout<<phases[0]<<phases[1]<<phases[2]<<phases[3]<<phases[4]<<phases[5]<<std::endl;
                _controller.update(_world->getTime(), phases, Ep);
				
							
                _world->step(false);

                // integrate Torque (force) over time
                Eigen::VectorXd state = rob->skeleton()->getForces().array().abs() * _world->getTimeStep();
                _energy += state.sum();

                // update safety measures
                if (_world->getTime() >= 2){
                    boost::fusion::for_each(_safety_measures, Refresh<DSFixedRhexDARTSimu, DS_Rhex>(*this, rob, init_trans));
                // update visualizations
                boost::fusion::for_each(_visualizations, Refresh<DSFixedRhexDARTSimu, DS_Rhex>(*this, rob, init_trans));
				}

                if (_break) {
                    //_covered_distance = -10002.0;
                    _arrival_angle = -10002.0;
                    _energy = -10002.0;
                    return;
                }

#ifdef GRAPHIC
                if (!_fixed_camera) {
                    auto COM = rob->skeleton()->getCOM();
                    _osg_viewer.getCameraManipulator()->setHomePosition(
                        osg::Vec3d((-1+move_camera), -3, 2), osg::Vec3d(COM(0), COM(1), COM(2)), osg::Vec3d(0, 0, 1));
                    _osg_viewer.home();
					//move_camera=move_camera+0.001;
					move_camera=old_COM-COM(0);
					old_COM=COM(0);
                }
                // process next frame
                _osg_viewer.frame();
#endif

                if (index % _desc_period == 0) {
                    // update descriptors
                    boost::fusion::for_each(_descriptors, Refresh<DSFixedRhexDARTSimu, DS_Rhex>(*this, rob, init_trans));
                }
				
				
				//which legs are currently in contact with ground is the last 6 elements in contacts
				//rest of contacts is memory of when each leg was in contact with ground
				  get_descriptor<rhex_dart::descriptors::Contact>(contacts);
				/*std::cout << "CONTACTS:" << std::endl;	
			    for (size_t i = (contacts.size()-6); i < contacts.size(); i++) {
					std::cout << contacts[i] << " ";
				}
				std::cout << std::endl;*/
	
				//Current position of the robot: now_pos[0] is x(forward); now_pos[1] is y(sideways); now_pos[2] is z(height)  	
				now_pos = rob->pos();
			   // std::cout<<"pos[0]: "<<now_pos[0]<<"  "<<"pos[1]: "<<now_pos[1]<<"  "<<"pos[2]: "<<now_pos[2]<<"  "<<std::endl;
				
				//Current rotation of the robot: now_rot[0] is roll; now_rot[1] is pitch; now_rot[2] is yaw (all ideally 0)
				now_rot = rob->rot();
				//std::cout<<"rot[0]: "<<now_rot[0]<<"  "<<"rot[1]: "<<now_rot[1]<<"  "<<"rot[2]: "<<now_rot[2]<<"  "<<std::endl;

				


                ++index;
                _body_avg_height += rob->pos()[2];
            }
				
				
            _old_index = index;

            if (!continuous) {
                if (!_stabilize_robot()) {
                   // _covered_distance = -10002.0;
                    _arrival_angle = -10002.0;
                    _energy = -10002.0;
                    return;
                }
            }
            Eigen::Vector3d fin_pos = rob->pos();

            // updates values of covered distance average body height and arrival angle
            _covered_distance = fin_pos[0]-init_pos[0];
            _body_avg_height = (chain)?_body_avg_height/(_world->getTime()-old_t):_world->getTime();
            _arrival_angle = std::round(atan((fin_pos[1]-init_pos[1])/(_covered_distance)) * 100) / 100.0;
			
			//Gives average velocities in x, y and z  (all the velocities divided by the amount of times it is tested)
			Eigen::Vector3d vels;
			get_descriptor<rhex_dart::descriptors::AvgCOMVelocities>(vels);
			/*std::cout << "Avg COM Velocities" << std::endl;
			for (size_t i = 0; i < vels.size(); i++) {
					std::cout << vels[i] << " ";
			}
			std::cout << std::endl;*/
			//Alternatively could do _covered_distance (found later on) and compare these,
			//as will run for same amount of time, so furthest is best. (just looks at x distance)
			
			adpt.fitness(_covered_distance,_world->getTime() ,vels);
			
        }

        robot_t robot()
        {
            return _robot;
        }

        dart::simulation::WorldPtr world()
        {
            return _world;
        }

#ifdef GRAPHIC
        void fixed_camera(const Eigen::Vector3d& camera_pos, const Eigen::Vector3d& look_at = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d& up = Eigen::Vector3d(0, 0, 1))
        {
            _fixed_camera = true;
            _camera_pos = camera_pos;
            _look_at = look_at;
            _camera_up = up;
	
            // set camera position
            _osg_viewer.getCameraManipulator()->setHomePosition(
                osg::Vec3d(_camera_pos(0), _camera_pos(1), _camera_pos(2)), osg::Vec3d(_look_at(0), _look_at(1), _look_at(2)), osg::Vec3d(_camera_up(0), _camera_up(1), _camera_up(2)));
            _osg_viewer.home();
        }

        void follow_rhex()
        {
            _fixed_camera = false;
        }
#endif

        template <typename Desc, typename T>
        void get_descriptor(T& result)
        {
            auto d = boost::fusion::find<Desc>(_descriptors);
            (*d).get(result);
        }

        double covered_distance() const
        {
            return _covered_distance;
        }

        double body_avg_height() const
        {
            return _body_avg_height;
        }

        double energy() const
        {
            return _energy;
        }

        double arrival_angle() const
        {
            return _arrival_angle;
        }

        const Eigen::Vector3d& final_pos() const
        {
            return _final_pos;
        }

        const Eigen::Vector3d& final_rot() const
        {
            return _final_rot;
        }

        double step() const
        {
            assert(_world != nullptr);
            return _world->getTimeStep();
        }

        void set_step(double step)
        {
            assert(_world != nullptr);
            _world->setTimeStep(step);
        }

        size_t desc_dump() const
        {
            return _desc_period;
        }

        void set_desc_dump(size_t desc_dump)
        {
            _desc_period = desc_dump;
        }

        void stop_sim(bool disable = true)
        {
            _break = disable;
        }
		
		void collision()
		{
		adpt.fail_state_colllision();	
		}

        // pose: Orientation-Position, dims: XYZ
        void add_box(const Eigen::Vector6d& pose, const Eigen::Vector3d& dims, std::string type = "free", double mass = 1.0, const Eigen::Vector4d& color = dart::Color::Red(1.0), const std::string& box_name = "box")
        {
            std::string name = box_name;
            // We do not want boxes with the same names!
            while (_world->getSkeleton(name) != nullptr) {
                if (name[name.size() - 2] == '_') {
                    int i = name.back() - '0';
                    i++;
                    name.pop_back();
                    name = name + std::to_string(i);
                }
                else {
                    name = name + "_1";
                }
            }

            dart::dynamics::SkeletonPtr box_skel = dart::dynamics::Skeleton::create(name);

            // Give the box a body
            dart::dynamics::BodyNodePtr body;
            if (type == "free")
                body = box_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;
            else
                body = box_skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            body->setMass(mass);
            body->setName(name);

            // Give the body a shape
            auto box = std::make_shared<dart::dynamics::BoxShape>(dims);
            auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
            box_node->getVisualAspect()->setColor(color);

            // Put the body into position
            if (type == "free") // free floating
                box_skel->setPositions(pose);
            else // fixed
                body->getParentJoint()->setTransformFromParentBodyNode(dart::math::expMap(pose));

            _world->addSkeleton(box_skel);
            _objects.push_back(box_skel);
        }

        // pose: Orientation-Position, dims: XYZ
        void add_ellipsoid(const Eigen::Vector6d& pose, const Eigen::Vector3d& dims, std::string type = "free", double mass = 1.0, const Eigen::Vector4d& color = dart::Color::Red(1.0), const std::string& ellipsoid_name = "sphere")
        {
            std::string name = _get_unique(ellipsoid_name);

            dart::dynamics::SkeletonPtr ellipsoid_skel = dart::dynamics::Skeleton::create(name);

            // Give the ellipsoid a body
            dart::dynamics::BodyNodePtr body;
            if (type == "free")
                body = ellipsoid_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;
            else
                body = ellipsoid_skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            body->setMass(mass);
            body->setName(name);

            // Give the body a shape
            auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(dims);
            auto ellipsoid_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(ellipsoid);
            ellipsoid_node->getVisualAspect()->setColor(color);

            // Put the body into position
            if (type == "free") // free floating
                ellipsoid_skel->setPositions(pose);
            else // fixed
                body->getParentJoint()->setTransformFromParentBodyNode(dart::math::expMap(pose));

            _world->addSkeleton(ellipsoid_skel);
            _objects.push_back(ellipsoid_skel);
        }

        void clear_objects()
        {
            for (auto obj : _objects) {
                _world->removeSkeleton(obj);
            }
            _objects.clear();
        }

    protected:
        bool _stabilize_robot(bool update_ctrl = false)
        {
            robot_t rob = this->robot();
            bool stabilized = false;
            int stab = 0;
            if (update_ctrl)
                _world->setTimeStep(0.001);

            for (size_t s = 0; s < 1000 && !stabilized; ++s) {
                Eigen::Vector6d prev_pose = rob->pose();

                /*if (update_ctrl){
                    _controller.update(_world->getTime());
                }*/
                _world->step();
                if ((rob->pose() - prev_pose).norm() < 1e-4)
                    stab++;
                else
                    stab = 0;
                if (stab > 30)
                    stabilized = true;
            }
            if (update_ctrl)
                _world->setTimeStep(0.015);
            return stabilized;
        }

        void _add_floor(double friction = 1.0, double width = 20.0, double length = 20.0)
        {
            // We do not want 2 floors!
            if (_world->getSkeleton("floor") != nullptr)
                return;

            dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");

            // Give the floor a body
            dart::dynamics::BodyNodePtr fbody = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            fbody->setFrictionCoeff(friction);

            // Give the body a shape
            double floor_height = 0.2;

            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(width, length, floor_height));

            auto box_node = fbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Gray());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
            fbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(floor);

        }

        void _add_slope(double friction = 1.0)
        {
            // We do not want 2 floors!
            if (_world->getSkeleton("slope") != nullptr)
                return;
            _add_floor(1, 12, 3);
            dart::dynamics::SkeletonPtr slope = dart::dynamics::Skeleton::create("slope");

            // Give the slope a body
            dart::dynamics::BodyNodePtr sbody = slope->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            sbody->setFrictionCoeff(friction);

            // Give the body a shape
            double slope_width = 5.0;
            double slope_height = 0.1;

            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(slope_width*5, slope_width, slope_height));

            auto box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Blue());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(3.3, 0.0, 0);

            tf.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(-0.523599,  Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

            sbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(slope);

        }

        void _add_rugged()
        {
            _add_floor(1, 20, 2);

            srand(5);

            for (size_t i = 0; i < 150; ++i){
                double x = ((double) rand() / RAND_MAX); // 0 - 1 for x pos
                double y = ((double) rand() / RAND_MAX) * 1.5 - 1; // -1 - 1 for y pos

                double a = ((double) rand() / RAND_MAX); // 0 - 1 for dimensions
                // double b = ((double) rand() / RAND_MAX); // 0 - 1 for dimensions
                // double c = ((double) rand() / RAND_MAX); // 0 - 1 for dimensions

                Eigen::Vector6d pose;
                pose << 0, 0, 0, (1.5 * x)+1, 1 * y, 0;
                Eigen::Vector3d dims;
                dims << 0.2 * a + 0.05, 0.2 * a + 0.05, 0.2 * a + 0.05;
                std::string type = "";

                add_ellipsoid(pose, dims, type);
            }
        }

        void _add_hill(double friction = 1.0)
        {
            if (_world->getSkeleton("hill") != nullptr)
                return;
            _add_floor(1, 20, 5);
            dart::dynamics::SkeletonPtr hill = dart::dynamics::Skeleton::create("hill");

            // give the hill a body
            dart::dynamics::BodyNodePtr hbody = hill->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            hbody->setFrictionCoeff(friction);

            // Give the body a shape
            double hill_radius = 5.0;
            auto sphere = std::make_shared<dart::dynamics::SphereShape>(hill_radius);

            auto sphere_node = hbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(sphere);

            sphere_node->getVisualAspect()->setColor(dart::Color::Blue());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
           // tf.translation() = Eigen::Vector3d(3.2, 0.0, -4.0);
			tf.translation() = Eigen::Vector3d(3.2, 0.0, -4.5);
            hbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(hill);
        }
		
		void _add_later_hill(double friction = 1.0)
        {
            if (_world->getSkeleton("later_hill") != nullptr)
                return;
            _add_floor(1, 20, 5);
            dart::dynamics::SkeletonPtr later_hill = dart::dynamics::Skeleton::create("later_hill");

            // give the hill a body
            dart::dynamics::BodyNodePtr hbody = later_hill->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            hbody->setFrictionCoeff(friction);

            // Give the body a shape
            double hill_radius = 5.0;
            auto sphere = std::make_shared<dart::dynamics::SphereShape>(hill_radius);

            auto sphere_node = hbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(sphere);

            sphere_node->getVisualAspect()->setColor(dart::Color::Blue());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
           // tf.translation() = Eigen::Vector3d(3.2, 0.0, -4.0);
			tf.translation() = Eigen::Vector3d(4.0, 0.3, -4.3);
            hbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(later_hill);
        }

        void _add_stairs(double friction = 1.0)
        {
            _add_floor(1, 20, 5);

            // Give the body a shape
            double step_x_width = 0.2;
            double step_y_width = 5;
            double step_height = 0.18;

            for (size_t i = 0; i < 100; ++i)
            {
                std::string name = _get_unique("step");
                dart::dynamics::SkeletonPtr step = dart::dynamics::Skeleton::create(name);

                // Give the floor a body
                dart::dynamics::BodyNodePtr sbody = step->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                sbody->setFrictionCoeff(friction);
                sbody->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(step_x_width, step_y_width, step_height));

                auto box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(2.7 + (i*step_x_width), 0.0, i * (step_height) - step_height/2);
                sbody->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(step);
            }
        }

        void _add_rugged_ditch(double friction = 1.0)
        {
            _add_floor(1, 1, 5);

            // much like _add_stairs, except consisting of a downwards set of steps and then an upward set of steps
            double step_x_width = 0.2;
            double step_y_width = 5;
            double step_height = 0.18;
            double step_count = 5;

            // descending steps
            for (size_t i = 0; i < step_count; ++i)
            {
                std::string name = _get_unique("step");
                dart::dynamics::SkeletonPtr step = dart::dynamics::Skeleton::create(name);

                // Give the floor a body
                dart::dynamics::BodyNodePtr sbody = step->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                sbody->setFrictionCoeff(friction);
                sbody->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(step_x_width, step_y_width, step_height));

                auto box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(0.4 + (i*step_x_width), 0.0, - step_height/2 + -1 * (i * step_height));
                sbody->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(step);
            }

            // ascending steps
            for (size_t i = step_count; i < step_count * 2; ++i)
            {
                std::string name = _get_unique("step");
                dart::dynamics::SkeletonPtr step = dart::dynamics::Skeleton::create(name);

                // Give the floor a body
                dart::dynamics::BodyNodePtr sbody = step->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                sbody->setFrictionCoeff(friction);
                sbody->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(step_x_width, step_y_width, step_height));

                auto box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(0.4 + (i * step_x_width), 0.0, i * (step_height) - step_height/2 + -2 * (step_count * step_height));
                sbody->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(step);
            }

            // straight platform for the end
            dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor1");

            // Give the floor a body
            dart::dynamics::BodyNodePtr fbody = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            fbody->setFrictionCoeff(friction);

            // Give the body a shape
            double floor_height = 0.2;
            double width = 20.0;
            double length = 20.0;

            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(width, length, floor_height));

            auto box_node = fbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Gray());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(12.3, 0.0, -floor_height / 2.0);
            fbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(floor);

        }

        void _add_ditch(double friction = 1.0)
        {
            _add_floor(1, 0.6, 5);

            dart::dynamics::SkeletonPtr slope = dart::dynamics::Skeleton::create("slope");

            // Give the slope a body
            dart::dynamics::BodyNodePtr sbody = slope->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            sbody->setFrictionCoeff(friction);

            // Give the body a shape
            double slope_width = 5.0;
            double slope_height = 0.1;

            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(slope_width/2, slope_width - 0.1, slope_height));

            auto box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Blue());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(1.4, 0, -0.33);

            tf.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(0.261799,  Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

            sbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(slope);

            //floor
            dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");

            // Give the floor a body
            dart::dynamics::BodyNodePtr fbody = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            fbody->setFrictionCoeff(friction);

            // Give the body a shape
            double floor_height = 0.2;

            box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(5, 5, floor_height));

            box_node = fbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Gray());

            // Put the body into position
            Eigen::Isometry3d tf_floor(Eigen::Isometry3d::Identity());
            tf_floor.translation() = Eigen::Vector3d(5, 0.0, -0.647);
            fbody->getParentJoint()->setTransformFromParentBodyNode(tf_floor);

            _world->addSkeleton(floor);

            // incline
            slope = dart::dynamics::Skeleton::create("slope1");
            sbody = slope->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            sbody->setFrictionCoeff(friction);

            box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(slope_width, slope_width, slope_height));

            box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Blue());

            // Put the body into position
            Eigen::Isometry3d tf_incline(Eigen::Isometry3d::Identity());
            tf_incline.translation() = Eigen::Vector3d(1.41 + 2.5, 0, -0.647);

            tf_incline.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(-0.261799,  Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

            sbody->getParentJoint()->setTransformFromParentBodyNode(tf_incline);

            _world->addSkeleton(slope);

            floor = dart::dynamics::Skeleton::create("floor1");

            // Give the floor a body
            fbody = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            fbody->setFrictionCoeff(friction);

            box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(5, 5, floor_height));

            box_node = fbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Gray());

            // Put the body into position
            Eigen::Isometry3d tf_floor_end(Eigen::Isometry3d::Identity());
            tf_floor_end.translation() = Eigen::Vector3d(8.8, 0.0, -0.1);
            fbody->getParentJoint()->setTransformFromParentBodyNode(tf_floor_end);

            _world->addSkeleton(floor);

        }

        // dart collisions dont support box - cylinder collisions, I use the next closest shape, long thin boxes
        // to mimic pipes/sticks
        void _add_pipes(double friction = 1.0)
        {
            // create cylinders without the same name
            double pipe_x_width = 0.05;
            double pipe_y_width = 10;
            double pipe_z_width = 0.05;
            int num_pipes = 10;
            double pipe_y_rotation = 1.57079632679 / 2; // 45 degrees rototation
            double pipe_x_spacing = 1;

            _add_floor(1, 20, 10);

            for (size_t i = 0; i < num_pipes; ++i)
            {
                std::string name = _get_unique("pipe");
                dart::dynamics::SkeletonPtr pipe = dart::dynamics::Skeleton::create(name);

                // Give the pipe a body
                dart::dynamics::BodyNodePtr body = pipe->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                body->setFrictionCoeff(friction);
                body->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(pipe_x_width, pipe_y_width, pipe_z_width));

                auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(0.4 + (i*pipe_x_spacing), 0.0, 0.15);

                tf.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(pipe_y_rotation,  Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

                body->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(pipe);
            }

        }
		
		void _add_three_pipes(double friction = 1.0)
        {
            // create cylinders without the same name
            double pipe_x_width = 0.05;
            double pipe_y_width = 10;
            double pipe_z_width = 0.05;
            int num_pipes = 3;
            double pipe_y_rotation = 1.57079632679 / 2; // 45 degrees rototation
            double pipe_x_spacing = 1;

            _add_floor(1, 20, 10);

            for (size_t i = 0; i < num_pipes; ++i)
            {
                std::string name = _get_unique("pipe");
                dart::dynamics::SkeletonPtr pipe = dart::dynamics::Skeleton::create(name);

                // Give the pipe a body
                dart::dynamics::BodyNodePtr body = pipe->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                body->setFrictionCoeff(friction);
                body->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(pipe_x_width, pipe_y_width, pipe_z_width));

                auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(1 + (i*pipe_x_spacing), 0.0, 0.001);

                tf.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(pipe_y_rotation,  Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

                body->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(pipe);
            }

        }

        // approximate a pipe shape for larger radius
        void _add_thick_pipe(double friction = 1.0)
        {
            // create cylinders without the same name
            double pipe_x_width = 0.15;
            double pipe_y_width = 10;
            double pipe_z_width = 0.15;
            double pipe_y_rotation = 1.57079632679 / 2; // 45 degrees rototation

            _add_floor(1, 20, 10);

            for (size_t i = 0; i < 2; ++i)
            {
                std::string name = _get_unique("pipe");
                dart::dynamics::SkeletonPtr pipe = dart::dynamics::Skeleton::create(name);

                // Give the pipe a body
                dart::dynamics::BodyNodePtr body = pipe->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                body->setFrictionCoeff(friction);
                body->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(pipe_x_width, pipe_y_width, pipe_z_width));

                auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(0.6, 0.0, 0.15);

                if (i == 1)
                    tf.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(pipe_y_rotation,  Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

                body->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(pipe);
            }

        }

        void _add_thin_pipe(double friction = 1.0)
        {
            // create cylinders without the same name
            double pipe_x_width = 0.10;
            double pipe_y_width = 10;
            double pipe_z_width = 0.10;
            double pipe_y_rotation = 0.78539816339; // 45 degrees rototation

            _add_floor(1, 20, 10);

            for (size_t i = 0; i < 2; ++i)
            {
                std::string name = _get_unique("pipe");
                dart::dynamics::SkeletonPtr pipe = dart::dynamics::Skeleton::create(name);

                // Give the pipe a body
                dart::dynamics::BodyNodePtr body = pipe->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
                body->setFrictionCoeff(friction);
                body->setName(name);

                auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(pipe_x_width, pipe_y_width, pipe_z_width));

                auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

                box_node->getVisualAspect()->setColor(dart::Color::Green());

                // Put the body into position
                Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
                tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

                if (i == 1)
                    tf.linear() = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(pipe_y_rotation,  Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

                body->getParentJoint()->setTransformFromParentBodyNode(tf);

                _world->addSkeleton(pipe);
            }

        }

        // Helper function to squash dart warnings from console output
        std::string _get_unique(std::string name) {
            while (_world->getSkeleton(name) != nullptr) {
                if (name[name.size() - 2] == '_') {
                    int i = name.back() - '0';
                    i++;
                    name.pop_back();
                    name = name + std::to_string(i);
                }
                else {
                    name = name + "_1";
                }
            }

            return name;
        }

        int _world_option;
        robot_t _robot;
        Eigen::Vector3d _final_pos;
        Eigen::Vector3d _final_rot;
        double _arrival_angle;
        double _covered_distance;
        double _energy;
        double _body_avg_height;
        dart::simulation::WorldPtr _world;
		
	
        size_t _old_index;
        size_t _desc_period;
        bool _break;
		rhex_dart::Adapt adpt;
        safety_measures_t _safety_measures;		
        descriptors_t _descriptors;
        viz_t _visualizations;
        std::vector<dart::dynamics::SkeletonPtr> _objects;

#ifdef GRAPHIC
        bool _fixed_camera;
        Eigen::Vector3d _look_at;
        Eigen::Vector3d _camera_pos;
        Eigen::Vector3d _camera_up;
        osg::ref_ptr<dart::gui::osg::WorldNode> _osg_world_node;
        dart::gui::osg::Viewer _osg_viewer;
#endif
    };
}

#endif

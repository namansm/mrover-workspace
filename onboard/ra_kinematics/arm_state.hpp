#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <fstream>
#include "json.hpp"
#include <vector>
#include <map>
#include <Eigen/Dense>

using namespace Eigen;
using namespace nlohmann;
using namespace std;

typedef Matrix<double, 6, 1> Vector6d;

class ArmState{

private:
    struct Joint {
        
        Joint(string name_in, json joint_geom)
            : name(name_in), angle(0), pos_world(Vector3d::Zero(3)), 
              global_transform(Matrix4d::Identity()), torque(Vector3d::Zero(3)) 
              {
                  pos_local << joint_geom["origin"]["xyz"][0], joint_geom["origin"]["xyz"][1], joint_geom["origin"]["xyz"][2];
                  local_center_of_mass << joint_geom["mass_data"]["com"]["x"], joint_geom["mass_data"]["com"]["y"], joint_geom["mass_data"]["com"]["z"];

                  rot_axis << joint_geom["axis"][0], joint_geom["axis"][1], joint_geom["axis"][2];

                  joint_limits["lower"] = joint_geom["limit"]["lower"];
                  joint_limits["upper"] = joint_geom["limit"]["upper"];
              }

        string name;
        double angle;
        Vector3d pos_world;
        Matrix4d global_transform;
        Vector3d torque;
        Vector3d pos_local;
        Vector3d local_center_of_mass;
        Vector3d rot_axis;
        map<string, double> joint_limits;
    };

    map<string, Joint *> joints;
    static const int num_collision_parts = 23;
    Vector3d ef_pos_world;
    Matrix4d ef_xform;
    Matrix<double, num_collision_parts, num_collision_parts> collision_mat;

    void transform_parts();

    void link_link_check();

    void delete_joints();

public:
    ArmState(json &geom);
    
    ~ArmState();

    void add_joint(string joint, json &joint_geom);

    vector<string> get_all_joints();

    // vector<string> get_all_links();

    Matrix<double, num_collision_parts, num_collision_parts> get_collision_mat();

    Vector3d get_joint_com(string joint);

    double get_joint_mass(string joint);

    Vector3d get_joint_axis(string joint);

    Vector3d get_joint_axis_world(string joint);

    map<string, double> get_joint_limits(string joint);

    Matrix4d get_joint_transform(string joint);

    void set_joint_transform(string joint, Matrix4d xform);

    Matrix4d get_ef_transform();

    void set_ef_transform(Matrix4d xform);

    void set_joint_pos_world(string joint, Vector3d position);

    Vector3d get_joint_pos_world(string joint);

    Vector3d get_joint_pos_local(string joint);

    Vector3d get_ef_pos_world();

    Vector6d get_ef_pos_and_euler_angles();

    map<string, double> get_angles();

    void set_joint_angles(vector<double> angles);

    bool obstacle_free();

};

#endif
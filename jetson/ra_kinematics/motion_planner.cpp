#include "motion_planner.hpp"
#include <random>
#include <deque>


MotionPlanner::MotionPlanner(ArmState& robot_state_in, lcm::LCM& lcm_in, KinematicsSolver& solver_in) :
        robot(robot_state_in),
        lcm(lcm_in),
        solver(solver_in) {
        // TODO make solver an argument for functions rather than member variable
    
    // add limits for each joint to all_limits, after converting to degrees
    for (string& joint : robot.get_all_joints()) {
      map<string, double> limits = robot.get_joint_limits(joint);

      limits["lower"] = limits["lower"] * 180 / M_PI;
      limits["upper"] = limits["upper"] * 180 / M_PI;

      all_limits.push_back(limits);
    }

    step_limits.push_back(1);
    step_limits.push_back(1);
    step_limits.push_back(2);
    step_limits.push_back(3);
    step_limits.push_back(5);
    step_limits.push_back(1);

    neighbor_dist = 3;
    max_iterations = 1000;
    i = 0;
}

Vector6d MotionPlanner::sample() {

    Vector6d z_rand;

    for (int i = 0; i < all_limits.size(); ++i) {
        std::uniform_real_distribution<double> distr(all_limits[i]["lower"], all_limits[i]["upper"]);
        std::default_random_engine eng;

        z_rand(i) = (distr(eng));
    }

    return z_rand;
}

MotionPlanner::Node* MotionPlanner::nearest(MotionPlanner::Node* tree_root, Vector6d& rand) {

    deque<Node*> q;
    q.push_back(tree_root);

    double min_dist = numeric_limits<double>::max();
    Node* min_node = nullptr;

    while (!q.empty()) {
        Node* node = q.front();
        q.pop_front();

        double dist = (node->config - rand).norm();

        if (dist < min_dist) {
          min_dist = dist;
          min_node = node;
        }

        for (Node* child : node->children) {
          q.push_back(child);
        }
    }

    return min_node;
}


Vector6d MotionPlanner::steer(MotionPlanner::Node* start, Vector6d& end) {
    Vector6d line_vec = end - start->config;

    for (int i = 0; i < step_limits.size(); ++i) {
        if (step_limits[i] - abs(line_vec(i)) >= 0) {
            return end;
        }
    }

    double min_t = numeric_limits<double>::max();

    Vector6d new_config = start->config;

    //parametrize the line
    for (int i = 0; i < line_vec.size(); ++i) {
      
        // TODO can line_vec[i] be 0?
        double t = step_limits[i] / abs(line_vec[i]);
        if (t < min_t) {
            min_t = t;
        }
    }

    for (int i = 0; i < line_vec.size(); ++i) {
        new_config(i) += min_t * line_vec[i];
    }

    return new_config;
}

vector<Vector6d> MotionPlanner::backtrace_path(MotionPlanner::Node* end, MotionPlanner::Node* root) {
    vector<Vector6d> path;

    // starting at end, add each position config to path
    while (end != root) {
        path.push_back(get_radians(end->config));
        end = end->parent;
    }

    // include the config for the root node
    path.push_back(get_radians(end->config));
    
    return path;
}

Vector6d MotionPlanner::get_radians(Vector6d& config) {
    for (int i = 0; i < 6; ++i) {
        config(i) *= (M_PI / 180);
    }

    return config;
}

MotionPlanner::Node* MotionPlanner::extend(Node* tree, Vector6d& z_rand) {
    inc_i();
    Node* z_nearest = nearest(tree, z_rand);
    Vector6d z_new = steer(z_nearest, z_rand);

    Vector6d z_new_angs;
    for (int i = 0; i < 6; ++i) {
        z_new_angs(i) = z_new(i);
    }
    if (!solver.is_safe(z_new_angs)) {
        return nullptr;
    }
    Node* new_node = &Node(z_new);
    new_node->parent = z_nearest;
    z_nearest->children.push_back(new_node);
    new_node->cost = z_nearest->cost + (z_nearest->config - z_new).norm();
    return new_node;
}

MotionPlanner::Node* MotionPlanner::connect(Node* tree, Vector6d& a_new) {
    Node* extension = extend(tree, a_new);
    Vector6d config = extension->config;

    // TODO should we be changing tree or a_new?
    while (extension && !(config == a_new)) {
        extension = extend(tree, a_new);
        config = extension->config;
    }

    return extension;
}

void MotionPlanner::rrt_connect(Vector6d& target) {
    Vector6d start;
    start(0) = robot.get_joint_angles()["joint_a"];
    start(1) = robot.get_joint_angles()["joint_b"];
    start(2) = robot.get_joint_angles()["joint_c"];
    start(3) = robot.get_joint_angles()["joint_d"];
    start(4) = robot.get_joint_angles()["joint_e"];
    start(5) = robot.get_joint_angles()["joint_f"];

    for (int i = 0; i < target.size(); ++i) {
        target(i) = target(i) * 180 / M_PI;
        start(i) = start(i) * 180 / M_PI;
    }

    start_root = &Node(start);
    goal_root = &Node(target);

    for (int i = 0; i < max_iterations; ++i) {
        Node* a_root = i % 2 == 0 ? start_root : goal_root;
        Node* b_root = i % 2 == 0 ? goal_root : start_root;
        Vector6d z_rand = sample();

        Node* a_new = extend(a_root, z_rand);

        if (a_new) {
            Node* b_new = connect(b_root, a_new->config);

            // if the trees are connected
            if (a_new->config == b_new->config) {
                vector<Vector6d> a_path = backtrace_path(a_new, a_root);
                vector<Vector6d> b_path = backtrace_path(b_new, b_root);

                // reverse a_path
                for (int j = 0; j < a_path.size() / 2; ++j) {
                    swap(a_path[j], a_path[a_path.size() - 1 - j]);
                }

                // add the intersection of the paths to a_path
                Vector6d middle;
                for (int j = 0; j < 6; ++j) {
                    middle(j) = a_new->config(j) * M_PI / 180;
                }

                a_path.push_back(middle);
                a_path.reserve(a_path.size() + b_path.size());

                for (Vector6d b : b_path) {
                    a_path.push_back(b);
                }

                // reverse entire path
                if (i % 2) {
                    for (int j = 0; j < a_path.size() / 2; ++j) {
                        swap(a_path[j], a_path[a_path.size() - 1 - j]);
                    }
                }

                spline_size = a_path.size();

                spline_fitting(a_path);
                return;
            }
        }
    }// for loop

    // if no path found, return an empty vector
    splines = vector<tk::spline>();

}

vector<tk::spline> MotionPlanner::spline_fitting(vector<Vector6d>& path) {

    // six vectors, each with the path of a single component
    vector< vector<double> > separate_paths;
    separate_paths.resize(6, vector<double>(path.size()));

    // convert path to vectors
    for (int i = 0; i < path.size(); ++i) {
        for (int j = 0; j < 6; ++j) {
            separate_paths[j][i] = path[i](j);
        }
    }

    // create a linear space betwee 0 and 1 with path.size() increments
    vector<double> x_;
    x_.reserve(path.size() + 1);
    double spline_step = path.size() <= 1 ? 1 : 1 / (path.size() - 1);
    for (int i = 0; i <= 1; i += spline_step) {
        x_.push_back(i);
    }

    // use tk to create six different splines, representing a spline in 6 dimensions
    splines.clear();
    splines.resize(6);
    for (int i = 0; i < 6; ++i) {
        splines[i].set_points(x_, separate_paths[i]);
    }
}

vector<double> MotionPlanner::get_spline_pos(double spline_t) {
    vector<double> angles;
    angles.reserve(6);

    for (const tk::spline& spline : splines) {
        angles.emplace_back(spline(spline_t));
    }

    // invert 5th angle for use purposes
    //angles[4] *= -1;

    return angles;
}

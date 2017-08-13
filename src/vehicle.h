#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  double s;

  double v;

  double a;

  double target_speed;

  int lanes_available;

  double max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, double s, double v, double a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<double> > > predictions);

  void configure(vector<double> road_data);

  void increment(double dt);

  vector<double> state_at(double t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<double> > > predictions);

  void realize_constant_speed();

  double _max_accel_for_lane(map<int,vector<vector<double> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<double> > > predictions);

  void realize_lane_change(map<int,vector< vector<double> > > predictions, string direction);

  void realize_prep_lane_change(map<int,vector< vector<double> > > predictions, string direction);

  vector<vector<double> > generate_predictions(int horizon);

};

#endif
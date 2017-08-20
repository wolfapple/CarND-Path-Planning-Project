#ifndef COST_H
#define COST_H

#define COLLISION   5.0
#define DANGER      3.0
#define COMFORT     0.75
#define EFFICIENCY  3.0
#define DESIRED_BUFFER 0.03
#define PLANNING_HORIZON 2

#include "vehicle.h"

struct TrajectoryData {
  int proposed_lane;
  double avg_speed;
  double max_acceleration;
  double rms_acceleration;
  double closest_approach;
  Vehicle::collider collides;
};

double change_lane_cost(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions, TrajectoryData data) {
  int proposed_lane = data.proposed_lane;
	int cur_lanes = trajectory[0].lane;
  // double cost = 0;
  // if (proposed_lane != cur_lanes) cost = COMFORT;
  // return cost;
  double pct = (proposed_lane - cur_lanes) / (vehicle.lanes_available - 1);
  return COMFORT * (1.0 - pct);
}

double inefficiency_cost(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions, TrajectoryData data) {
  double speed = data.avg_speed;
  double target_speed = vehicle.target_speed;
  return target_speed - speed;
  double pct = (target_speed - speed) / target_speed;
  return EFFICIENCY * (1.0 - pct);
  // return pct*pct*EFFICIENCY;
}

double collision_cost(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions, TrajectoryData data) {
  if (data.collides.collision) {    
    double ttc = data.collides.time;
    double mult = exp(-(ttc*ttc));
    return mult * COLLISION;
  } else {
    return 0.0;
  }
}

double buffer_cost(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions, TrajectoryData data) {
  if (data.closest_approach == 0.0) return 10 * DANGER;
  double timesteps_away = data.closest_approach / data.avg_speed;
  if (timesteps_away > DESIRED_BUFFER) return 0.0;
  double mult = 1.0 - pow((timesteps_away / DESIRED_BUFFER), 2);
  return mult * DANGER;
}

map<int, vector< vector<double> > > filter_predictions_by_lane(map<int, vector< vector<double> > > predictions, int lane) {
  map<int,vector< vector<double> > > filtered;
  map<int,vector< vector<double> > >::iterator it = predictions.begin();
  while(it != predictions.end()) {
    int v_id = it->first;
    vector<vector<double> > v = it->second;    
    if(v[0][0] == lane) {
      filtered[v_id] = v;
    }
    it++;
  }
  return filtered;
}

bool check_collision(Vehicle::SnapShot snapshot, double s_previous, double s_now) {
  double s = snapshot.s;
  double v = snapshot.v;
  double v_target = s_now - s_previous;
  if (s_previous < s) {
    if (s_now >= s) return true;
    else return false;
  }
  if (s_previous > s) {
    if (s_now <= s) return true;
    else return false;
  }
  if (s_previous == s) {
    if (v_target > v) return false;
    else return true;
  }
  return false;
}

TrajectoryData get_helper_data(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions) {
  Vehicle::SnapShot current = trajectory[0];
  Vehicle::SnapShot first = trajectory[1];
  Vehicle::SnapShot last = trajectory.back();

  double dt = (double)trajectory.size();
  int proposed_lane = first.lane;
  double avg_speed = (last.s - current.s) / dt;  

  double closest_approach = 999999;
  double max_accel = 0;
  vector<double> rms_accels;
  Vehicle::collider collider;
  collider.collision = false;
  map<int,vector< vector<double> > > filtered = filter_predictions_by_lane(predictions, proposed_lane);
  for (int i=1;i < PLANNING_HORIZON + 1; i++) {
    Vehicle::SnapShot snapshot = trajectory[i];
    rms_accels.push_back(snapshot.a*snapshot.a);
    if (abs(snapshot.a) > abs(max_accel)) max_accel = snapshot.a;
    map<int,vector< vector<double> > >::iterator it = filtered.begin();
    while(it != filtered.end()) {
      vector<vector<double> > v = it->second;
      vector<double> state = v[i];
      vector<double> last_state = v[i-1];
      bool collides = check_collision(snapshot, last_state[1], state[1]);
      if (collides) {
        collider.collision = true;
        collider.time = i;
      }
      double dist = abs(state[1] - snapshot.s);
      if (dist < closest_approach) closest_approach = dist;
      it++;
    }
  }
  double rms_accel = accumulate(rms_accels.begin(), rms_accels.end(), 0.0) / rms_accels.size();

  TrajectoryData data;
  data.proposed_lane = proposed_lane;
  data.avg_speed = avg_speed;
  data.max_acceleration = max_accel;
  data.rms_acceleration = rms_accel;
  data.closest_approach = closest_approach;
  data.collides = collider;

  return data;
}

double calculate_cost(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions) {
  TrajectoryData data = get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;
  cost += inefficiency_cost(vehicle, trajectory, predictions, data);
  // cost += collision_cost(vehicle, trajectory, predictions, data);
  // cost += buffer_cost(vehicle, trajectory, predictions, data);
  cost += change_lane_cost(vehicle, trajectory, predictions, data);
  return cost;
}

#endif
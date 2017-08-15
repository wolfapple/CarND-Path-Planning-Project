#include "cost.h"
#include "vehicle.h"

double Cost::change_lane_cost(Vehicle vehicle,  vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions, Cost::TrajectoryData data) {
  // Penalizes lane changes AWAY from the goal lane and rewards
  // lane changes TOWARDS the goal lane.
  
  int currLane = vehicle.lane;
  int proposed_lane = data.propsed_lane;
  double cost  = 0;
  
  if(proposed_lane > currLane)
      cost = COMFORT;
  else if(proposed_lane < currLane)
      cost = -COMFORT;
  
  return cost;
}

double TrajectoryCost::speed_cost(Vehicle vehicle,vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions,TrajectoryCost::TrajectoryData trajectoryData){
    int target_speed = vehicle.target_speed;
    float avgSpeed = trajectoryData.avgSpeed;
    double multiplier = (target_speed - avgSpeed) / target_speed;
    double cost = multiplier * multiplier * EFFICIENCY;
    return cost;
}


double TrajectoryCost::collision_cost(Vehicle vehicle,vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions,TrajectoryCost::TrajectoryData trajectoryData){
    double cost = 0;
    if(trajectoryData.collides.collision == true) {
        double timeToCollision = trajectoryData.collides.time;
        timeToCollision *= timeToCollision;
        double multiplier = exp(-timeToCollision);
        cost = multiplier * COLLISION;
    }
    return cost;
}


double TrajectoryCost::buffer_cost(Vehicle vehicle,vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions,TrajectoryCost::TrajectoryData trajectoryData){
    
    int closestDist = trajectoryData.closestDist;
    float tToCollision = closestDist / (float) trajectoryData.avgSpeed;
    double cost = 0;
    if(closestDist == 0)
        cost = 10 * DANGER;
    
    else if(tToCollision > DESIRED_BUFFER){
        cost = 0;
    }
    else{
        double multiplier = tToCollision/DESIRED_BUFFER;
        multiplier *= multiplier;
        multiplier = 1 - multiplier;
        cost = multiplier * DANGER;
        
    }
    
    return cost;
}

map<int,vector< vector<double> > > TrajectoryCost::filterVehicles(map<int,vector< vector<double> > > predictions, int lane){
    map<int,vector< vector<double> > > ret;
    map<int,vector< vector<double> > >::iterator it;
    for(it = predictions.begin(); it != predictions.end(); it++){
        if(it->second[0][0] == lane && it->first != -1 ){
            ret[it->first] = it->second;
        }
    }
    return ret;
}

TrajectoryCost::TrajectoryData TrajectoryCost::getTrajectoryData(const vector<Vehicle::SnapShot>& trajectory, const Vehicle& vehicle, map<int,vector< vector<double> > > predictions){
    TrajectoryCost::TrajectoryData trData;
    Vehicle::SnapShot last = trajectory[trajectory.size()-1];
    Vehicle::SnapShot first = trajectory[1];
    Vehicle::SnapShot current = trajectory[0];
    int dt = (trajectory.size());
    
    if(first.state == "PLCR") {
        trData.proposedLane = min(current.lane-1, vehicle.lanes_available);
    }
    else if(first.state == "PLCL"){
        trData.proposedLane = max(0,current.lane+1);        
    }
    trData.avgSpeed = (last.s - current.s) / (double)dt;
    
    vector<double> accels;
    Collider collider;
    collider.collision = false;
    double closestDist = 9999999;
    double maxAcc = -99999;
    double rmsAcc = 0;
    map<int,vector< vector<double> > > laneVehicles = filterVehicles(predictions,last.lane);
    for(int i = 1; i < dt; ++i){
        Vehicle::SnapShot myCar = trajectory[i];
        double a = myCar.a;
        accels.push_back(a);
        rmsAcc += (a*a);
        if(a >maxAcc)
            maxAcc = a;
        
        for(map<int,vector< vector<double> > >::iterator j= laneVehicles.begin() ; j != laneVehicles.end() ; j++){
            
            vector<double> otherCar_prev = j->second[i-1];
            vector<double> otherCar_state = j->second[i];
            if(collider.collision == false){
                if( (otherCar_prev[1] < myCar.s && otherCar_state[1] >= myCar.s) ||
                   (otherCar_prev[1] > myCar.s && otherCar_state[1] <= myCar.s)) {
                    collider.collision = true;
                    collider.time = i;
                }
                
            }
            int distance = abs(otherCar_state[1] - myCar.s);
            if(distance < closestDist){
                closestDist = distance;
            }
            
            
            
        }
        
    }
    
    trData.maxAccl = maxAcc;
    trData.rmsAccl = (float)rmsAcc / accels.size();
    trData.closestDist = closestDist;
    trData.collides = collider;
    
    
    return trData;
}

double TrajectoryCost::calculateCost(Vehicle vehicle, vector<Vehicle::SnapShot> trajectory, map<int,vector< vector<double> > > predictions) {
  double cost = 0;
  TrajectoryCost::TrajectoryData trajData = getTrajectoryData(trajectory, vehicle, predictions);
//   for(int i = 0; i< cf.size(); i++) {
//     CostFnPtr costFunc = cf[i];
//     cost += (this->*costFunc)(vehicle, trajectory, predictions,trajData);
//   }
  return cost;
}
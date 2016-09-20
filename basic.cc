///////////////////////////////
// File: basic.cc
// Desc: minimal controller example
// Created: 2011.10.19
// Author: Richard Vaughan <vaughan@sfu.ca>
// License: GPL
/////////////////////////////////

#include "stage.hh"
#include "map"
#include "math.h"
#include "set"
using namespace Stg;
using namespace std;

enum RobotState
{
    UNKNOWN  = 0,
    SEARCHING = 1,
    HOMING = 2
};

#define VEL 0.9
#define PI 3.14159265359

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

typedef struct
{
    ModelPosition* position;
    ModelRanger* ranger;
    ModelRanger* laser;
    ModelFiducial* fiducial;
    RobotState state;
    bool foundDist;
    bool avoiding;
    Pose target;
    set< pair< int, int> > trail;
} robot_t;

typedef struct
{
    double w;
    double ph ;
}cell;

std::map <std::pair< int, int>, cell> mp;

/** this is added as a callback to a ranger model, and is called
    whenever the model is updated by Stage.
*/
int RangerUpdate( ModelRanger* mod, robot_t* robot)
{
    const std::vector<ModelRanger::Sensor>& sensors = mod->GetSensors();

    // ( inspect the ranger data and decide what to do )

    if( robot->state == SEARCHING)
        robot->ranger->Say( "Searching");
    else if(robot->state == HOMING)
        robot->ranger->Say( "Homing");
    // GUI window
    //    for( int i = 0; i < robot->position->waypoints.size(); i++)
    //    std::cout << robot->position->GetGlobalPose().x << robot->position->GetGlobalPose().y << " " << robot->position->GetGlobalPose().a << std::endl;


    robot->trail.insert(pair< int, int>(round(robot->position->GetGlobalPose().x), round(robot->position->GetGlobalPose().y)));
    // map update thingy
    for( int i = -20; i < 20; i++)
        for( int j = -20; j < 20; j++)
            if(mp[std::pair<int, int>(i, j)].ph > 0)
                mp[std::pair<int, int>(i, j)].ph -= 0.001;
    if( robot->state == SEARCHING)
    {
        mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].w = robot->position->GetGlobalPose().a;
        mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].ph += 3;
    }
    else
    {
        if( robot->position->GetGlobalPose().a < 0)
            mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].w = PI+robot->position->GetGlobalPose().a;
        else
            mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].w = PI-robot->position->GetGlobalPose().a;
        mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].ph += 3;
    }

    if(robot->foundDist)
    {
        bool charged = false;
        int rot = 0;
        for( int i = 0; i < 4; i++)
            if( sensors[i].ranges[0] < 4)
            {
                charged = true;
                rot = -1;
            }
        for( int i = 4; i < 8; i++)
            if( sensors[i].ranges[0] < 4)
            {
                charged = true;
                rot = 1;
            }
        if(false && rot)
        {
            robot->position->SetSpeed( 0.1, 0, rot*0.9 );
            robot->avoiding = rot;
        }
        if( charged)
        {
            if( robot->state == SEARCHING)
            {
                robot->state = HOMING;
                robot->foundDist = false;
                robot->trail.clear();
            }
            else
            {
                robot->state = SEARCHING;
                robot->foundDist = false;
                robot->trail.clear();
            }
        }
    }
    //    for( int i = -20; i < 20; i++)
    //        for( int j = -20; j < 20; j++)
    //            std::cout << map[std::pair<int, int>(i, j)].w;
    //    std::cout << std::endl;
    return 0;
}

int LaserUpdate( ModelRanger* mod, robot_t* robot)
{
    const std::vector<meters_t>& scan = robot->laser->GetSensors()[0].ranges;
    int sample_count = scan.size();
    if( sample_count < 1)
        return 0;
    if( robot->foundDist)
    {
        double y = robot->target.y - robot->position->GetGlobalPose().y;
        double x = robot->target.x - robot->position->GetGlobalPose().x;
        double rotation = normalize( atan2( y, x) - robot->position->GetGlobalPose().a);
        robot->position->SetSpeed( VEL, 0, rotation);
    }
    else
    {
        double desiredRot = 10;
        double x;
        double y;
        if( robot->state == SEARCHING)
        {
            double pheromone = 0;
            for( int i = -3; i < 4; i++)
                for( int j = -3; j < 4; j++)
                {
                    if( abs(i) + abs(j) > 5 || robot->trail.find( pair<int, int> (round( robot->position->GetGlobalPose().x + i), round( robot->position->GetGlobalPose().y + j))) != robot->trail.end())
                        continue;
                    double ph = mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x + i), round( robot->position->GetGlobalPose().y + j))].ph;
                    double w = mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x + i), round( robot->position->GetGlobalPose().y + j))].w;
                    if( ph > pheromone)
                    {
                        pheromone = ph;
                        desiredRot = w;
                    }
                    x = robot->position->GetGlobalPose().x + i;
                    y = robot->position->GetGlobalPose().y + j;
                }
        }
        else if( robot->state == HOMING)
        {
            double pheromone = 0;
            for( int i = -3; i < 4; i++)
                for( int j = -3; j < 4; j++)
                {
                    if( robot->trail.find( pair<int, int> (round( robot->position->GetGlobalPose().x + i), round( robot->position->GetGlobalPose().y + j))) != robot->trail.end())
                        continue;
                    double ph = mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x + i), round( robot->position->GetGlobalPose().y + j))].ph;
                    double w = mp[std::pair<int, int>(round( robot->position->GetGlobalPose().x + i), round( robot->position->GetGlobalPose().y + j))].w;
                    if( ph > pheromone)
                    {
                        pheromone = ph;
//                        if( w < 0)
                            desiredRot = normalize(PI+w);
//                        else
//                            desiredRot = PI-w;
                    }
                    x = robot->position->GetGlobalPose().x + i;
                    y = robot->position->GetGlobalPose().y + j;
                }
        }
        double rot = 0;
        double vel = VEL;
        if( desiredRot != 10)
        {
    //                double yy = y - robot->position->GetGlobalPose().y;
    //                double xx = x - robot->position->GetGlobalPose().x;
    //                double rotation = normalize( atan2( yy, xx) - robot->position->GetGlobalPose().a);

            rot =  atan2(sin(desiredRot- robot->position->GetGlobalPose().a), cos(desiredRot-robot->position->GetGlobalPose().a));
            if( fabs(rot) > PI/6)
                vel = 0;
//            else
//            {
//                double yy = y - robot->position->GetGlobalPose().y;
//                double xx = x - robot->position->GetGlobalPose().x;
//                double rotation = normalize( atan2( yy, xx) - robot->position->GetGlobalPose().a);
//                robot->position->SetSpeed( VEL, 0, rotation);
//                return 0;
//            }
        }
        std::cout << desiredRot << " " << rot <<  std::endl;
        if( fabs(rot) > 0.3)
            rot = 0.3*sign(rot);
        for( int i = 0; i < 90; i++)
            if( scan[i] < 0)
                rot = 0.3;
        for( int i = 90; i < 180; i++)
            if( scan[i] < 0)
                rot = -0.3;
        robot->position->SetSpeed( vel, 0, rot);
        robot->avoiding = sign(rot);
    }
    return 0;
}

int FiducialUpdate( ModelFiducial* fid, robot_t* robot)
{
    // find the closest teammate
    unsigned int n=0;
    ModelFiducial::Fiducial* fd = fid->GetFiducials(&n);
    if(n > 0){
        for(int i=0;i<n;i++){
            if( !robot->foundDist && fd[i].id == 3 && robot->state == SEARCHING)
            {
                robot->foundDist = true;
                robot->target.x = fd[i].pose.x;
                robot->target.y = fd[i].pose.y;
            }
            else if( !robot->foundDist && fd[i].id == 33 && robot->state == HOMING)
            {
                robot->foundDist = true;
                robot->target.x = fd[i].pose.x;
                robot->target.y = fd[i].pose.y;
            }
        }
    }
    return 0;
}

/** Stage calls this when the model starts up 
 */
extern "C" int Init( Model* mod )
{
    robot_t* robot = new robot_t;
    robot->position = (ModelPosition*)mod;

    std::cout << "Initing";
    // subscribe to the ranger, which we use for navigating
    robot->ranger = (ModelRanger*)mod->GetChild( "ranger:0" );
    assert( robot->ranger );
    robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );
    assert( robot->laser );
    robot->fiducial = (ModelFiducial*)mod->GetUnusedModelOfType( "fiducial" ) ;
    assert( robot->fiducial );
    // ask Stage to call into our ranger update function
    robot->ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)RangerUpdate, robot );
    robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)LaserUpdate, robot );
    robot->fiducial->AddCallback( Model::CB_UPDATE, (model_callback_t)FiducialUpdate, robot );

    robot->ranger->Subscribe();
    robot->position->Subscribe();
    robot->laser->Subscribe();
    robot->fiducial->Subscribe();

    //Robot initialization
    robot->state = SEARCHING;
    robot->foundDist = false;
    robot->avoiding = false;

    for( int i = -20; i < 20; i++)
        for( int j = -20; j < 20; j++)
        {
            mp[std::pair<int, int>(i, j)].w = 0;
            mp[std::pair<int, int>(i, j)].ph = 0;
        }
    return 0; //ok
}



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
using namespace Stg;

enum RobotState
{
    UNKNOWN  = 0,
    SEARCHING = 1,
    HOMING = 2
};

#define VEL = 0.9;

typedef struct
{
    ModelPosition* position;
    ModelRanger* ranger;
    ModelRanger* laser;
    ModelFiducial* fiducial;
    enum RobotState state;
    bool foundDist;
    bool avoiding;
    Pose target;
} robot_t;

typedef struct
{
    double w;
    double ph = 0;
}cell;

std::map <std::pair< int, int>, cell> map;

/** this is added as a callback to a ranger model, and is called
    whenever the model is updated by Stage.
*/
int RangerUpdate( ModelRanger* mod, robot_t* robot)
{
    const std::vector<ModelRanger::Sensor>& sensors = mod->GetSensors();

    // ( inspect the ranger data and decide what to do )

    char toSay[9];
    sprintf( toSay, "%d", robot->position->GetId());
    robot->ranger->Say( toSay); // GUI window
//    for( int i = 0; i < robot->position->waypoints.size(); i++)
//    std::cout << robot->position->GetGlobalPose().x << robot->position->GetGlobalPose().y << " " << robot->position->GetGlobalPose().a << std::endl;


    // map update thingy
    for( int i = -20; i < 20; i++)
        for( int j = -20; j < 20; j++)
            if(map[std::pair<int, int>(i, j)].ph > 0)
                map[std::pair<int, int>(i, j)].ph -= 0.001;
    map[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].ph += 3;
    map[std::pair<int, int>(round( robot->position->GetGlobalPose().x), round( robot->position->GetGlobalPose().y))].w = robot->position->GetGlobalPose().a;

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
    if( sample_count < 1 )
        return 0;
    int rot = 0;
    for( int i = 0; i < 90; i++)
        if( scan[0] < 3)
            rot = -1;
    for( int i = 90; i < 180; i++)
        if( scan[0] < 3)
            rot = 1;
    robot->position->SetSpeed( 0.9, 0, rot*0.3 );
    robot->avoiding = rot;
    return 0;
}

int FiducialUpdate( ModelFiducial* fid, robot_t* robot)
{
    // find the closest teammate
    unsigned int n=0;
    ModelFiducial::Fiducial* fd = fid->GetFiducials(&n);
    robot->foundDist = false;
    if(n > 0){
        for(int i=0;i<n;i++){
            if( fd[i].id == 3 && robot->state == RobotState::SEARCHING)
            {
                robot->foundDist = true;
                robot->target.x = fd[i].pose.x;
                robot->target.y = fd[i].pose.y;
            }
            else if( fd[i].id == 33 && robot->state == RobotState::HOMING)
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
    robot->state = RobotState::SEARCHING;
    robot->foundDist = false;
    robot->avoiding = false;

    for( int i = -20; i < 20; i++)
        for( int j = -20; j < 20; j++)
        {
            map[std::pair<int, int>(i, j)].w = 0;
            map[std::pair<int, int>(i, j)].ph = 0;
        }
    return 0; //ok
}



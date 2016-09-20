///////////////////////////////
// File: basic.cc
// Desc: minimal controller example
// Created: 2011.10.19
// Author: Richard Vaughan <vaughan@sfu.ca>
// License: GPL
/////////////////////////////////

#include "stage.hh"
using namespace Stg;

typedef struct
{
    ModelPosition* position;
    ModelRanger* ranger;
    ModelRanger* laser;
    ModelFiducial* fiducial;
} robot_t;

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
    int rot = 0;
    for( int i = 0; i < 4; i++)
        if( sensors[i].ranges[0] < 5)
            rot = -1;
    for( int i = 4; i < 8; i++)
        if( sensors[i].ranges[0] < 5)
            rot = 1;
    robot->position->SetSpeed( 0.4, 0, rot*0.3 );  // output a speed command (X, Y, Z)


    return 0;
}

int LaserUpdate( ModelRanger* mod, robot_t* robot)
{
    const std::vector<meters_t>& scan = robot->laser->GetSensors()[0].ranges;
    int sample_count = scan.size();
    if( sample_count < 1 )
        return 0;
    std::cout << mod->GetId() << " -- ";
    for (uint32_t i = 0; i < sample_count; i++){
        std::cout  << i << " -> " << scan[i] << " | ";
    }
    std::cout << std::endl;
    return 0;
}

int FiducialUpdate( ModelFiducial* fid, robot_t* robot)
{
    // find the closest teammate
    unsigned int n=0;
    ModelFiducial::Fiducial* fd = fid->GetFiducials(&n);
    if(n > 0){
        for(int i=0;i<n;i++){
            std::cout << fd[i].id << std::endl;
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

    return 0; //ok
}



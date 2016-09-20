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
    radians_t closest_bearing;
    meters_t closest_range;
    radians_t closest_heading_error;

} robot_t;


static ModelPosition* position(NULL);

/** this is added as a callback to a ranger model, and is called
    whenever the model is updated by Stage.
*/
int RangerUpdate( ModelRanger* mod, robot_t* robot)
{
    const std::vector<ModelRanger::Sensor>& sensors = robot->ranger->GetSensors();

    // ( inspect the ranger data and decide what to do )

    // output something to prove it is working
    char toSay[9];
    sprintf( toSay, "%d", robot->position->GetId());
    robot->ranger->Say( toSay); // GUI window
    robot->position->SetSpeed( 0.4, 0, 0.1 );  // output a speed command (X, Y, Z)
    return 0;
}

/** Stage calls this when the model starts up 
 */
extern "C" int Init( Model* mod )
{
    robot_t* robot = new robot_t;
    robot->position = (ModelPosition*)mod;

    // subscribe to the ranger, which we use for navigating
    robot->ranger = (ModelRanger*)mod->GetUnusedModelOfType( "ranger" );
    assert( robot->ranger );

    // ask Stage to call into our ranger update function
    robot->ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)RangerUpdate, robot );
    robot->ranger->Subscribe();
    robot->position->Subscribe();
    return 0; //ok
}



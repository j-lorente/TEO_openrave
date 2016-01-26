// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Recovery.hpp"

/**************************************************************************************************************/

// VIEWER
void SetViewer(EnvironmentBasePtr penv, const string& viewername) {
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    penv->AddViewer(viewer); //Attach viewer to the environment
    viewer->main(true);
}

/**************************************************************************************************************/

//CAMERA
void SetCamera(EnvironmentBasePtr penv, dReal tx, dReal ty, dReal tz, dReal axisx, dReal axisy, dReal axisz,
               dReal ang)
{
    //For the rotations, openrave uses quaternions.
    //The conversion between a rotation given by a rotation axis + angle and the quaterion is given by:
    dReal q0 = cos(ang/2);
    dReal q1 = axisx*sin(ang/2);
    dReal q2 = axisy*sin(ang/2);
    dReal q3 = axisz*sin(ang/2);
    Vector rotation(q0,q1,q2,q3);

    //Obtain the translation vector (directly from tx,ty and tz)
    Vector translation(tx,ty,tz);

    //Calculate the transformation given by the rotation and translation
    Transform T(rotation,translation);

    //Apply the transformation to the camera
    penv->GetViewer()->SetCamera(T);
}

/**************************************************************************************************************/

bool Recovery::init() {

    //OPENRAVE
    RaveInitialize(true); //Start OpenRAVE
    penv = RaveCreateEnvironment(); //Create the environment
    //RaveSetDebugLevel(Level_Debug);
    string viewername = "qtcoin";
    boost::thread thviewer(boost::bind(SetViewer,penv,viewername)); //Launch the viewer on a different thread
    string scenefilename = "../../Environment/env/teo_push.xml";
    penv->Load(scenefilename); //Load the scene
    usleep(300000); //Wait for the viewer to initialize

    //Set the camera view
    SetCamera(penv, 1.015330, -6.947658, 1.008429, -0.990864, -0.095934, 0.094790, 1.458429);

    //LOAD PLUGIN
    RaveLoadPlugin("../../Plugin/pidbuild/libpidcontroller");

    //GET ROBOT
    vector<RobotBasePtr> robots;
    penv->GetRobots(robots);
    cout << "Robot: " << robots.at(0)->GetName() << endl;
    probot = robots.at(0);

    //GET MANIPULATORS
    vector<RobotBase::ManipulatorPtr> vectorOfManipulatorPtr = probot->GetManipulators();
    for(size_t j=0;j<vectorOfManipulatorPtr.size();j++) {
        cout << "* Manipulator[" << j << "]: " << vectorOfManipulatorPtr[j]->GetName() << endl;
    }

    //PID CONTROLLER
    pcontrol = RaveCreateController(penv,"PIDController 500 20 20");
    {
        EnvironmentMutex::scoped_lock lock(penv->GetMutex()); //Lock environment (prevents changes)
        vector<int> dofindices(probot->GetDOF());
        for(int i = 0; i < probot->GetDOF(); ++i) {
            dofindices[i] = i;
        }
        probot->SetController(pcontrol,dofindices,1);
    }

    //GET ROBOT'S DEGREES OF FREEDOM
    vector<dReal>dEncRaw(probot->GetDOF());

    //GET LINK FROM THE ROBOT
    plink = probot->GetLink("r31");

    //SET FORCE TO BE APPLIED
    Vector force;
    force.x=-250;
    force.y=0;
    force.z=0;
    Vector position;
    position.x=0;
    position.y=0;
    position.z=0;

    pcontrol->SetDesired(dEncRaw); //Set desired joint positions

    while(true){

        int sim_time = penv->GetSimulationTime(); //Simulation time in microseconds

        if (sim_time>3000000 && sim_time<3500000)
        {
            plink->SetForce(force, position, false); //Apply force to torso
        }

        if (sim_time>8000000 && sim_time<8500000) //After 8 seconds of simulation
        {
            force.x=250;
            plink->SetForce(force, position, false); //Apply force to torso
        }

        //GET ROBOT'S COM
        Vector com = probot->GetCenterOfMass();
        cout << "Center of Mass: " << com << endl;

        pcontrol->SetDesired(dEncRaw); //Set desired joint positions

        usleep(10000);

    }

    return true;
}



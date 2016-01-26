#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;

class PIDController : public ControllerBase
{
public:

    PIDController(EnvironmentBasePtr penv, std::istream& ss) : ControllerBase(penv)
    {
        RegisterCommand("MyCommand",boost::bind(&PIDController::MyCommand,this,_1,_2),
                        "This is an example command");
        RegisterCommand("status",boost::bind(&PIDController::ControlStatus,this,_1,_2),
                        "Console display of the controller variables");
        RegisterCommand("parameters",boost::bind(&PIDController::ControlParameters,this,_1,_2),
                        "Set controller parameters");
        controlParameters.resize(3);
        std::cout << "PID parameters: ";
        for (int i=0; i<3; i++)
        {
            ss >> controlParameters[i];
            std::cout << controlParameters[i] << " ";
        }
        std::cout << std::endl;


    }
    virtual ~PIDController() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }

    bool ControlStatus(std::ostream& sout, std::istream& sinput)
    {
        std::cout << " desired ";
        ConsolePrintStdVector(desired);
        std::cout << " controlError ";
        ConsolePrintStdVector(controlError);
        std::cout << " actual ";
        ConsolePrintStdVector(actual);
        std::cout << " control Parameters ";
        ConsolePrintStdVector(controlParameters);
        std::cout << " control Signal ";
        ConsolePrintStdVector(controlSignal);


        return true;

    }

    bool ControlParameters(std::ostream& sout, std::istream& sinput)
    {
        controlParameters.resize(3);
        for (uint i=0; i<controlParameters.size(); i++)
        {
        sinput >> controlParameters[i];
        }
        std::cout << " new pid parameters ";
        ConsolePrintStdVector(controlParameters);

        return true;

    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        _dofindices = dofindices;
        _nControlTransformation = nControlTransformation;
        DOFResize(dofindices.size());
        Reset(0);
        return true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return _nControlTransformation;
    }

    virtual void Reset(int options)
    {
        DOFResize(_dofindices.size());

        desired.assign(desired.size(),0);
        actual.assign(actual.size(),0);
        controlError.assign(controlError.size(),0);
        controlErrorOld.assign(controlErrorOld.size(),0);
        controlErrorD.assign(controlErrorD.size(),0);
        controlErrorI.assign(controlErrorI.size(),0);
        /*controlParameters*/
        if (controlParameters.size()<1)
        {
            controlParameters.push_back(10);
            controlParameters.push_back(5);
            controlParameters.push_back(10);
        }

    }
    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        DOFResize(_dofindices.size());
        desired = values;
        ConsolePrintStdVector(desired);
        return true;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        return false;
    }
    virtual void SimulationStep(dReal fTimeElapsed)
    {

        _probot->GetDOFValues(actual);
        ControlError(fTimeElapsed);
        if (controlSignal.size() > 0)
        {
            _probot->SetDOFTorques(ControlSignal(), false);
        }


    }
    virtual bool IsDone() {
        return false;
    }
    virtual dReal GetTime() const {
        return 0;
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }
    bool ControlError(dReal timeInterval)
    {

        controlErrorOld = controlError;
        //error
        for (uint i=0; i<controlError.size(); i++)
        {
            controlError[i]=desired[i]-actual[i];
        }
        //error integrator
        for (uint i=0; i<controlError.size(); i++)
        {
            controlErrorI[i]=(controlError[i]+controlErrorOld[i])*timeInterval;
        }
        //error derivative
        for (uint i=0; i<controlError.size(); i++)
        {
            controlErrorD[i]=(controlError[i]-controlErrorOld[i])/timeInterval;
        }
        return true;
    }

    std::vector<dReal> ControlSignal()
    {
        controlSignal=controlError;

        for (uint i=0 ; i<controlError.size() ; i++)
        {
            controlSignal[i]=controlError[i]*controlParameters[0]
                    +controlErrorI[i]*controlParameters[1]
                    +controlErrorD[i]*controlParameters[2];
        }
        return controlSignal;
    }

    int DOFResize(int newSize)
    {
        desired.resize(newSize);
        actual.resize(newSize);
        controlError.resize(newSize);
        controlErrorI.resize(newSize);
        controlErrorD.resize(newSize);
        controlErrorOld.resize(newSize);
        controlSignal.resize(newSize);
        return 0;
    }

    bool ConsolePrintStdVector(std::vector<dReal>& printVector)
    {
        std::cout << "Vector of " << printVector.size() << " dimensions: ";
        for (uint i=0 ; i<printVector.size() ; i++)
        {
            std::cout << printVector[i] << " ";
        }
        std::cout << std::endl;
        return true;
    }

protected:
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    int _nControlTransformation;
    std::vector<dReal> desired,actual;
    std::vector<dReal> controlError,controlErrorOld;
    std::vector<dReal> controlErrorD,controlErrorI;
    std::vector<dReal> controlSignal;
    std::vector<dReal> controlParameters;

private:

};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Controller && interfacename == "pidcontroller" ) {
        return InterfaceBasePtr(new PIDController(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Controller].push_back("PIDController");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}


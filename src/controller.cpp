// Tutorial on Inverse Kinematics.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <mutex>
#include <string>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/minJerkCtrl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class Controller : public RFModule
{
    double link_length;
    string mode;

    BufferedPort<Vector> portMotors;
    BufferedPort<Vector> portEncoders;
    BufferedPort<Vector> portTarget;
    BufferedPort<Vector> portTipVel;
    RpcServer portCmd;

    mutex mtx;
    Vector encoders;
    Vector target;
    Vector gains;

    bool usePlanner;
    minJerkTrajGen *planner;

    // compute the 2D position of the tip of the manipulator
    Vector forward_kinematics(const Vector &j) const
    {
        Vector ee(2);
        ee[0]=link_length*(cos(j[0])+cos(j[0]+j[1]));
        ee[1]=link_length*(sin(j[0])+sin(j[0]+j[1]));
        return ee;
    }

    // compute the Jacobian of the tip's position
    Matrix jacobian(const Vector &j) const
    {
        Matrix J(2,2);
        J(0,0)=-link_length*(sin(j[0])+sin(j[0]+j[1])); J(0,1)=-link_length*sin(j[0]+j[1]);
        J(1,0)=link_length*(cos(j[0])+cos(j[0]+j[1])); J(1,1)=link_length*cos(j[0]+j[1]);
        return J;
    }

public:
    bool configure(ResourceFinder &rf)override
    {
        link_length=rf.check("link-length",Value(100.0)).asDouble();

        portMotors.open("/tutorial_inverse-kinematics-controller/motors:o");
        portEncoders.open("/tutorial_inverse-kinematics-controller/encoders:i");
        portTarget.open("/tutorial_inverse-kinematics-controller/target:o");
        portTipVel.open("/tutorial_inverse-kinematics-controller/tip-vel:o");
        portCmd.open("/tutorial_inverse-kinematics-controller/cmd:rpc");
        attach(portCmd);

        // init the encoder values of the 2 joints
        encoders=zeros(2);

        // init the target position
        target=zeros(2);

        // init gains
        gains.resize(3);
        gains[0]=0.0004;
        gains[1]=1.0;
        gains[2]=2.0;

        // init planner
        planner=new minJerkTrajGen(target,getPeriod(),4.0);
        usePlanner=false;

        // init mode
        mode="idle";

        return true;
    }

    bool close()override
    {
        delete planner;

        portMotors.close();
        portEncoders.close();
        portTarget.close();
        portTipVel.close();
        portCmd.close();

        return true;
    }

    double getPeriod()override
    {
        return 0.01;
    }

    bool updateModule()override
    {
        // protect against race conditions
        lock_guard<mutex> lg(mtx);

        // update the encoder readouts from the net
        if (Vector *enc=portEncoders.read(false))
            encoders=*enc;

        // plan target trajectory
        planner->computeNextValues(target);

        // select desired end-effector position
        Vector ee_d=(usePlanner?planner->getPos():target);

        // compute quantities for differential kinematics
        Vector ee=forward_kinematics(encoders);
        Matrix J=jacobian(encoders);
        Vector err=ee_d-ee;

        // solve the task
        Vector &vel=portMotors.prepare();
        if (mode=="t")
        {
            Matrix G=gains[0]*eye(2,2);
            vel=J.transposed()*G*err;
        }
        else if (mode=="inv")
        {
            Matrix G=gains[1]*eye(2,2);
            vel=pinv(J)*G*err;
        }
        else if (mode=="dls")
        {
            double k=10.0;
            Matrix G=gains[2]*eye(2,2);
            vel=J.transposed()*pinv(J*J.transposed()+k*k*eye(2,2))*G*err;
        }
        else
        {
            vel=zeros(2);
        }

        // deliver the computed velocities to the actuators
        portMotors.writeStrict();

        // provide magnitude of tip Cartesian velocity
        // for visualization purpose 
        portTipVel.prepare()=norm(J*vel)*ones(1);
        portTipVel.writeStrict();

        // send the target for visualization purpose
        portTarget.prepare()=ee_d;
        portTarget.writeStrict();

        return true;
    }

    // this service handles the RPC requests for changing the mode
    // as well as the target positions and gains
    bool respond(const Bottle &command, Bottle &reply)override
    {
        // protect against race conditions
        lock_guard<mutex> lg(mtx);

        string cmd=command.get(0).asString();
        if (cmd=="mode")
        {
            if (command.size()>1)
            {
                string new_mode=command.get(1).asString();
                if ((new_mode=="idle") || (new_mode=="t") ||
                    (new_mode=="inv") || (new_mode=="dls"))
                {
                    mode=new_mode;
                    reply.addString("ok");
                    return true;
                }
            }

            reply.addString("invalid mode");
        }
        else if (cmd=="target")
        {
            if (command.size()>2)
            {
                target[0]=command.get(1).asDouble();
                target[1]=command.get(2).asDouble();
                reply.addString("ok");
                return true;
            }

            reply.addString("invalid target");
        }
        else if (cmd=="planner")
        {
            if (command.size()>1)
            {
                string new_mode=command.get(1).asString();
                if ((new_mode=="on") || (new_mode=="off"))
                {
                    usePlanner=(new_mode=="on");
                    reply.addString("ok");
                    return true;
                }
            }

            reply.addString("invalid mode");
        }
        else if (cmd=="gain")
        {
            if (command.size()>1)
            {
                double g=command.get(1).asDouble();
                if (mode=="t")
                    gains[0]=g;
                else if (mode=="inv")
                    gains[1]=g;
                else if (mode=="dls")
                    gains[2]=g;

                reply.addString("ok");
                return true;
            }

            reply.addString("invalid gain");
        }
        else
            reply.addString("invalid command");

        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Controller controller;
    return controller.runModule(rf);
}


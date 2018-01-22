// Tutorial on Inverse Kinematics.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/pids.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


cv::Point repoint(const ImageOf<PixelRgb> &img, const Vector &p)
{
    return cv::Point((int)(img.width()/2.0+p[0]),
                     (int)(img.height()/2.0-p[1]));
}


class Link
{
    double length;
    vector<Vector> points;

public:
    Link(const double length_) :
       length(length_), points(6,Vector(3,1.0))
    {
        double height=6;

        points[0][0]=0.0;
        points[0][1]=0.0;

        points[1][0]=0.0;
        points[1][1]=height/2.0;

        points[2][0]=length;
        points[2][1]=height/2.0;

        points[3][0]=length;
        points[3][1]=0.0;

        points[4][0]=length;
        points[4][1]=-height/2.0;

        points[5][0]=0.0;
        points[5][1]=-height/2.0;
    }

    Matrix draw(const Matrix &H, const double joint, ImageOf<PixelRgb> &img) const
    {
        double c=cos(joint);
        double s=sin(joint);

        Matrix H_=eye(3,3);
        H_(0,0)=c; H_(0,1)=-s;
        H_(1,0)=s; H_(1,1)=c;
        H_=H*H_;

        vector<cv::Point> pts;
        for (auto &point:points)
        {
            Vector p=H_*point;
            pts.push_back(repoint(img,p));
        }
        vector<vector<cv::Point>> poly(1,pts);

        cv::Mat imgMat=cv::cvarrToMat(img.getIplImage());
        cv::fillPoly(imgMat,poly,cv::Scalar(96,176,224));
        cv::circle(imgMat,pts[0],4,cv::Scalar(0,255,0),CV_FILLED);
        cv::circle(imgMat,pts[3],4,cv::Scalar(0,255,0),CV_FILLED);

        Matrix T=eye(3,3); T(0,2)=length;
        return H_*T;
    }
};


class Robot
{
    vector<Link> links;
    Integrator *joints;

public:
    Robot(const int n, const double length, const double Ts)
    {
        for (int i=0; i<n; i++)
            links.push_back(Link(length));
        joints=new Integrator(Ts,Vector(n,0.0));
    }

    void move(const Vector &velocity, ImageOf<PixelRgb> &img)
    {
        Matrix H=eye(3,3);
        joints->integrate(velocity);
        for (int i=0; i<links.size(); i++)
            H=links[i].draw(H,joints->get()[i],img);
    }

    Vector getJoints() const
    {
        return joints->get();
    }

    virtual ~Robot()
    {
        delete joints;
    }
};


class RobotModule : public RFModule
{
    Robot *robot;

    BufferedPort<ImageOf<PixelRgb>> portEnvironment;
    BufferedPort<Bottle> portMotors;
    BufferedPort<Bottle> portEncoders;
    BufferedPort<Bottle> portTarget;

    Vector velocity;
    Vector target;
    int env_edge;

public:
    bool configure(ResourceFinder &rf)override
    {
        int dof=rf.check("dof",Value(2)).asInt();
        double link_length=rf.check("link-length",Value(100.0)).asDouble();
        env_edge=rf.check("environment-edge",Value(500)).asInt();

        robot=new Robot(dof,link_length,getPeriod());

        portEnvironment.open("/tutorial_inverse-kinematics-robot/environment:o");
        portMotors.open("/tutorial_inverse-kinematics-robot/motors:i");
        portEncoders.open("/tutorial_inverse-kinematics-robot/encoders:o");
        portTarget.open("/tutorial_inverse-kinematics-robot/target:i");

        velocity.resize(dof,0.0);
        target.resize(2,0.0);
        return true;
    }

    bool close()override
    {
        delete robot;

        portEnvironment.close();
        portMotors.close();
        portEncoders.close();
        portTarget.close();

        return true;
    }

    double getPeriod()override
    {
        return 0.01;
    }

    bool updateModule()override
    {
        if (Bottle *vel=portMotors.read(false))
            if (vel->size()>=(int)velocity.length())
                for (int i=0; i<vel->size(); i++)
                    velocity[i]=vel->get(i).asDouble();

        if (Bottle *tar=portTarget.read(false))
        {
            if (tar->size()>=2)
            {
                target[0]=tar->get(0).asDouble();
                target[1]=tar->get(1).asDouble();
            }
        }

        ImageOf<PixelRgb> &env=portEnvironment.prepare();
        env.resize(env_edge,env_edge); env.zero();

        cv::Mat imgMat=cv::cvarrToMat(env.getIplImage());
        cv::circle(imgMat,repoint(env,target),5,cv::Scalar(255,0,0),CV_FILLED);

        robot->move(velocity,env);

        Vector joints=robot->getJoints();
        Bottle &encoders=portEncoders.prepare();
        encoders.clear();

        for (int i=0; i<(int)joints.length(); i++)
            encoders.addDouble(joints[i]);

        portEnvironment.writeStrict();
        portEncoders.writeStrict();

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

    RobotModule robot;
    return robot.runModule(rf);
}

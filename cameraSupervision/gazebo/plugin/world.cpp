/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include <boost/bind.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

namespace gazebo {

/******************************************************************************/
class ModelMover : public gazebo::ModelPlugin
{
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr renderer_connection;
    yarp::os::BufferedPort<yarp::os::Bottle> inPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;
    ignition::math::Pose3d starting_pos;
    yarp::sig::Matrix chessboard_frame;

    /**************************************************************************/
    void onWorldFrame()
    {
        if (auto* b = inPort.read(false))
        {
            if (b->size() >= 6)
            {
                if (model->GetJoint("fixed_to_ground"))
                {
                    if (model->RemoveJoint("fixed_to_ground"))
                    {
//                        yInfo() << "Removed fixed_to_ground joint";
                    }
                }

                const auto x = b->get(0).asFloat64();
                const auto y = b->get(1).asFloat64();
                const auto z = b->get(2).asFloat64();
                const auto roll = b->get(3).asFloat64();
                const auto pitch = b->get(4).asFloat64();
                const auto yaw = b->get(5).asFloat64();
                yarp::sig::Matrix T0 = yarp::math::zeros(4,4);
                yarp::sig::Vector t0{x,y,z,1.0};
                yarp::sig::Vector rpy0{roll,pitch,yaw};
                T0.setSubmatrix(yarp::math::rpy2dcm(rpy0),0,0);
                T0.setCol(3,t0);
                yarp::sig::Matrix T1=chessboard_frame*T0;
                yarp::sig::Vector t1=T1.getCol(3);
                yarp::sig::Vector rpy1=yarp::math::dcm2rpy(T1);
                ignition::math::Pose3d new_pose(t1[0], t1[1], t1[2], rpy1[0], rpy1[1], rpy1[2]);
//                yInfo() << "New pose:" << t1.toString() << rpy1.toString();
                model->SetWorldPose(new_pose);

                std::string model_name = model->GetName();
                physics::LinkPtr child = model->GetLink(model_name+"::marker");
                physics::LinkPtr parent = model->GetLink("world");
                if (child || parent)
                {
                    if (model->CreateJoint("fixed_to_ground", "fixed", parent, child))
                    {
//                        yInfo() << "Added fixed_to_ground joint";
                    }
                }
            }
        }

        if (outPort.getOutputCount() > 0)
        {
            yarp::os::Bottle &bTransf=outPort.prepare();
            bTransf.clear();
            bTransf.addList().read(chessboard_frame);
            outPort.write();
        }
    }

public:
    /**************************************************************************/
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr)
    {
        this->model = model;
        auto model_sdf = model->GetSDF();
        if( model_sdf->HasElement("pose") )
        {
            starting_pos = model_sdf->Get<ignition::math::Pose3d>("pose");
        }
        else
        {
            starting_pos = ignition::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        chessboard_frame=yarp::math::zeros(4,4);
        yarp::sig::Vector transl{starting_pos.X(),starting_pos.Y(),starting_pos.Z(),1.0};
        yarp::sig::Vector rpy{starting_pos.Roll(),starting_pos.Pitch(),starting_pos.Yaw()};
        chessboard_frame.setSubmatrix(yarp::math::rpy2dcm(rpy),0,0);
        chessboard_frame.setCol(3,transl);

        inPort.open("/" + model->GetName() + "/mover:i");
        outPort.open("/" + model->GetName() + "/mover:o");

        auto bind = boost::bind(&ModelMover::onWorldFrame, this);
        renderer_connection = gazebo::event::Events::ConnectWorldUpdateBegin(bind);
    }

    /**************************************************************************/
    virtual ~ModelMover()
    {
        if (!inPort.isClosed())
        {
            inPort.close();
        }
        if (!outPort.isClosed())
        {
            outPort.close();
        }
    }
};

}

GZ_REGISTER_MODEL_PLUGIN(gazebo::ModelMover)

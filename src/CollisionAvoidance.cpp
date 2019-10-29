#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <string>
//#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include "Vector3.h"
#include "Definitions.h"
#include "Agent.h"
#include "KdTree.h"
#include "headers.h"
#include "settings.h"

//for logging
#include <iostream>
#include <fstream>


namespace gazebo
{
class CollisionAvoidance : public ModelPlugin
{
    ignition::math::Vector3d final_position;
    ignition::math::Vector3d actual_position;
    ignition::math::Vector3d prev_position;
    ignition::math::Pose3<double> pose;
    std::vector<ORCA::Agent> agents;
    std::vector<ORCA::Agent *> agents_pntr;
    std::vector<bool> sec5; //For start all the drones togheter
    std::string name;
    int n, amount, server_fd;
    ORCA::KdTree tree;
    clock_t tStart;
    bool  CA= true; //CollisionAvoidance (CA) se 1 il collision avoidance e' attivo se 0 non lo e'
    bool stopped = true;
    bool first = true;
    double actual_trajectory =0;
    std::ofstream myFile;
    common::Time execution_time = 0;

    void send_to_all(Message *m, int amount)
    {
        int n, len;
        for (int i = 0; i < amount; i++)
        {
            if (i != m->src)
            {
                struct sockaddr_in *server = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
                int socketfd = client_init("127.0.0.1", 7000 + i, server);
                int ret = client_send(socketfd, server, m);
                if (ret < 0)
                    printf("%s\n", strerror(errno));
                close(socketfd);
                free(server);
            }
        }
    }

    void receive(int server_fd, int amount)
    {
        Message *m;

        for (int i = 0; i < amount; i++)
        {
            
            m = server_receive(server_fd);
            if (m)
            {
                agents[(m->src) - 1].id_ = m->src;
                agents[(m->src) - 1].position_[0] = m->x;
                agents[(m->src) - 1].position_[1] = m->y;
                agents[(m->src) - 1].position_[2] = m->z;
                agents[(m->src) - 1].velocity_[0] = m->vx;
                agents[(m->src) - 1].velocity_[1] = m->vy;
                agents[(m->src) - 1].velocity_[2] = m->vz;
                sec5[(m->src) - 1] = m->id;
            }
        }
    }

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;

        if (_sdf->HasElement("final_position"))
            final_position = _sdf->Get<ignition::math::Vector3d>("final_position");
        else
            final_position = ignition::math::Vector3d(0, 0, 0);

        

        //Setup of this drone Agent
        name = this->model->GetName();
        n = std::stoi(name.substr(6));

        std::string world_name = this->model->GetWorld()->Name();
        //std::cout<<"World name = "<<world_name<<std::endl;
        TotalNumberDrones = std::stoi(world_name.substr(6));
        //std::cout<<"Total Number of Drones: "<<TotalNumberDrones<<std::endl;


        //initialize vectors of agents and agents_pntr
        agents.resize(TotalNumberDrones);
        agents_pntr.resize(TotalNumberDrones);
        
        agents[n - 1].id_ = n;
        agents[n - 1].maxNeighbors_ = TotalNumberDrones;
        agents[n - 1].maxSpeed_ = MAX_VELOCITY;
        agents[n - 1].neighborDist_ = 1;
        agents[n - 1].radius_ = 0.4;
        agents[n - 1].timeHorizon_ = 50.0f;
        agents[n - 1].tree_ = &tree;
        agents[n - 1].velocity_[0] = 0;
        agents[n - 1].velocity_[1] = 0;
        agents[n - 1].velocity_[2] = 0;
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CollisionAvoidance::OnUpdate, this));

        for (int i = 0; i < TotalNumberDrones; i++)
        {
            agents_pntr[i] = &agents[i];
        }
        tree.setAgents(agents_pntr);
        //Setup of the Server
        amount = TotalNumberDrones + 2;
        server_fd = server_init(7000 + n);
        //per il logging
        std::string file = "./log/"+name +"_testX_ORCA.txt";
        myFile.open(file,std::ios::app);

        //per la sincronizzazione
        for (int i = 0; i <= TotalNumberDrones; i++)
        {
            sec5.push_back(false);
        }
        tStart = clock();
        
    }

    // Called by the world update start event
public:
    void OnUpdate()
    {
        if (first){
            first = false;
            actual_position = this->model->WorldPose().Pos();
            myFile<<"Traiettoria originiale: "<< actual_position.Distance(final_position) <<std::endl;
            execution_time = this->model->GetWorld()->SimTime();
        }
        if (actual_position.Distance(final_position) > 0.2)
        {
            

            // 0.0 - UPDATE MY POS and VEL
            pose = this->model->WorldPose();
            actual_position = pose.Pos();
            //myFile<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()<<","<<n<<std::endl;
            agents[n - 1].position_[0] = pose.Pos().X();
            agents[n - 1].position_[1] = pose.Pos().Y();
            agents[n - 1].position_[2] = pose.Pos().Z();

            //Calcolo il delta spazio ad ogni delta t e li sommo
            double ds = (actual_position.Distance(prev_position));
            actual_trajectory += ds;
            //std::cout << "Delta spazio = "<<ds <<std::endl;
            prev_position = actual_position;



            bool go = true;
            for (int i = 0; i < TotalNumberDrones; i++)
            {
               if (!sec5[i])
                   go = false;
            }
            ignition::math::Vector3d velocity;
            if (go)
            {
                velocity = (final_position - actual_position).Normalize()*50;
            }
            else
                velocity = final_position * 0;
            agents[n - 1].prefVelocity_[0] = velocity.X();
            agents[n - 1].prefVelocity_[1] = velocity.Y();
            agents[n - 1].prefVelocity_[2] = velocity.Z();
    
            // 0.1 - SEND POS AND VEL
            Message m;
            m.src = n;
            if (((double)(clock() - tStart) / CLOCKS_PER_SEC) > 0.3)
            {
                sec5[n - 1] = true;
                m.id = 1;
            }
            else
            {
               m.id = 0;
            }
            m.x = pose.Pos().X();
            m.y = pose.Pos().Y();
            m.z = pose.Pos().Z();
            m.vx = agents[n - 1].velocity_[0];
            m.vy = agents[n - 1].velocity_[1];
            m.vz = agents[n - 1].velocity_[2];
            send_to_all(&m, amount);
            
            // 0.2 - UPDATE OTHERS POS AND VEL
            receive(server_fd, amount);

            tree.buildAgentTree();

            // 1 - COMPUTE NEIGHBORS

            agents[n - 1].computeNeighbors();

            // 2 - COMPUTE VELOCITIES

            agents[n - 1].computeNewVelocity();

            // 3 - UPDATE
            
            agents[n - 1].update();
            ignition::math::Vector3d newVelocity(agents[n - 1].velocity_[0], agents[n - 1].velocity_[1], agents[n - 1].velocity_[2]);
            if(CA)
                this->model->SetLinearVel(newVelocity);
            else 
                this->model->SetLinearVel(velocity); 
            
        }
        else
        {
            if (stopped) {
                stopped = false;
                myFile<<"Final trajectory: "<< actual_trajectory<<std::endl;
                execution_time = this->model->GetWorld()->SimTime() - execution_time;
                
                myFile<<"Tempo di esecuzione: "<< execution_time.Double() <<std::endl<<"____________________________________"<<std::endl;
                myFile.close();
                std::cout << "Drone "<<name <<" arrivato!"<<std::endl;
            }
            this->model->SetLinearVel(final_position*0);
            
        }
        
        
    }

    // Pointer to the model
private:
    physics::ModelPtr model;

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CollisionAvoidance)
} // namespace gazebo
// Artificial potential field (Bounded)
// This file implement the collision avoidance system based on the artificial potential field bounded.
// Bounded because the force is felt only inside a specidic radius from the center of the drone

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include "headers.h"
#include "Neighbour.h"
#include "settings.h"
#include <cmath>
//for logging
#include <iostream>
#include <fstream>



namespace gazebo
{
class BAPF : public ModelPlugin
{
    ignition::math::Vector3d final_position;
    ignition::math::Vector3d actual_position;
    neighbour me;
    ignition::math::Pose3<double> pose;
    std::vector<Neighbour> agents ;
    std::vector<bool> sec5; //For start all the drones togheter
    std::string name;
    int n, amount, server_fd;
    
    clock_t tStart;
    bool  CA= true; //CollisionAvoidance (CA) se 1 il collision avoidance e' attivo se 0 non lo e'
    bool swap = true;
    gazebo::common::Time prevTime;

    std::ofstream myFile;

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
                ignition::math::Vector3d positionReceived( m->x , m->y , m->z );
                double distance = actual_position.Distance(positionReceived);
                if( distance <= radius){
                    Neighbour tmp;
                    tmp.id_ = m->src;
                    tmp.x = positionReceived.X();
                    tmp.y = positionReceived.Y();
                    tmp.z = positionReceived.Z();
                    tmp.vx = m->vx;
                    tmp.vy = m->vy;
                    tmp.vz = m->vz;
                    sec5[(m->src) - 1] = m->id;
                    agents.push_back(tmp);
                    //std::cout<<"Pushed back "<< tmp.id_<<std::endl;
                }

            }
        }
    }

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;
        prevTime = this->model->GetWorld()->RealTime();
        if (_sdf->HasElement("final_position"))
            final_position = _sdf->Get<ignition::math::Vector3d>("final_position");
        else
            final_position = ignition::math::Vector3d(0, 0, 0);

        std::string world_name = this->model->GetWorld()->Name();
        //std::cout<<"World name = "<<world_name<<std::endl;
        TotalNumberDrones = std::stoi(world_name.substr(6));
        //std::cout<<"Total Number of Drones: "<<TotalNumberDrones<<std::endl;
        //initialize vectors of agents and agents_pntr
        agents.resize(TotalNumberDrones);

        //Setup of this drone Agent
        name = this->model->GetName();
        n = std::stoi(name.substr(6));
        //std::cout<< "numero "<< n <<std::endl;
        me.id_ = n;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&BAPF::OnUpdate, this));

        //Setup of the Server
        amount = TotalNumberDrones + 2;
        server_fd = server_init(7000 + n);
        
        //per il logging
        //std::string file = "/home/matteo/Desktop/Results/" + name +" testX_CA0.csv";
        //myFile.open(file);

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
        if (actual_position.Distance(final_position) > 0.3)
        {
            
            // 0.0 - UPDATE MY POS and VEL
            pose = this->model->WorldPose();
            actual_position = pose.Pos();
            //myFile<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()<<","<<n<<std::endl;
            me.x = pose.Pos().X();
            me.y = pose.Pos().Y();
            me.z = pose.Pos().Z();
            
            //syncronization for start
            bool go = true;
            //for (int i = 0; i < TotalNumberDrones; i++)
            //{
            //   if (!sec5[i])
            //       go = false;
            //}
            
            ignition::math::Vector3d velocity;
            if (go)
            {
                velocity = (final_position - actual_position).Normalize()*50;
                me.vx = velocity.X();
                me.vy = velocity.Y();
                me.vz = velocity.Z();
            }
            else
                velocity = final_position * 0;
    
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
            m.x = me.x;
            m.y = me.y;
            m.z = me.z;
            m.vx = me.vx;
            m.vy = me.vy;
            m.vz = me.vz;
            send_to_all(&m, amount);
            
            // 0.2 - UPDATE OTHERS POS AND VEL
            receive(server_fd, amount);

            //COLLISION AVOIDANCE ALGORITHM HERE
            
            ignition::math::Vector3d me_position(me.x,me.y,me.z);
            ignition::math::Vector3d me_velocity(me.vx,me.vy,me.vz);
            ignition::math::Vector3d repulsion_force(0,0,0);
            //Se non ho vicini vai alla massima velocita
            bool vai_easy=false;
            if(agents.empty())
                vai_easy=true;

            while(!agents.empty()){
                auto agent = agents.back();
                ignition::math::Vector3d agent_position(agent.x,agent.y,agent.z);
                double d = me_position.Distance(agent_position); //aggiungere raggio del drone 
                //repulsion_force += k*(radius/d)*(me_position-agent_position).Normalize();
                repulsion_force += (500*(mass*mass)/(d*d))*(me_position-agent_position).Normalize();
                agents.pop_back();
            }
            double d = me_position.Distance(final_position);
            ignition::math::Vector3d attractive_force = -(k*(mass*2000)/(d*d))*(me_position-final_position).Normalize();
            repulsion_force += attractive_force;
            // 3 - UPDATE  

            // Time delta
            //std::cout<< name <<" repulsion force: "<< repulsion_force << "\n";
            double dt = (this->model->GetWorld()->SimTime() - prevTime).Double();
            std::cout << "Delta t = "<<dt <<std::endl;
            prevTime = this->model->GetWorld()->SimTime();
            ignition::math::Vector3d repulsion_velocity = (repulsion_force/mass)*dt;
            //std::cout<< name <<" repulsion velocity: "<< repulsion_velocity << "\n";
            ignition::math::Vector3d maxVelocity = MAX_VELOCITY*(final_position-actual_position).Normalize();
            //maxVelocity.X(maxVelocity.X()*200);
            //maxVelocity.Y(maxVelocity.Y()*200);
            ignition::math::Vector3d newVelocity;
            if (vai_easy){
                newVelocity=maxVelocity;
            }else{
                newVelocity = repulsion_velocity ;
                //ignition::math::Vector3d newVelocity = repulsion_velocity.Length() > maxVelocity.Length() ? maxVelocity : repulsion_velocity ;
            }
            //std::cout<< name <<" new velocity: "<< newVelocity << "\n";

            if(CA)
                this->model->SetLinearVel(newVelocity);
            else 
                this->model->SetLinearVel(velocity); 
            
        }
        else
        {
            this->model->SetLinearVel(final_position*0);
            std::cout <<"Drone_"<<name<<" ARRIVATO\n";
            // if (swap){
            //     srand (time(NULL));
            //     int rCirconferenza = 5;
            //     float alpha = (std::rand() % 360)*M_PI/180;
            //     //std::cout<<name<<" angolo "<< alpha << "\n "<< "X: "<<rCirconferenza*std::cos(alpha)<<"\nY: "<<rCirconferenza*std::sin(alpha)<<std::endl; 
            //     final_position.X(rCirconferenza*std::cos(alpha));
            //     final_position.Y(rCirconferenza*std::sin(alpha));
            //     final_position.Z(5);
            //     //std::cout<<name <<" final position: "<< final_position<<std::endl;
            //     swap=false;
            // }else
            // {
            //     final_position.X(0);
            //     final_position.Y(0);
            //     final_position.Z(5);
            //     //std::cout<<name <<" final position: "<< final_position<<std::endl;
            //     swap=true;
            // }
            
            //myFile.close();
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
GZ_REGISTER_MODEL_PLUGIN(BAPF)
} // namespace gazebo
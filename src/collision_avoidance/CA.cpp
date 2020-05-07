#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


//#include <ros/ros.h>
#include <string>
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
#include "Neighbour.h"
#include <cmath>



//for logging
#include <iostream>
#include <fstream>

namespace gazebo
{
    class CA : public ModelPlugin
    {

        ignition::math::Vector3d final_position;
        ignition::math::Vector3d actual_position;
        neighbour me;
        ignition::math::Vector3d prev_position;
        ignition::math::Pose3<double> pose;
        std::vector<Neighbour> agents ;
        std::vector<ORCA::Agent> agents_; //for ORCA
        std::vector<ORCA::Agent *> agents_pntr; //for ORCA
        ORCA::KdTree tree;

        std::vector<bool> sec5; //For start all the drones togheter
        std::string name;
        int n, amount, server_fd;
        clock_t tStart;
        gazebo::common::Time prevTime;

        bool  CA_activated = true; //CollisionAvoidance (CA) se 1 il collision avoidance e' attivo se 0 non lo e'

        bool swap = true;
        int test = 1;
        enum class algo{ORCA,BAPF,EAPF};
        algo al = algo::ORCA;
        //for the logging
        bool stopped = true;
        bool first = true;
        double actual_trajectory = 0;
        std::ofstream myFile;
        common::Time execution_time =0;


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
                	if(al == algo::ORCA){
                		    agents_[(m->src) - 1].id_ = m->src;
                		    agents_[(m->src) - 1].position_[0] = m->x;
                		    agents_[(m->src) - 1].position_[1] = m->y;
                		    agents_[(m->src) - 1].position_[2] = m->z;
                		    agents_[(m->src) - 1].velocity_[0] = m->vx;
                		    agents_[(m->src) - 1].velocity_[1] = m->vy;
                		    agents_[(m->src) - 1].velocity_[2] = m->vz;
                		    sec5[(m->src) - 1] = m->id;
                	}else{
                		ignition::math::Vector3d positionReceived( m->x, m->y, m->z );
                		double distance = actual_position.Distance(positionReceived);
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
            if (_sdf->HasElement("algorithm")){
            	std::string algor = _sdf->Get<std::string>("algorithm");
            	if(algor == "ORCA"){
            		al = algo::ORCA;
            	} else if (algor=="BAPF")
            	{
            		al = algo::BAPF;
            	} else if (algor== "EAPF")
            	{
            		al = algo::EAPF;
            	} else
            	{
            		al = algo::ORCA;
            	}
            }
            else
                al = algo::ORCA;
            if (_sdf->HasElement("test")){
            	test= _sdf->Get<int>("test");
			}
            //Setup of this drone Agent
            name = this->model->GetName();
            n = std::stoi(name.substr(6));
            me.id_ = n;

            std::string world_name = this->model->GetWorld()->Name();
            TotalNumberDrones = std::stoi(world_name.substr(6));

            //initialize vectors of agents and agents_pntr
            agents.resize(TotalNumberDrones);

            if(al == algo::ORCA)
            {
            	agents_.resize(TotalNumberDrones);
            	agents_pntr.resize(TotalNumberDrones);
                agents_[n - 1].id_ = n;
                agents_[n - 1].maxNeighbors_ = TotalNumberDrones;
                agents_[n - 1].maxSpeed_ = MAX_VELOCITY;
                agents_[n - 1].neighborDist_ = 1;
                agents_[n - 1].radius_ = 0.4;
                agents_[n - 1].timeHorizon_ = 50.0f;
                agents_[n - 1].tree_ = &tree;
                agents_[n - 1].velocity_[0] = 0;
                agents_[n - 1].velocity_[1] = 0;
                agents_[n - 1].velocity_[2] = 0;

                for (int i = 0; i < TotalNumberDrones; i++)
                {
                    agents_pntr[i] = &agents_[i];
                }
                tree.setAgents(agents_pntr);
            }


            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                         std::bind(&CA::OnUpdate, this));

            //Setup of the Server
            amount = TotalNumberDrones + 2;
            server_fd = server_init(7000 + n);

            //per la sincronizzazione
            for (int i = 0; i <= TotalNumberDrones; i++)
            {
                sec5.push_back(false);
            }
            tStart = clock();

            //per il logging
            std::string file = "./log" + name + "_testX_EAPF.txt";
            myFile.open(file, std::ios::app);
        }

        // Called by the world update start event
    public:
        void OnUpdate()
        {
            if (first)
            {
                first = false;
                actual_position = this->model->WorldPose().Pos();
                myFile << "Traiettoria originiale: " << actual_position.Distance(final_position) << std::endl;
                execution_time = this->model->GetWorld()->SimTime();
            }
            if (actual_position.Distance(final_position) > 0.5)
            {

                // 0.0 - UPDATE MY POS and VEL
                pose = this->model->WorldPose();
                actual_position = pose.Pos();
                //myFile<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()<<","<<n<<std::endl;
                if(al == algo::ORCA)
                {
                    agents_[n - 1].position_[0] = pose.Pos().X();
                    agents_[n - 1].position_[1] = pose.Pos().Y();
                    agents_[n - 1].position_[2] = pose.Pos().Z();
                }
                else
                {
                    me.x = pose.Pos().X();
                    me.y = pose.Pos().Y();
                    me.z = pose.Pos().Z();
                }



                //Calcolo il delta spazio ad ogni delta t e li sommo
                double ds = (actual_position.Distance(prev_position));
                actual_trajectory += ds;
                //std::cout << "Delta spazio = "<<ds <<std::endl;
                prev_position = actual_position;

                //syncronization for start
                bool go = true;
                for (int i = 0; i < TotalNumberDrones; i++)
                {
                    if (!sec5[i])
                        go = false;
                }

                ignition::math::Vector3d velocity;
                if (go)
                {
                    velocity = (final_position - actual_position).Normalize() * 50;
                    
                    if(al != algo::ORCA)
                    {
                        me.vx = velocity.X();
                        me.vy = velocity.Y();
                        me.vz = velocity.Z();
                    }
                }
                else
                    velocity = final_position * 0;

                if(al == algo::ORCA)
                    {
                        agents_[n - 1].prefVelocity_[0] = velocity.X();
                        agents_[n - 1].prefVelocity_[1] = velocity.Y();
                        agents_[n - 1].prefVelocity_[2] = velocity.Z();
                    }

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

                if(al == algo::ORCA)
                {
                    m.x = pose.Pos().X();
                    m.y = pose.Pos().Y();
                    m.z = pose.Pos().Z();
                    m.vx = agents_[n - 1].velocity_[0];
                    m.vy = agents_[n - 1].velocity_[1];
                    m.vz = agents_[n - 1].velocity_[2];
                }
                else
                {
                    m.x = me.x;
                    m.y = me.y;
                    m.z = me.z;
                    m.vx = me.vx;
                    m.vy = me.vy;
                    m.vz = me.vz;
                }

                send_to_all(&m, amount);

                // 0.2 - UPDATE OTHERS POS AND VEL
                receive(server_fd, amount);
                ignition::math::Vector3d newVelocity;
                switch(al)
                {
	                case algo::ORCA:
	                {
	                   	tree.buildAgentTree();

	                    // 1 - COMPUTE NEIGHBORS

	                    agents_[n - 1].computeNeighbors();

	                    // 2 - COMPUTE VELOCITIES

	                    agents_[n - 1].computeNewVelocity();

	                    // 3 - UPDATE

	                    agents_[n - 1].update();
	                    newVelocity = ignition::math::Vector3d(agents_[n - 1].velocity_[0], agents_[n - 1].velocity_[1], agents_[n - 1].velocity_[2]);
	                    break;
	                }
	                case algo::BAPF:
	                {
	                	// std::cout << "sadfdsafsafas\n";

	                    ignition::math::Vector3d me_position(me.x, me.y, me.z);
	                    ignition::math::Vector3d me_velocity(me.vx, me.vy, me.vz);
	                    ignition::math::Vector3d repulsion_force(0, 0, 0);
	                    //Se non ho vicini vai alla massima velocita
	                    bool vai_easy = false;
	                    if(agents.empty())
	                        vai_easy = true;
	                    int radius = 5;
	                    while(!agents.empty())
	                    {
	                        auto agent = agents.back();
	                        ignition::math::Vector3d agent_position(agent.x, agent.y, agent.z);
	                        //float r3 = 0.1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(0.5-0.1)));
	                        double d = me_position.Distance(agent_position); //aggiungere raggio del drone
	                        //repulsion_force += k*(radius/d)*(me_position-agent_position).Normalize();
	                        repulsion_force += (500 * (mass * mass) / (d * d)) * (me_position - agent_position);
	                        agents.pop_back();
	                    }
	                    double d = me_position.Distance(final_position);
	                    ignition::math::Vector3d attractive_force = -(k * (mass * 2000) / (d * d)) * (me_position - final_position);
	                    repulsion_force += attractive_force;
	                    // 3 - UPDATE

	                    // Time delta
	                    //std::cout<< name <<" repulsion force: "<< repulsion_force << "\n";
	                    double dt = (this->model->GetWorld()->SimTime() - prevTime).Double();
	                    prevTime = this->model->GetWorld()->SimTime();
	                    ignition::math::Vector3d repulsion_velocity = (repulsion_force / mass) * dt;
	                    //std::cout<< name <<" repulsion velocity: "<< repulsion_velocity << "\n";
	                    ignition::math::Vector3d maxVelocity = MAX_VELOCITY * (final_position - actual_position).Normalize();
	                    //maxVelocity.X(maxVelocity.X()*200);
	                    //maxVelocity.Y(maxVelocity.Y()*200);
	                    newVelocity;
	                    if (vai_easy)
	                    {
	                        newVelocity = maxVelocity;
	                    }
	                    else
	                    {
	                        newVelocity = repulsion_velocity ;
	                        //ignition::math::Vector3d newVelocity = repulsion_velocity.Length() > maxVelocity.Length() ? maxVelocity : repulsion_velocity ;
	                    }
	                    //std::cout<< name <<" new velocity: "<< newVelocity << "\n";
	                    break;
	                }
	                case algo::EAPF:
	                {
	                    ignition::math::Vector3d me_position(me.x, me.y, me.z);
	                    ignition::math::Vector3d me_velocity(me.vx, me.vy, me.vz);
	                    ignition::math::Vector3d repulsion_force(0, 0, 0);
	                    while(!agents.empty())
	                    {
	                        auto agent = agents.back();
	                        ignition::math::Vector3d agent_position(agent.x, agent.y, agent.z);
	                        double d = me_position.Distance(agent_position); //aggiungere raggio del drone

	                        //std::cout<<(radius/d)<<"\n";
	                        //repulsion_force += k*(radius/d)*(me_position-agent_position).Normalize();
	                        repulsion_force += (100 * (mass * mass) / (d * d)) * (me_position - agent_position).Normalize();
	                        agents.pop_back();
	                    }
	                    double d = me_position.Distance(final_position);
	                    ignition::math::Vector3d attractive_force = -(k * (mass * 1000) / (d * d)) * (me_position - final_position).Normalize();
	                    repulsion_force += attractive_force;
	                    // 3 - UPDATE

	                    // Time delta
	                    //std::cout<< name <<" repulsion force: "<< repulsion_force << "\n";
	                    double dt = (this->model->GetWorld()->RealTime() - prevTime).Double();
	                    //std::cout << "Delta t = "<<dt <<std::endl;
	                    prevTime = this->model->GetWorld()->RealTime();
	                    ignition::math::Vector3d repulsion_velocity = (repulsion_force / mass) * dt;
	                    //std::cout<< name <<" repulsion velocity: "<< repulsion_velocity << "\n";
	                    ignition::math::Vector3d maxVelocity = MAX_VELOCITY * (final_position - actual_position).Normalize();
	                    newVelocity = repulsion_velocity.Length() > maxVelocity.Length() ? maxVelocity : repulsion_velocity ;
	                    //std::cout<< name <<" new velocity: "<< newVelocity << "\n";
	                    break;
	                }
                }

                //COLLISION AVOIDANCE ALGORITHM HERE



                if(CA_activated)
                    this->model->SetLinearVel(newVelocity);
                else
                    this->model->SetLinearVel(velocity);

            }
            else
            {
            	if(test == 3){
            		struct timespec ts;
        		    clock_gettime(CLOCK_MONOTONIC, &ts);

        		    /* using nano-seconds instead of seconds */
        		    srand((time_t)ts.tv_nsec);
            		int rCirconferenza = (std::rand() % 10);
            		float theta = (std::rand() % 360)*M_PI/180;
            		float gamma = (std::rand() % 360)*M_PI/180;
            		final_position.X(rCirconferenza*std::sin(theta)*std::sin(gamma));
            		final_position.Y(rCirconferenza*std::cos(theta));
            		final_position.Z(10+(rCirconferenza*std::sin(theta)*std::cos(gamma)));

            		if (execution_time.Double() >200)
            		{
            		    stopped = false;
            		    myFile << "Final trajectory: " << actual_trajectory << std::endl;
            		    execution_time = this->model->GetWorld()->SimTime() - execution_time;
            		    myFile << "Tempo di esecuzione: " << execution_time.Double() << std::endl;
            		    myFile.close();
            		    std::cout << "Drone " << name << " arrivato!" << std::endl;
            		}
            		this->model->SetLinearVel(final_position * 0);

            	}else{
            		if (stopped)
            		{
            		    stopped = false;
            		    myFile << "Final trajectory: " << actual_trajectory << std::endl;
            		    execution_time = this->model->GetWorld()->SimTime() - execution_time;
            		    myFile << "Tempo di esecuzione: " << execution_time.Double() << std::endl;
            		    myFile.close();
            		    std::cout << "Drone " << name << " arrivato!" << std::endl;
            		}
            		this->model->SetLinearVel(final_position * 0);
            	}         
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
    GZ_REGISTER_MODEL_PLUGIN(CA)
} // namespace gazebo

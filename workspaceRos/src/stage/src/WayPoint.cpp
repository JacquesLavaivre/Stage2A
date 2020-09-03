#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <stdlib.h> 
#include <cmath>
#include "tf/tf.h"
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <stack>

using namespace std;

#define PI 3.14159265

//Déclaration des variables globales

vector<double> wp1 {0.475,1.675};
vector<double> wp2 {1.925,-0.825};
vector<double> wp3  {-1.7, -1.725};


vector<double>ob1 {-0.475, 1.45};
vector<double>ob2 {1.225, 1};
vector<double>ob3 {0.4, 0.5};
vector<double>ob4 {1.75, 0.4750};
vector<double>ob5 {1.65, 1.6};
vector<double>ob6 {0.15, -0.75};
vector<double>ob7 {-0.65, -2.025};
stack<vector<double>> pile;

geometry_msgs::Pose poseBruit;

void chatterCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	poseBruit.position.x = msg->position.x;
	poseBruit.position.y = msg->position.y;
	poseBruit.position.z = msg->position.z;

	poseBruit.orientation.x = msg->orientation.x;
	poseBruit.orientation.y = msg->orientation.y;
	poseBruit.orientation.z = msg->orientation.z;
	poseBruit.orientation.w = msg->orientation.w;
}


//Fonction du Filtre de Kalman
double Kalman(double m, double mf, double R, double P, double Q, double K, double H)
{
	//K = P*H/(H*P*+R);
	//mf += K*(m-H*mf);

	//P = (1-K*H)*P+Q;

	return m, P; //mf, P;
}

//Fonction permettant au robot d'avancer
void avancer(geometry_msgs::Twist &msg)
{
	msg.angular.z = 0;
	msg.linear.x = 0.5;
}

//Fonction permettant au robot de s'arrêter
void arreter(geometry_msgs::Twist &msg)
{
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
}

//Fonction permettant de tourner à droite
void tournerdroite(geometry_msgs::Twist &msg)
{
	msg.linear.x = 0;
	msg.angular.z = 0.5;
}

//Fonction permettant de tourner à gauche
void tournergauche(geometry_msgs::Twist &msg)
{
	msg.linear.x = 0;
	msg.angular.z = -1;
}

//Permet de savoir si on se rapproche de l'obstacle ou du waypoint au tour suivant
int rapproche(vector<double> &waypoint, double x, double y, double yaw)
{	
	double dx = x*sin(yaw);
	double dy = y*cos(yaw);

	if(sqrt(pow(x-waypoint[0],2)+pow(y-waypoint[1],2)) > sqrt(pow(x+dx-waypoint[0],2)+pow(y+dy-waypoint[1],2)))
	{
		return 1;
	}

	else
	{
		return 0;
	}
}

// Coprs du progamme
int main(int argc, char **argv){

	ros::init(argc, argv, "WayPoint");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("posBruite", 1000, chatterCallback);

	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Rate loop_rate(10);

	geometry_msgs::Pose poseIni;


	vector<vector<double>> route = {wp1, wp2, wp3};
	vector<vector<double>> obstacle = {ob1, ob2, ob3, ob4, ob5, ob6, ob7};

	geometry_msgs::Twist msg;

	//Initialisation 

	arreter(msg);

	int i = 0;

	vector<double> R = {1, 1, 1, 1, 1, 1, 1};
	vector<double> H = {1, 1, 1, 1, 1, 1, 1};
	vector<double> Q ={10, 10, 10, 10, 10, 10, 10};
	vector<double> P ={0, 0 ,0 , 0, 0, 0, 0};
	vector<double> K ={0, 0, 0, 0, 0, 0, 0};
	vector<double> waypoint = route[i];

	poseIni.position.x = 0;
	poseIni.position.y = 0;
	poseIni.position.z = 0;

	poseIni.orientation.x = 0;
	poseIni.orientation.y = 0;
	poseIni.orientation.z = 0;
	poseIni.orientation.w = 0;

	double t0 = time(NULL);

	double x0 = -1.75;
	double y0 = 1.75;


	int c = 0;

	while (ros::ok()){

		//Permet d'éviter des bugs au lancement du programme
		while(time(NULL) - t0 < 0.5)
		{
			msg.linear.x = 0.01;
			cmd_pub.publish(msg);

		}


		//On détecte si on est près d'un obstacle, si oui on rentre la position de l'obstacle dans la pile

		double d = 1000000;

		for(int j =0; j<=6; j++)
		{	
			
			
			if(sqrt(pow(poseBruit.position.x-obstacle[j][0],2)+pow(poseBruit.position.y-obstacle[j][1],2)) < 0.8)
				{
					if(pile.empty())
					{
						pile.push(obstacle[j]);
						c = j;
						double d = sqrt(pow(poseBruit.position.x-obstacle[j][0],2)+pow(poseBruit.position.y-obstacle[j][1],2));
					}

					else
					{
						if(sqrt(pow(poseBruit.position.x-obstacle[j][0],2)+pow(poseBruit.position.y-obstacle[j][1],2)) < d)
						{	
							pile.pop();
							pile.push(obstacle[j]);
							c = j;
							double d = sqrt(pow(poseBruit.position.x-obstacle[j][0],2)+pow(poseBruit.position.y-obstacle[j][1],2));
						}
					}
				}

		}


		
		if(!pile.empty()) //Si on est près d'un obstacle
		{	
			
			//cout << "Je dois éviter l'obstacle "<< c << " " << endl;

			// On récupère le cap actuel
			double roll, pitch, yaw;
			tf::Quaternion q(poseBruit.orientation.x, poseBruit.orientation.y, poseBruit.orientation.z, poseBruit.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRotation(q);
			m.getRPY(roll, pitch, yaw);

			double headOk = atan((poseBruit.position.y-obstacle[c][1])/(poseBruit.position.x-obstacle[c][0]))*180/PI;

			//cout << "yawb : " << yawb << " yaw :" << yaw*180/PI << " dif :" << yawb-yaw*180/PI<< endl;
			//cout << headOk-yaw*180/PI << endl;

			// L'ensemble des boucles if permet au robot de tourner à 60 degrés dès qu'il détecte l'obstacle et ensuite d'avancer tout droit pour l'esquiver

			if(fabs(headOk-yaw*180/PI)<30)
			{
				if(headOk-yaw*180/PI<0)
				{
					tournerdroite(msg);
				}
				else
				{
					tournergauche(msg);
				}

				//cout << "Je tourne pour éviter l'obstacle" << endl;
			}

			else
			{
				avancer(msg);
				//cout << " J'avance pour éviter l'obstacle" << endl;
			}

			if( sqrt(pow(poseBruit.position.x-obstacle[i][0],2)+pow(poseBruit.position.y-obstacle[i][1],2)) > 0.5)
			{
				pile.pop();
			}
		}



		else // Autrement dit si on est près d'aucun obstacle
		{
			//cout << "Je dois aller au waypoint" << endl;

			if(i <=3) // Si jamais on a pas rejoint tous les waypoints
			{
				if(sqrt(pow(poseBruit.position.x-waypoint[0],2)+pow(poseBruit.position.y-waypoint[1],2)) < 0.2)// Si on est sur un waypoint
				{
					i +=1;
					waypoint = route[i]; // On passe à l'autre waypoint
				}

		
				else // Si on est entrain de rejoindre un waypoint
				{	

					/*filtrage des mesures
					poseIni.position.x, P[0] = Kalman(poseBruit.position.x, poseIni.position.x, R[0], P[0], Q[0], K[0], H[0]);				
					poseIni.position.y, P[1] = Kalman(poseBruit.position.y, poseIni.position.y, R[1], P[1], Q[1], K[1], H[1]);
					poseIni.position.z, P[2] = Kalman(poseBruit.position.z, poseIni.position.z, R[2], P[2], Q[2], K[2], H[2]); 
					poseIni.orientation.x, P[3] = Kalman(poseBruit.orientation.x, poseIni.orientation.x, R[3], P[3], Q[3], K[3], H[3]); 
					poseIni.orientation.y, P[4] = Kalman(poseBruit.orientation.y, poseIni.orientation.y, R[4], P[4], Q[4], K[4], H[4]);
					poseIni.orientation.z, P[5] = Kalman(poseBruit.orientation.z, poseIni.orientation.z, R[5], P[5], Q[5], K[5], H[5]);
					poseIni.orientation.w, P[6] = Kalman(poseBruit.orientation.w, poseIni.orientation.w, R[6], P[6], Q[6], K[6], H[6]);
					*/

					double headOk = atan((poseBruit.position.y-waypoint[1])/(poseBruit.position.x-waypoint[0]))*180/PI; //On calcule le cap à avoir pour rejoindre le waypoint

					// à améliorer, problèmes car l'image de atan est entre -pi/2 et pi/2
					if( waypoint[0] < 0 )
					{
						if(headOk > 0)
						{
							headOk = headOk -180;
						}

						else
						{
							headOk = headOk + 180;
						}

					}

					//On calcul le cap du robot
					double roll, pitch, yaw;
					tf::Quaternion q(poseBruit.orientation.x, poseBruit.orientation.y, poseBruit.orientation.z, poseBruit.orientation.w);
					tf::Matrix3x3 m(q);
					m.getRotation(q);
					m.getRPY(roll, pitch, yaw);


					// On régule le robot en fonction de l'écart au cap voulu
					if(fabs(headOk-yaw*180/PI)>5)
					{
						if(headOk-yaw*180/PI < 0)
						{
							tournergauche(msg);

							//cout << "Je tourne à gauche pour me diriger vers le waypoint"<< endl;
						}

						else
						{
							tournerdroite(msg);

							//cout << "Je tourne à droite pour me diriger vers le waypoint" << endl;
						}
					}
					
					if(fabs(headOk-yaw*180/PI)<=5)
					{
						avancer(msg);

						//cout << "J'avance vers le waypoint"<< endl;
					}
				} 
			}
		}


		if(i>3)//Si jamais on a fait tous les waypoints on s'arrête
		{
			arreter(msg);
		}

		cmd_pub.publish(msg);

		ros::spinOnce();
	}	
		
}




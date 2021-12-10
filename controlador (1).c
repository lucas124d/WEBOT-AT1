#include <stdio.h>
#include <stdlib.h>

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>

#define THETA_THRESHOLD 1 
#define MAX_SPEED 6.28 
#define TANGENSIAL_SPEED 0.12874
#define ROBOT_ROTATIONAL_SPEED 0.772881647
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.587

#define BOX_THRESHOLD 0.35


static WbDeviceTag left_motor, right_motor;
WbNodeRef robot_node;
WbFieldRef trans_field;
WbFieldRef rotac_field;
WbDeviceTag ps[8];
double ps_values[8];
const double *robot_position;	
const double *robot_rotation;

int time_step;

double boxes[9][2] = {
	{-0.25,-0.25},  // Wooden box(0)
	{-0.25,0},     // Wooden box(1)
	{-0.25,0.25}, // Wooden box(2)
	{0, -0.25},     // Wooden box(3)
	{0, 0},        // Wooden box(4)
	{0,0.25},     // Wooden box(5)
	{0.25, -0.25},  // Wooden box(6)
	{0.25,0},      // Wooden box(7)
	{0.25,0.25}   // Wooden box(8)
};

int visited_boxes[9];


double *convert_coords_to_cartesian(const double coordinates[3]){
	double *coordenada_cartesiana = malloc(2);
	coordenada_cartesiana[0] = coordinates[0];
	coordenada_cartesiana[1] = -coordinates[2];
	return coordenada_cartesiana;
}

double convert_heading_to_cartesian(double heading){	

	heading = heading + 90;

	if (heading > 360.0)
		heading = heading - 360;
	
	return heading;
}

bool is_cartesian_theta_equal(const double theta, const double theta2){
	if(fabs(theta-theta2) < THETA_THRESHOLD)
		return true;
	else
		return false;
}


double calculate_destination_theta_in_degrees(const double robot_coord[2], const double dest_coord[2]){
	return atan2(dest_coord[1] - robot_coord[1], dest_coord[0] - robot_coord[0]) * 180 / M_PI;
}


double calculate_robot_theta(double heading, double dest_theta){
	double theta = dest_theta - heading;

	if (theta > 180 - THETA_THRESHOLD)
		theta = -(360-theta);
	else if (theta < -180 + THETA_THRESHOLD)
		theta = (360+theta);

	return theta + 1;
}

double calculate_distance(const double robot_coord[2], const double dest_coord[2]){
	return sqrt(pow(dest_coord[0] - robot_coord[0], 2) + pow(dest_coord[1] - robot_coord[1], 2));
}

double convert_heading_in_degrees(double heading_in_rad){
           return heading_in_rad * 180 / M_PI;
}

int get_min_distance_box_info(const double robot_coord[2], double boxes[9][2], int visited_boxes[9]){
  		   int i = 0;
  		   double min_distance = 1000;
		   double temp = 0.0;
		   int caixa_alvo = 0;
		   for (i = 0; i < 9; i++){
		   		if(visited_boxes[i] == 0){
			  		temp = calculate_distance(robot_coord, boxes[i]);
			  		if (temp <= min_distance){
			  			min_distance = temp;
						caixa_alvo = i;
			  		}
				} 
		   }
		   return caixa_alvo;
}

void motor_stop(){
	wb_motor_set_velocity(left_motor, 0);
	wb_motor_set_velocity(right_motor, 0);
}

void motor_move_forward(){
	wb_motor_set_velocity(left_motor, MAX_SPEED);
	wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_left(){
	wb_motor_set_velocity(left_motor, -MAX_SPEED);
	wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_right(){
	wb_motor_set_velocity(left_motor, MAX_SPEED);
	wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

void rotate_to_box(const double theta, int time_step){
	if (!is_cartesian_theta_equal(theta,0)){
		double duration = abs(theta) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
	 	printf("Duração para chegar ao destino: %4.4f\n",duration);
		
		if(theta>0)
			motor_rotate_left();
		else if (theta < 0)
			motor_rotate_right();

		double start_time = wb_robot_get_time();
		do{
			wb_robot_step(time_step);
		}while (wb_robot_get_time() < start_time + duration);	
	}
}


void move_forward(double distance, int time_step){
	double duration = (distance/2) / TANGENSIAL_SPEED;

	motor_move_forward();

	double start_time = wb_robot_get_time();
	do{
		wb_robot_step(time_step);
	}while (wb_robot_get_time() < start_time + duration);
        
           motor_stop();

}

void move_robot_special(double distance, int option){
           double duration = 0.0;
           if (option == 1){
             duration = distance / TANGENSIAL_SPEED;
          
             motor_move_forward();
           }
           else if (option == 2){
             duration = distance / ROBOT_ANGULAR_SPEED_IN_DEGREES;
             motor_rotate_left();
           }
           else if (option == 3){
              duration = distance / ROBOT_ANGULAR_SPEED_IN_DEGREES;
              motor_rotate_right();
           }

	double start_time = wb_robot_get_time();
	do{
		wb_robot_step(time_step);
	}while (wb_robot_get_time() < start_time + duration);
        
}


int get_time_step(){
	static int time_step = -1;
	if (time_step == -1)
		time_step = (int)wb_robot_get_basic_time_step();
	return time_step;
}

void init(){
	time_step = get_time_step();
	
           char ps_names[8][4] ={
             "ps0", "ps1", "ps2", "ps3",
             "ps4", "ps5", "ps6", "ps7"
           };
           for (int i = 0; i < 8 ; i++) {
             ps[i] = wb_robot_get_device(ps_names[i]);
             wb_distance_sensor_enable(ps[i], time_step);
           }
          	
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

    robot_node = wb_supervisor_node_get_from_def("e-Puck");
    trans_field = wb_supervisor_node_get_proto_field(robot_node, "translation");
    rotac_field = wb_supervisor_node_get_proto_field(robot_node, "rotation");
          
}

int main(int argc, char **argv){
	wb_robot_init();
	
	init();

	int target = -1;
	
	int error_counter = 0;

	
	double *current_coord;
	double theta_destination;
	double robot_heading;
	double robot_heading_deg;

	double converted_theta_to_destination;
	double destination_distance;
	
	while(wb_robot_step(time_step) != -1){
                       for (int i = 0; i < 8 ; i++)
                         ps_values[i] = wb_distance_sensor_get_value(ps[i]);
                        		
                       bool right_obstacle =  ps_values[0] > 140.0 ||
                                              ps_values[1] > 140.0;
                       bool left_obstacle =   ps_values[6] > 140.0 ||
                                              ps_values[7] > 140.0;
                                           
                      robot_position = wb_supervisor_field_get_sf_vec3f(trans_field);	
                      robot_rotation = wb_supervisor_field_get_sf_rotation(rotac_field);
		current_coord = convert_coords_to_cartesian(robot_position);

		if(target == -1){
      		           error_counter = 0;
			target = get_min_distance_box_info(current_coord, boxes, visited_boxes);
		

                  		theta_destination = calculate_destination_theta_in_degrees(current_coord, boxes[target]);
                  		robot_heading_deg = convert_heading_in_degrees(robot_rotation[3]);
                  		robot_heading = convert_heading_to_cartesian(robot_heading_deg);
                  		converted_theta_to_destination = calculate_robot_theta(robot_heading, theta_destination);
                  		
                  		rotate_to_box(converted_theta_to_destination, time_step);
                  
                  		destination_distance = calculate_distance(current_coord, boxes[target]);
                  
        		           move_forward(destination_distance, time_step);
		}
		
		if(right_obstacle || left_obstacle){
          		            		  
          		  if (destination_distance <= BOX_THRESHOLD)
                          visited_boxes[target] = 1;
                        
                          
                        //motor_stop();
                        
                        if(left_obstacle && right_obstacle){
                          printf("Colisão frontal\n");
                          move_robot_special(180,2);
                        }
                        else if(left_obstacle){
                          printf("Colisão esquerda\n");
                          move_robot_special(30,3);
                        }
                        else if(right_obstacle){
                          printf("Colisão direita\n");
                          move_robot_special(30,2);
                        } 
                        target = -1;
                        move_robot_special(0.1,1);
                      }
                      
                      if (error_counter > 10){
                        printf("Recalculando rota\n");
                        target = -1;
                      }
                      
                      error_counter += 1;
                                            
	}
}
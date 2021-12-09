#include <stdio.h>
#include <stdlib.h>

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/distancia_sensor.h>

#define THETA_THRESHOLD 1 //Grau

#define MAX_SPEED 6.28 //rad/s

#define TANGENSIAL_SPEED 0.12874

#define ROBOT_ROTATIONAL_SPEED 0.772881647

#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.587

#define BOX_THRESHOLD 0.35


// Motores
static WbDeviceTag left_motor, right_motor;
// Nó do supervisor do E-Puck
WbNodeRef robot_node;
// Nó do supervisor da translação (X,Y,Z) do robô
WbFieldRef trans_campo;
// Nó do supervisor da rotação (X,Y,Z,ângulo) do robô
// o ângulo é dado em radianos
WbFieldRef rotac_campo;
// Sensores de proximidade
WbDeviceTag ps[8];

// Leitura da saída dos sensores
double ps_values[8];
const double *pos_robo;	
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


double calculate_distancia(const double robot_coord[2], const double dest_coord[2]){
	return sqrt(pow(dest_coord[0] - robot_coord[0], 2) + pow(dest_coord[1] - robot_coord[1], 2));
}


double convert_heading_in_degrees(double heading_in_rad){
           return heading_in_rad * 180 / M_PI;
}


/**
* Adquire a caixa mais próxima do robô e a indica.
*
* Calcula a distância mínima entre o robô e todas as caixas mapeadas,
* dessa forma, independente do ponto inicial do robô ele poderá se
* localizar corretamente.
*
* @param robot_coord Array com o posicionamento do robô já convertido para cartesiano.
* @param boxes Matriz de dimensões conhecidas e contém as posições X
*              e Y das caixas.
* @param visited_boxes Array contendo status das caixas, que varia entre
*              0: Não visitada e 1: Visitada.
*
* @return int índice da matriz com as coordenadas da caixa alvo mais próxima.
*/
int get_min_distancia_box_info(const double robot_coord[2], double boxes[9][2], int visited_boxes[9]){
  		   int i = 0;
  		   double min_distancia = 1000;
		   double temp = 0.0;
		   int caixa_alvo = 0;
		   printf("Caixas visitadas\n");
		   for (i = 0; i < 9; i++){
		   		printf("--->Caixa %d: %d\n",i,visited_boxes[i]);

		   		// Caixa não foi visitada, pode ser considera na descoberta de menor distância.
		   		if(visited_boxes[i] == 0){
		   			// Fórmula de distância entre dois pontos.
			  		temp = calculate_distancia(robot_coord, boxes[i]);
			  		if (temp <= min_distancia){
			  			min_distancia = temp;
						// Salva a caixa que possui a menor distância.
						caixa_alvo = i;
			  		}
				} 
		   }
		   return caixa_alvo;
}

//==========================//
//          MOTORES         //
//==========================//

/*
 * Função que define velocidade 0 à ambos motores.
 * Fazendo o robô parar.
 */
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
		double duracao = abs(theta) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
		
		if(theta>0)
			motor_rotate_left();
		else if (theta < 0)
			motor_rotate_right();

		double start_time = wb_robot_get_time();
		do{
			wb_robot_step(time_step);
		}while (wb_robot_get_time() < start_time + duracao);	
	}
}


void move_forward(double distancia, int time_step){
	double duracao = (distancia/2) / TANGENSIAL_SPEED;

	motor_move_forward();

	double start_time = wb_robot_get_time();
	do{
		wb_robot_step(time_step);
	}while (wb_robot_get_time() < start_time + duracao);
        
           motor_stop();

}


void move_robot_special(double distancia, int option){
           double duracao = 0.0;
           if (option == 1){
             duracao = distancia / TANGENSIAL_SPEED;
          
             motor_move_forward();
           }
           else if (option == 2){
             duracao = distancia / ROBOT_ANGULAR_SPEED_IN_DEGREES;
             motor_rotate_left();
           }
           else if (option == 3){
              duracao = distancia / ROBOT_ANGULAR_SPEED_IN_DEGREES;
              motor_rotate_right();
           }

	double start_time = wb_robot_get_time();
	do{
		wb_robot_step(time_step);
	}while (wb_robot_get_time() < start_time + duracao);
        
           //motor_stop();
}



//==========================//
//     CÓDIGO PRINCIPAL     //
//==========================//

/*
 * Retorna o tempo da simulação.
 */ 
int get_time_step(){
	static int time_step = -1;
	if (time_step == -1)
		time_step = (int)wb_robot_get_basic_time_step();
	return time_step;
}

void init(){
	time_step = get_time_step();
	
	// Nome dos sensores de proximidade
           char ps_names[8][4] ={
             "ps0", "ps1", "ps2", "ps3",
             "ps4", "ps5", "ps6", "ps7"
           };
           // Inicialização dos sensores.
           for (int i = 0; i < 8 ; i++) {
             // Salva o valor apontado no sensor de proximidade
             ps[i] = wb_robot_get_device(ps_names[i]);
             // Ativa todos os sensores, os resultados serão coletados
             // periódicamente em milissegundos TIME_STEP. 
             wb_distancia_sensor_enable(ps[i], time_step);
           }
          	
	// Inicialização dos motores das rodas direita e esquerda.
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

	// Mapeia o robô através do supervisor
    robot_node = wb_supervisor_node_get_from_def("e-Puck");
    // Translação do robô (x,y,z)
    trans_campo = wb_supervisor_node_get_proto_field(robot_node, "translation");
    // Rotação do robô (x,y,z,ângulo)
    rotac_campo = wb_supervisor_node_get_proto_field(robot_node, "rotation");
          
}

int main(int argc, char **argv){
	wb_robot_init();
	
	init();

	int alvo = -1;
	
	int cont_erro = 0;

	
	double *cord_atual;
	double dest_theta;
	double robot_heading;
	double robot_heading_deg;

	double conves_theta_p_destion;
	double dist_destin;
	
	while(wb_robot_step(time_step) != -1){
		 // Atualiza sensores de proximidade
                       for (int i = 0; i < 8 ; i++)
                         ps_values[i] = wb_distancia_sensor_get_value(ps[i]);
                        		
                       bool obsta_dir =  ps_values[0] > 140.0 ||
                                              ps_values[1] > 140.0;
                       bool obsta_esq =   ps_values[6] > 140.0 ||
                                              ps_values[7] > 140.0;
                                           
		// Atualiza o vetor de posições do robô ([0]: X, [1]: Y, [2]: Z)
                      pos_robo = wb_supervisor_field_get_sf_vec3f(trans_campo);	
                      // Atualiza o vetor de rotação do robô ([0]: X, [1]: Y, [2]: Z, [3]: ângulo)
                      robot_rotation = wb_supervisor_field_get_sf_rotation(rotac_campo);
		// Adquire a coordenada cartesiana do robô.
		cord_atual = convert_coords_to_cartesian(pos_robo);

		if(alvo == -1){
      		           cont_erro = 0;
			alvo = get_min_distancia_box_info(cord_atual, boxes, visited_boxes);
		

                  		dest_theta = calculate_destination_theta_in_degrees(cord_atual, boxes[alvo]);
                  		robot_heading_deg = convert_heading_in_degrees(robot_rotation[3]);
                  		robot_heading = convert_heading_to_cartesian(robot_heading_deg);
                  		conves_theta_p_destion = calculate_robot_theta(robot_heading, dest_theta);
                  		
                  		rotate_to_box(conves_theta_p_destion, time_step);
                  
                  		dist_destin = calculate_distancia(cord_atual, boxes[alvo]);
                  
        		           move_forward(dist_destin, time_step);
		}
		
		if(obsta_dir || obsta_esq){
          		            		  
          		  if (dist_destin <= BOX_THRESHOLD)
                          visited_boxes[alvo] = 1;
                        
                          
                        //motor_stop();
                        
                        if(obsta_esq && obsta_dir){
                          printf("Colisão frontal\n");
                          move_robot_special(180,2);
                        }
                        else if(obsta_esq){
                          printf("Colisão esquerda\n");
                          move_robot_special(30,3);
                        }
                        else if(obsta_dir){
                          printf("Colisão direita\n");
                          move_robot_special(30,2);
                        } 
                        alvo = -1;
                        move_robot_special(0.1,1);
                      }
                      
                      if (cont_erro > 10){
                        printf("Recalculando rota\n");
                        alvo = -1;
                      }
                      
                      cont_erro += 1;
                                            
	}
}
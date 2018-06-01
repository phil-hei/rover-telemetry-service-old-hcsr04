/* Copyright 2018 Pedro Cuadra - pjcuadra@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <vector>

#include <systemd/sd-event.h>
#include <json-c/json.h>

#include <afb/afb-wsj1.h>
#include <afb/afb-ws-client.h>
#include <app/RoverConfig.h>
#include <app/RoverDriving.h>

#include <roverapi/rover_pahomqtt.hpp>
#include <roverapi/rover_mqttcommand.hpp>

using namespace std;
using namespace rover;

void drive_rover(RoverDriving &driving, int speed, char command) {
  switch (command) {
    case 'W':
      driving.setspeed(speed);
      driving.goforward();
      break;
    case 'S':
      driving.setspeed(speed);
      driving.gobackward();
      break;
    case 'K':
      driving.setspeed(speed);
      driving.turnright();
      break;
    case 'J':
      driving.setspeed(speed);
      driving.turnleft();
      break;
    default:
      driving.stop();
  }
}

void get_sensor_data(RoverSensorData_t &sensor_data) {
  sensor_data.ultrasonic_front = 1;
	sensor_data.ultrasonic_rear = 2;
	sensor_data.hmc5883l_bearing = 3;
	sensor_data.infrared[0] = 4;
	sensor_data.infrared[1] = 5;
	sensor_data.infrared[2] = 6;
	sensor_data.infrared[3] = 7;
	sensor_data.gy521_accel_x = 8;
  sensor_data.gy521_accel_y = 9;
	sensor_data.gy521_accel_z = 10;
	sensor_data.gy521_gyro_x = 11;
	sensor_data.gy521_gyro_y = 12;
	sensor_data.gy521_gyro_z = 13;
	sensor_data.gy521_angle_x = 14;
	sensor_data.gy521_angle_y = 15;
	sensor_data.gy521_angle_z = 16;
	sensor_data.core[0] = 17;
	sensor_data.core[1] = 18;
	sensor_data.core[2] = 19;
	sensor_data.core[3] = 20;
}

/* entry function */
int main(int ac, char **av, char **env)
{
	int rc = 0;
  RoverMQTTCommand *rover_mqtt;
  string host;
  char * host_c;
  string config_val;
  int host_port = 0;
  int rover_id = 0;
  int qos = 0;
  int use_redirected_topics = 0;
  string passwd;
  char * passwd_c;
  string username;
  char * username_c;
  RoverControlData_t control_data;
  RoverSensorData_t sensor_data;
  int rt = -1;
	int try_count = 0;
	const int max_tries = 100;


	/*get port and token from the command arg*/
	char *port = av[1];
	char *token = av[2];
  char uri[500];

  sprintf(uri, "127.0.0.1:%s/api?token=%s", port, token);

  // Create services objects
  RoverConfig config(uri);
  RoverDriving driving(uri);

  // Get all configuration values
  rt = config.get("MQTT_BROKER_C", host);
  rt = config.get("MQTT_USERNAME_C", username);
  rt = config.get("MQTT_PASSWORD_C", passwd);

  rt = config.get("MQTT_BROKER_PORT_C", config_val);
  host_port = stoi(config_val);

  rt = config.get("ROVER_IDENTITY_C", config_val);
  rover_id = stoi(config_val);

  rt = config.get("ROVER_MQTT_QOS_C", config_val);
  qos = stoi(config_val);

  host_c = strdup(host.c_str());
  username_c = strdup(username.c_str());
  passwd_c = strdup(passwd.c_str());

  rt = config.get("USE_REDIRECTED_TOPICS_C", config_val);
  use_redirected_topics = stoi(config_val);

  rover_mqtt = new RoverMQTTCommand(host_c,
										                host_port,
									                	rover_id,
								                		qos,
							                   		username_c,
							                			passwd_c,
									                	"rover_mqtt_publisher");


  // Connect to Rover
  while (1) {
    if (rover_mqtt->getRoverConnected() != 1) {
      rt = rover_mqtt->connectRover();
      AFB_NOTICE("Connecting Rover...");
      continue;
    }

    rt = rover_mqtt->subscribeToDrivingTopic();
    AFB_NOTICE("Subscribing...");


    if (!rt) {
      AFB_NOTICE("Client rover_mqtt_subscriber: Subscription succesful!");
      break;
    }

    AFB_NOTICE("Client rover_mqtt_subscriber: Subscription unsuccessful!");

    if (try_count >= max_tries - 1) {
      break;
    }

    try_count += 1;

    usleep(1000);
  }

  // Loop forever readFromDrivingTopic and publishtotelemetry
	while (true) {
		control_data = rover_mqtt->readFromDrivingTopic();

		if (control_data.data_ready == 1) {
      drive_rover(driving, control_data.speed, control_data.command);
		}

    get_sensor_data(sensor_data);

    if (use_redirected_topics) {
			rt = rover_mqtt->publishToTelemetryTopic(sensor_data);
      AFB_NOTICE("Using Redirected Publishing");
		} else {
      rt = rover_mqtt->publishToTelemetryTopicNonRedirected(sensor_data);
      AFB_NOTICE("Using NonRedirected Publishing");
		}

    if (!rt) {
      AFB_NOTICE("Client rover_mqtt_publisher: Publishing successful");
    } else {
      AFB_NOTICE("Client rover_mqtt_publisher: Publishing unsuccessful");
    }
  }

  if (rc) {
    return -1;
  }

}

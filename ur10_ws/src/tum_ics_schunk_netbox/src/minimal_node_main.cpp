
#include <ros/ros.h>
#include <mutex>
#include <geometry_msgs/WrenchStamped.h>
//#include "tum_ics_ft_sensor_utils/geometry_msgs_utility/WrenchStamped.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PORT 49152    /* Port the Net F/T always uses */
#define COMMAND 2     /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct
{
  uint32 rdt_sequence;
  uint32 ft_sequence;
  uint32 status;
  int32 FTData[6];
} RESPONSE;

int main(int argc, char **argv)
{
  if (3 > argc)
  {
    fprintf(stderr, "Usage: %s IP_ADDRESS\n", argv[0]);
    return -1;
  }

  // Init ros node.
  ros::init(argc, argv, "schunk_ft_sensor");
  ros::NodeHandle nh;

  long conv = strtol(argv[2], nullptr, 10);
  ros::Rate rate(conv);

  // publisher
  geometry_msgs::WrenchStamped ft_data;
  ft_data.header.frame_id = "ft_sensor_link";

  ros::Publisher rft_publisher =
      nh.advertise<geometry_msgs::WrenchStamped>("raw", 10);

  int socketHandle;                                    // Handle to UDP socket used to communicate with Net F/T.
  struct sockaddr_in addr;                             // Address of Net F/T.
  struct hostent *he;                                  // Host entry for Net F/T.
  byte request[8];                                     // The request data sent to the Net F/T.
  RESPONSE resp;                                       // The structured response received from the Net F/T.
  byte response[36];                                   // The raw response data received from the Net F/T.
  int i;                                               // Generic loop/array index.
  int err;                                             // Error status of operations.

  // Calculate number of samples, command code, and open socket here.
  socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
  if (socketHandle == -1)
  {
    exit(1);
  }

  *(uint16 *)&request[0] = htons(0x1234);      // standard header.
  *(uint16 *)&request[2] = htons(COMMAND);     // per table 9.1 in Net F/T user manual.
  *(uint32 *)&request[4] = htonl(NUM_SAMPLES); // see section 9.1 in Net F/T user manual.

  // Sending the request.
  he = gethostbyname(argv[1]);
  memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);

  err = connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr));
  if (err == -1)
    exit(2);

  while (ros::ok())
  {
    // Send data request.
    send(socketHandle, request, 8, 0);

    // Receiving the response.
    recv(socketHandle, response, 36, 0);
    resp.rdt_sequence = ntohl(*(uint32 *)&response[0]);
    resp.ft_sequence = ntohl(*(uint32 *)&response[4]);
    resp.status = ntohl(*(uint32 *)&response[8]);
    for (i = 0; i < 6; i++)
    {
      resp.FTData[i] = ntohl(*(int32 *)&response[12 + i * 4]);
    }

    // publish it to a topic.
    ft_data.header.stamp = ros::Time::now();
    ft_data.wrench.force.x = static_cast<double>(resp.FTData[0]) / (1e6);
    ft_data.wrench.force.y = static_cast<double>(resp.FTData[1]) / (1e6);
    ft_data.wrench.force.z = static_cast<double>(resp.FTData[2]) / (1e6);
    ft_data.wrench.torque.x = static_cast<double>(resp.FTData[3]) / (1e6);
    ft_data.wrench.torque.y = static_cast<double>(resp.FTData[4]) / (1e6);
    ft_data.wrench.torque.z = static_cast<double>(resp.FTData[5]) / (1e6);
    rft_publisher.publish(ft_data);

    rate.sleep();
  }

  return 0;
}

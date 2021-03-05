#include <stdio.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>

#include <stdlib.h> // exit()
#include <string.h> // memset()


#define PORT 8090
#define SERVER_IP "0.0.0.0"
#define CLIENT_IP "192.168.31.225"
char RecvBuf[100];
char SendBuf[100];


int main()
{
    int result;
    int SendSocket;
    int RecvSocket;

    struct sockaddr_in server = {};
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr.s_addr = inet_addr(SERVER_IP);

    struct sockaddr_in client = {};
    client.sin_family = AF_INET;
    client.sin_port = htons(PORT);
    client.sin_addr.s_addr = inet_addr(CLIENT_IP);

    SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
   
    socklen_t len = sizeof(server);
   
    if(bind(RecvSocket, (struct sockaddr*) &server, len ) < 0 ) {
        perror("bind() ERROR");
        return 3;
    }
    
    float left, right, x, y;
    int messLength;

    while( 1 )
    {
        if(recvfrom(RecvSocket, RecvBuf, sizeof(RecvBuf), 0,(struct sockaddr*)&server, &len) < 0) {
            perror("recvfrom() ERROR");
            return 4;
        } else {
            std::system("clear");
            printf("%s \n", RecvBuf);
        }

        std::string data(RecvBuf);

        while (!data.empty())
        {
            int separator = data.find(';');
            std::string parse = data.substr(1, separator - 1); 

            switch (data.front())
            {
            case 'X':
                x = -(std::stof(parse));
                printf("Gyroscope X: %f\n", x);
                break;
            
            case 'Y':
                y = -(std::stof(parse));
                printf("Gyroscope Y: %f\n", y);
                break;
            
            case 'Z':
                printf("Gyroscope Z: %s\n", parse.c_str());
                break;
            
            case 'B':
                printf("Battery voltage: %s\n", parse.c_str());
                break;
            
            default:
                printf("Unkown data: %s\n", parse.c_str());
                break;
            }
            if(separator == -1) data.clear();
            else data.erase(data.begin(), data.begin() + separator + 1);

        }
        left = (x - y) * 30;
        right = (x + y) * 30;
        if(left >  255) left = 255;
        if(right > 255) right = 255;
        if(left < -255) left = -255;
        if(right < -255) right = -255;
        messLength = sprintf(SendBuf, "L%.0f;R%.0f", right, left);
        result = sendto(SendSocket, SendBuf, messLength, 0, (struct sockaddr *) & client, sizeof(client));
        printf("Sent bytes: %d, %s\n", result, SendBuf);
    }
}
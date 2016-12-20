#ifndef TCPCONNECTION_H
#define TCPCONNECTION_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h> // hostent
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cerrno>

#define LISTEN_BACKLOG 2

class TcpConnection
{
private:
    int listenfd;
    int sockfd;

public:
    TcpConnection()
    {
        sockfd = -1;
        listenfd = -1;
    }

    virtual ~TcpConnection() {
        // safe to call even if not connected
        disconnect();
        disconnectListener();
    }

    void listenOn(int port)
    {
        if (listenfd != -1) {
            std::cerr << "socket already in use\n";
            exit(1);
        }

        listenfd = socket(AF_INET, SOCK_STREAM, 0);
        if (listenfd == -1)
        {
            perror("Could not create socket");
            exit(1);
        }

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(listenfd, (sockaddr*) &address, sizeof(sockaddr_in)) < 0)
        {
            perror("bind failed");
            exit(1);
        }

        if (listen(listenfd, LISTEN_BACKLOG) < 0)
        {
            perror("listen failed");
            exit(1);
        }
    }

    void waitForIncomingConnection()
    {
        struct sockaddr_in incomingAddr;
        socklen_t incomingAddrLen = sizeof(incomingAddr);

        sockfd = accept(listenfd, (sockaddr*)&incomingAddr, &incomingAddrLen);
        if (sockfd < 0)
        {
            perror("ERROR on incoming connection accept");
            exit(1);
        }
    }

    void connectTo(const std::string &host, int port)
    {
        if (sockfd != -1) {
            std::cerr << "socket already in use\n";
            exit(1);
        }

        sockaddr_in remote;

        //setup address structure
        std::cout << "Host Name: " << host << std::endl;
        if (inet_aton(host.c_str(), &remote.sin_addr) == 0) // is this not a numeric IP addr?
//        if (inet_addr(host.c_str()) == -1) // is it a hostname?
        {
            struct hostent *he;
            struct in_addr **addr_list;

            //resolve the hostname, its not an ip address
            if ((he = gethostbyname(host.c_str())) == NULL)
            {
                //gethostbyname failed
                herror("gethostbyname");
                std::cerr << "Failed to resolve hostname\n";
                exit(1);
            }


            //Cast the h_addr_list to in_addr , since h_addr_list also has the
            // ip address in long format only
            addr_list = (struct in_addr **) he->h_addr_list;

//            for (int i = 0; addr_list[i] != NULL; i++)
//            {
                //strcpy(ip , inet_ntoa(*addr_list[i]) );
                remote.sin_addr = *addr_list[0];

                std::cout << host << " resolved to " << inet_ntoa(*addr_list[0])
                        << std::endl;

//                break;
//            }
        }
//        else // it's a numeric IP addr
//        {
//            inet_aton(host.c_str(), &remote.sin_addr);
////            remote.sin_addr.s_addr = inet_addr();
//        }

        remote.sin_family = AF_INET;
        remote.sin_port = htons(port);

        // create the socket
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd == -1)
        {
            perror("Could not create socket");
            exit(1);
        }

        std::cout << "Socket created\n";
        std::cout << host << " resolved to " << inet_ntoa(remote.sin_addr) << std::endl;
        // now we have a valid socket and remote info, so connect
        if (connect(sockfd, (sockaddr*) &remote, sizeof(sockaddr_in)) < 0)
        {
            perror("Connect failed");
//            shutdown(sockfd, SHUT_RDWR);
//            close(sockfd);
//            sockfd = -1;
            exit(1);
        } else {
            std::cout << "successfully connected to "
                    << inet_ntoa(remote.sin_addr)
                    << ':'
                    << port
                    << std::endl;
        }
    }

    void disconnectListener() {
        if (listenfd == -1)
            return;

        shutdown(listenfd, SHUT_RDWR);
        close(listenfd);
        listenfd = -1;

        std::cout << "disconnected listener\n";
    }


    void disconnect() {
        if (sockfd == -1)
            return;

        shutdown(sockfd, SHUT_RDWR);
        close(sockfd);
        sockfd = -1;

        std::cout << "disconnected from remote\n";
    }

    void disconnectAll() {
        disconnect();
        disconnectListener();
    }

    ssize_t sendData(const void *pData, size_t len)
    {
        ssize_t numWritten;
        size_t numRemaining = len;
        const char * ptr = (const char *) pData;

        while (numRemaining)
        {
            numWritten = write(sockfd, ptr, numRemaining);
            if (numWritten <= 0)
            {
                if (errno == EINTR)
                    continue;
                else
                    return -1; // error
            }

            numRemaining -= numWritten;
            ptr += numWritten;
        }
        return len;
    }

    ssize_t readData(void * pData, size_t maxLen)
    {
        ssize_t numRead;
        size_t numRemaining = maxLen;
        char * ptr = (char *) pData;

        while (numRemaining)
        {
            numRead = read(sockfd, ptr, numRemaining);
            if (numRead < 0)
            {
                if (errno == EINTR)
                    continue;
                else
                    return -1; // error
            }
            else if (numRead == 0) // EOF
                break;

            numRemaining -= numRead;
            ptr += numRead;
        }
        return maxLen - numRemaining;

    }
};

#endif // TCPCONNECTION_H

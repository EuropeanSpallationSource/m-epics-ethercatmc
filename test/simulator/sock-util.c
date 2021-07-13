#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <libgen.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>

#include "cmd.h"
#include "sock-util.h"
#include "sock-ads.h"

#define EPICSSOCKETENABLEADDRESSREUSE_FLAG (1)

#if !defined(__linux__)
  #if defined(SOCK_CLOEXEC) && !defined(__rtems__) && !defined(vxWorks)
  /* with glibc, SOCK_CLOEXEC does not expand to a simple constant */
  #  define HAVE_SOCK_CLOEXEC
  #else
  #  undef SOCK_CLOEXEC
  #  define SOCK_CLOEXEC (0)
  #endif
#endif


#ifdef USE_WINSOCK2
#include <winsock2.h>
#define SHUT_RDWR     SD_BOTH
#else
#include <sys/types.h>
#include <arpa/inet.h>   /* htons, ntohs .. */
#include <sys/socket.h>
#include <arpa/inet.h>   /* htons, ntohs .. */
#include <netdb.h>
#include <sys/select.h>
#include <fcntl.h>
#endif

#if (!defined _WIN32 && !defined __WIN32__ && !defined __CYGWIN__)
  #include <signal.h>
  typedef int SOCKET;
  #define INVALID_SOCKET -1
  typedef socklen_t osiSocklen_t;
#endif

#ifdef START_WINSOCK2
#include "startWinSock.h"
#else
#define startWinSock() 0
#endif


#include "sock-util.h"
#include "logerr_info.h"

/* defines */
#define NUM_CLIENT_CONS 10
#define CLIENT_CONS_BUFLEN 1024

/*****************************************************************************/

/* typedefs */


typedef union osiSockAddr46 {
  struct sockaddr          sa;  /* old struct, too short to hold IPv6 addresses */
  struct sockaddr_storage  ss;  /* new struct. long enough for everything */
  struct sockaddr_in       in;  /* Internet, IPv4 */
  struct sockaddr_in6      in6; /* INternet, IPv6 */
} osiSockAddr46;


typedef struct client_con_type {
  size_t        len_used;
  unsigned char *buffer;
  time_t        last_active_sec;
  time_t        idleTimeout;
  int           fd;
  int           is_listen;
  int           is_ADS;
} client_con_type;

/* static variables */
static client_con_type client_cons[NUM_CLIENT_CONS];
/*****************************************************************************/
#define sockutilhexdump(help_txt, bufptr, buflen)\
{\
  const void* buf = (const void*)bufptr;\
  int len = (int)buflen;\
  uint8_t *data = (uint8_t *)buf;\
  int count;\
  unsigned pos = 0;\
  while (len > 0) {\
    struct {\
      char asc_txt[8];\
      char space[2];\
      char hex_txt[8][3];\
      char nul;\
    } print_buf;\
    memset(&print_buf, ' ', sizeof(print_buf));\
    print_buf.nul = '\0';\
    for (count = 0; count < 8; count++) {\
      if (count < len) {\
        unsigned char c = (unsigned char)data[count];\
        if (c >= 0x20 && c < 0x7F)\
          print_buf.asc_txt[count] = c;\
        else\
          print_buf.asc_txt[count] = '.';\
        snprintf((char*)&print_buf.hex_txt[count],\
                 sizeof(print_buf.hex_txt[count]),\
                 "%02x", c);\
        /* Replace NUL with ' ' after snprintf */\
        print_buf.hex_txt[count][2] = ' ';\
      }\
    }\
    fprintf(stdlog,\
              "%s:%d %s [%02x]%s\n",\
            __FILE__, __LINE__, help_txt, pos, (char*)&print_buf);       \
    len -= 8;\
    data += 8;\
    pos += 8;\
  }\
}\


#define errlogPrintf printf
#define epicsSnprintf snprintf

void epicsSocketConvertErrnoToString (char *pBuf, size_t bufSize)
{
   strerror_r(errno, pBuf, bufSize);
}

/*****************************************************************************/
SOCKET epicsSocketCreate(int domain, int type, int protocol)
{
    SOCKET sock = socket ( domain, type | SOCK_CLOEXEC, protocol );
    if ( sock < 0 ) {
        sock = INVALID_SOCKET;
    }
    else {
        int status = fcntl ( sock, F_SETFD, FD_CLOEXEC );
        if ( status < 0 ) {
            char buf [ 64 ];
            epicsSocketConvertErrnoToString (  buf, sizeof ( buf ) );
            errlogPrintf (
                "epicsSocketCreate: failed to "
                "fcntl FD_CLOEXEC because \"%s\"\n",
                buf );
            close ( sock );
            sock = INVALID_SOCKET;
        }
    }
    return sock;
}

int epicsSocketDestroy (SOCKET sock)
{
  return close(sock);
}

void  epicsSocketEnableAddressReuseDuringTimeWaitState ( SOCKET s )
{
#ifdef _WIN32
    /*
     * Note: WINSOCK appears to assign a different functionality for
     * SO_REUSEADDR compared to other OS. With WINSOCK SO_REUSEADDR indicates
     * that simultaneously servers can bind to the same TCP port on the same host!
     * Also, servers are always enabled to reuse a port immediately after
     * they exit ( even if SO_REUSEADDR isnt set ).
     */
#else
    int yes = true;
    int status;
    status = setsockopt ( s, SOL_SOCKET, SO_REUSEADDR,
        (char *) & yes, sizeof ( yes ) );
    if ( status < 0 ) {
        errlogPrintf (
            "epicsSocketEnableAddressReuseDuringTimeWaitState: "
            "unable to set SO_REUSEADDR?\n");
    }
#endif
}

/*****************************************************************************/
SOCKET epicsSocketCreateBind(int domain, int type, int protocol,
                             unsigned short port, int flags)
{
    osiSockAddr46 addr;
    int status;
    SOCKET sock = socket ( domain, type | SOCK_CLOEXEC, protocol );
    if ( sock < 0 ) {
      return INVALID_SOCKET;
    }  else {
      status = fcntl ( sock, F_SETFD, FD_CLOEXEC );
      if ( status < 0 ) {
        char buf [ 64 ];
        epicsSocketConvertErrnoToString (  buf, sizeof ( buf ) );
        errlogPrintf (
                      "epicsSocketCreate: failed to "
                      "fcntl FD_CLOEXEC because \"%s\"\n",
                      buf );
        epicsSocketDestroy (sock);
        return INVALID_SOCKET;
      }
    }
    if (flags & EPICSSOCKETENABLEADDRESSREUSE_FLAG) {
      epicsSocketEnableAddressReuseDuringTimeWaitState(sock);
    }

    memset(&addr, 0 , sizeof (addr) );
    if (domain == AF_INET) {
      addr.in.sin_family = domain;
      addr.in.sin_addr.s_addr = htonl (INADDR_ANY);
      addr.in.sin_port = htons ( port );
      status = bind (sock, &addr.sa, sizeof (addr.in));
    } else if (domain == AF_INET6) {
      addr.in6.sin6_family = domain;
      addr.in6.sin6_port = htons(port);
      status = bind (sock, &addr.sa, sizeof (addr.in6));
    } else {
      errlogPrintf ("epicsSocketCreate: invalid domain %d\n",
                    domain);
      epicsSocketDestroy (sock);
      return INVALID_SOCKET;
    }
    if (status < 0) {
      char sockErrBuf[64];
      epicsSocketConvertErrnoToString (sockErrBuf, sizeof (sockErrBuf));
      errlogPrintf ("epicsSocketCreate: failed to "
                    "bind because \"%s\"\n",
                    sockErrBuf);
      epicsSocketDestroy (sock);
      return INVALID_SOCKET;
    }
    return sock;
}

unsigned ipAddrToDottedIP64 (
    const osiSockAddr46 *paddr, char *pBuf, unsigned bufSize )
{
    static const char * pErrStr = "<IPA>";
    unsigned strLen;
    int status = -1;

    if ( bufSize == 0u ) {
        return 0u;
    }
    if (paddr->in.sin_family == AF_INET) {
      unsigned addr = ntohl ( paddr->in.sin_addr.s_addr );
      /*
       * inet_ntoa() isnt used because it isnt thread safe
       * (and the replacements are not standardized)
       */
      status = epicsSnprintf (pBuf, bufSize, "%u.%u.%u.%u:%hu",
                              (addr >> 24) & 0xFF,
                              (addr >> 16) & 0xFF,
                              (addr >> 8) & 0xFF,
                              (addr) & 0xFF,
                              ntohs ( paddr->in.sin_port ) );
    } else if (paddr->in6.sin6_family == AF_INET6) {
      if (IN6_IS_ADDR_V4MAPPED(&paddr->in6.sin6_addr)) {
        status = epicsSnprintf (pBuf, bufSize, "%u.%u.%u.%u:%hu",
                                paddr->in6.sin6_addr.s6_addr[12],
                                paddr->in6.sin6_addr.s6_addr[13],
                                paddr->in6.sin6_addr.s6_addr[14],
                                paddr->in6.sin6_addr.s6_addr[15],
                                ntohs ( paddr->in6.sin6_port ) );
      } else {
        status = epicsSnprintf (pBuf, bufSize,
                                "[%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X]:%hu",
                                paddr->in6.sin6_addr.s6_addr[0],
                                paddr->in6.sin6_addr.s6_addr[1],
                                paddr->in6.sin6_addr.s6_addr[2],
                                paddr->in6.sin6_addr.s6_addr[3],
                                paddr->in6.sin6_addr.s6_addr[4],
                                paddr->in6.sin6_addr.s6_addr[5],
                                paddr->in6.sin6_addr.s6_addr[6],
                                paddr->in6.sin6_addr.s6_addr[7],
                                paddr->in6.sin6_addr.s6_addr[8],
                                paddr->in6.sin6_addr.s6_addr[9],
                                paddr->in6.sin6_addr.s6_addr[10],
                                paddr->in6.sin6_addr.s6_addr[11],
                                paddr->in6.sin6_addr.s6_addr[12],
                                paddr->in6.sin6_addr.s6_addr[13],
                                paddr->in6.sin6_addr.s6_addr[14],
                                paddr->in6.sin6_addr.s6_addr[15],
                                ntohs ( paddr->in6.sin6_port ) );
      }
    }
    if ( status > 0 ) {
      strLen = ( unsigned ) status;
      if ( strLen < bufSize - 1 ) {
        return strLen;
      }
    }
    strLen = strlen ( pErrStr );
    if ( strLen < bufSize ) {
        strcpy ( pBuf, pErrStr );
        return strLen;
    }
    else {
        strncpy ( pBuf, pErrStr, bufSize );
        pBuf[bufSize-1] = '\0';
        return bufSize - 1u;
    }
}

/*****************************************************************************/
/*****************************************************************************/
void init_client_cons(void)
{
  unsigned i;
  memset(client_cons, 0, sizeof(client_cons));
  for (i=0; i < NUM_CLIENT_CONS; i++) {
    client_cons[i].fd = -1; /* fd is closed */
    client_cons[i].buffer = malloc(CLIENT_CONS_BUFLEN);
  }
}

void add_client_con(int fd, int is_listen, int is_ADS)
{
  unsigned int i;
  for (i=0; i < NUM_CLIENT_CONS; i++) {
    if (client_cons[i].fd < 0) {
      client_cons[i].fd = fd;
      client_cons[i].idleTimeout = 0;
      client_cons[i].is_listen = is_listen;
      client_cons[i].is_ADS = is_ADS;
      LOGINFO7("%s/%s:%d add i=%d fd=%d is_listen=%d is_ADS=%d\n",
               __FILE__,__FUNCTION__, __LINE__,
               i, fd, is_listen, is_ADS);
      return;
    }
  }
  LOGINFO7("%s/%s:%d add() and close() i=%d fd=%d\n",
           __FILE__,__FUNCTION__, __LINE__, i, fd);
  /* No more space, we should close the oldest not used,
       the most stale.
       But now we close the new one */
  close(fd);
}


int find_client_con(int fd)
{
  unsigned int i;
  for (i=0; i < NUM_CLIENT_CONS; i++) {
    if (client_cons[i].fd == fd) {
      LOGINFO7("%s/%s:%d add i=%d fd=%d\n",
               __FILE__,__FUNCTION__, __LINE__, i, fd);
      return i;
    }
  }
  return -1;
}

void close_and_remove_client_con_i(int i)
{
  if (i >= 0) {
    int fd = client_cons[i].fd;
    int res = close(fd);
    LOGINFO7("%s/%s:%d close i=%d fd=%d res=%d (%s)\n",
             __FILE__,__FUNCTION__, __LINE__,
             i, fd, res,
             res ? strerror(errno) : "");
    client_cons[i].fd = -1;
    handle_close_and_remove_client_con();
    return;
  }
  LOGINFO7("%s/%s:%d close i=%d\n",
           __FILE__,__FUNCTION__, __LINE__, i);
}


void close_and_remove_client_con_fd(int fd)
{
  close_and_remove_client_con_i(find_client_con(fd));
}

/*****************************************************************************/

int get_listen_socket(const char *listen_port_asc)
{
  enum bind_ok_status  {
    bind_ok_not_tried   = -1,
    bind_ok_ok          = 0,
    bind_ok_addr_in_use = 1,
    bind_ok_failed      = 2
  };
  int bind_ok = bind_ok_not_tried;
  int reuse_on = 1;
  int sockfd = -1;
  int socket_family = AF_INET;
  int socket_type = SOCK_STREAM;
  int socket_protocol = 0;
#ifndef USE_WINSOCK2
  struct addrinfo *ai = NULL;
  struct addrinfo hints;
  int gai;
#endif

  if (startWinSock()) {
    RETURN_ERROR_OR_DIE(1, "%s\n", "startWinSock() failed");
  }

#ifndef USE_WINSOCK2
  /* initialize the hints */
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET6;        /* Listen to both IPV4 and IPV6 */
  hints.ai_socktype = SOCK_STREAM;   /* Want TCP */
  hints.ai_flags = AI_PASSIVE;       /* PASSIVE means a "server" */

  gai = getaddrinfo(NULL, listen_port_asc, &hints, &ai);
  if (gai) {
    LOGERR("%s/%s:%d getaddrinfo() failed: %s\n",
          __FILE__, __FUNCTION__, __LINE__,
           gai_strerror(gai));
  }
  if (ai) {
    if (ai->ai_next) {
      LOGERR("More than socket available, ignored\n");
    }
    socket_family = ai->ai_family;
    socket_type = ai->ai_socktype;
    socket_protocol = ai->ai_protocol;
  }
#endif

  sockfd = socket(socket_family, socket_type, socket_protocol);
  if (sockfd < 0) {
    if (socket_family == AF_INET6) {
      /* Some systems have IPv6 compiled,
         but not activated: try with IPv4 */
      socket_family = AF_INET;
      socket_type = SOCK_STREAM;
      socket_protocol = 0;
    }
  }
  if (sockfd < 0) {
    sockfd = socket(socket_family, socket_type, socket_protocol);
    if (sockfd < 0) {
      LOGERR_ERRNO("socket(%d %d %d ) failed sockfd=%d\n",
                   socket_family, socket_type, socket_protocol, sockfd);
      goto freeandret;
    }
  }
  /* The following is needed to prevent the shut down and restart
     of the server needing a timeout from the socket layer */
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
                 (char*)&reuse_on, sizeof(reuse_on)))  {
    LOGERR("setsockopt() failed\n");
    goto error;
  }
  /* make the socket a server socket */
#ifndef USE_WINSOCK2
  if (bind(sockfd, ai->ai_addr, ai->ai_addrlen) == 0) {
    bind_ok = bind_ok_ok;
  } else if (errno == EADDRINUSE) {
    LOGERR("%s/%s:%d bind() failed (%s) (errno=%d)\n",
           __FILE__,__FUNCTION__, __LINE__,
           strerror(errno), errno);
    goto error;
  } else {
    LOGINFO7("%s/%s:%d bind() failed (%s) (errno=%d)\n",
             __FILE__,__FUNCTION__, __LINE__,
             strerror(errno), errno);
    bind_ok = bind_ok_failed;
  }
#endif

  switch (bind_ok) {
    case bind_ok_not_tried:
    case bind_ok_failed:
    {
      int res;
      unsigned port = (unsigned)atoi(listen_port_asc);
      struct sockaddr_in server_addr;
      memset(&server_addr, 0, sizeof(server_addr));
      server_addr.sin_family = AF_INET;
      server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
      server_addr.sin_port = htons(port);
      res = bind(sockfd, (struct sockaddr*)&server_addr, (unsigned int)sizeof(server_addr));
      if (res < 0) {
        LOGERR("%s/%s:%d bind() failed (%s) (errno=%d)\n",
             __FILE__,__FUNCTION__, __LINE__,
               strerror(errno), errno);
        goto error;
      }
    }
    break;
    case bind_ok_ok:
    case bind_ok_addr_in_use:
      break;
  }

  if ((sockfd >= 0) && (listen(sockfd, 1) < 0))
  {
    LOGERR("listen() failed\n");
    goto error;
  }
  goto freeandret;


  error:
  close(sockfd);
  sockfd = -1;

  freeandret:
#ifndef USE_WINSOCK2
  freeaddrinfo(ai);
#endif
  if (sockfd >= 0) {
    LOGINFO("listening on port %s\n", listen_port_asc);
  }
  return sockfd;
}

static void handle_data_on_ADS_socket(int i, int fd, ssize_t read_res, size_t len_used)
{
  size_t ret;
  if (client_cons[i].buffer[0] || client_cons[i].buffer[1]) {
    LOGINFO7("%s/%s:%d fd=%d NOT AMS! buffer[0]=%u buffer[1]=%u buffer=\"%s\"\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             client_cons[i].buffer[0], client_cons[i].buffer[1],
             client_cons[i].buffer);
    close_and_remove_client_con_i(i);
  } else {
    LOGINFO7("%s/%s:%d FD_ISSET fd=%d read_res=%ld\n",
             __FILE__, __FUNCTION__, __LINE__, fd, (long)read_res);

    ret = handle_ams_request(fd, (char *)&client_cons[i].buffer[0],
                             len_used, CLIENT_CONS_BUFLEN);
    if (ret != len_used) {
      close_and_remove_client_con_i(i);
    }
    client_cons[i].len_used = 0;
  }
}


static void handle_data_on_ASC_socket(int i, int fd, ssize_t read_res, size_t len_used)
{
  /* append received data to the end
     keep one place for the '\n'  */
  char *pNewline;

  LOGINFO7("%s/%s:%d FD_ISSET fd=%d read_res=%ld\n",
           __FILE__, __FUNCTION__, __LINE__, fd, (long)read_res);
  client_cons[i].buffer[len_used] = '\0';
  pNewline = strchr((char *)client_cons[i].buffer, '\n');
  LOGINFO7("%s/%s:%d FD_ISSET i=%d fd=%d len_used=%lu pNewline=%d\n",
           __FILE__, __FUNCTION__, __LINE__, i, fd,
           (unsigned long)len_used, pNewline ? 1 : 0);
  if (pNewline) {
    size_t line_len = 1 + (void*)pNewline - (void*)client_cons[i].buffer;
    int had_cr = 0;
    LOGINFO7("%s/%s:%d FD_ISSET i=%d fd=%d line_len=%lu\n",
             __FILE__, __FUNCTION__, __LINE__, i, fd,
             (unsigned long)line_len);
    *pNewline = 0; /* Remove '\n' */
    if (line_len > 1) pNewline--;
    if (*pNewline == '\r') {
      had_cr = 1;
      *pNewline = '\0';
    }
    if (handle_input_line_fd(fd, (const char *)&client_cons[i].buffer[0], had_cr, 1)) {
      close_and_remove_client_con_i(i);
    }
    client_cons[i].len_used = 0;
  }
}

/*****************************************************************************/
void socket_loop_with_select(void)
{
  /* We come here if we have accepted a new connection */
  unsigned int i;
  int end_recv_loop = 0;
  int end_select_loop = 0;
  do
  {
    do
    {
      int max_timeout = 2 * 60 * 60; /*  2 hours */
      int res;
      fd_set rfds;
      struct timeval tv_now;
      struct timeval tv_select;
      int maxfd = 0;

      (void)gettimeofday(&tv_now, NULL);

      FD_ZERO (&rfds);
      tv_select.tv_sec = max_timeout;
      tv_select.tv_usec = 0;

      for (i=0; i < NUM_CLIENT_CONS; i++) {
        int fd = client_cons[i].fd;
        LOGINFO7("%s/%s:%d select(): i=%u fd=%d\n",
                 __FILE__, __FUNCTION__, __LINE__,
                 i, fd);

        time_t idleTimeout = client_cons[i].idleTimeout;
        if (fd < 0) continue;
        if (idleTimeout) {
          time_t last_active_sec = client_cons[i].last_active_sec;
          if (tv_now.tv_sec - idleTimeout > last_active_sec) {
            LOGINFO7("%s/%s:%d timeout i=%d fd=%d\n",
                     __FILE__, __FUNCTION__, __LINE__, i, fd);
            close_and_remove_client_con_i(i);
            fd = -1;
          } else {
            /* Wait at least 1 second */
            time_t wait_now = 1 + idleTimeout + last_active_sec - tv_now.tv_sec;
            if (tv_select.tv_sec > wait_now) {
              tv_select.tv_sec = wait_now;
            }
          }
        }
        if (fd >= 0) {
          FD_SET(fd, &rfds);
          if (maxfd < fd) {
            maxfd = fd;
          }
        }
      }
      LOGINFO7("%s/%s:%d select(): maxfd=%d tv_sec=%lu\n",
               __FILE__, __FUNCTION__, __LINE__,
               maxfd, (unsigned long)tv_select.tv_sec);
      res = select (maxfd + 1, &rfds, NULL, NULL, &tv_select);
      LOGINFO7("%s/%s:%d maxfd=%d res(select)=%d %s\n",
               __FILE__, __FUNCTION__, __LINE__,
               maxfd,
               res,
               res < 0 ? strerror(errno) : "");
      (void)gettimeofday(&tv_now, NULL);
      if (res < 0) {
        end_select_loop = 1;
        end_recv_loop = 1;
      } else {
        unsigned int i;

        for (i=0; i < NUM_CLIENT_CONS; i++) {
          int fd = client_cons[i].fd;
          if (fd < 0) continue;
          if (FD_ISSET (fd, &rfds)) {
            LOGINFO7("%s/%s:%d FD_ISSET fd=%d\n",
                     __FILE__, __FUNCTION__, __LINE__, fd);
            if (client_cons[i].is_listen) {
              int is_listen = 0;
              int accepted_socket;
              accepted_socket = accept(client_cons[i].fd, NULL, NULL);
              {
                osiSockAddr46 addr;
                char buf[64];
                osiSocklen_t saddr_length = sizeof (addr);
                int status;
                status = getpeername ( accepted_socket, &addr.sa, &saddr_length );
                if ( status < 0 ) {
                  char sockErrBuf[64];
                  epicsSocketConvertErrnoToString (sockErrBuf, sizeof ( sockErrBuf ) );
                  errlogPrintf ( "listen: getpeername () error was \"%s\"\n", sockErrBuf );
                } else {
                  ipAddrToDottedIP64(&addr, buf, sizeof(buf));
                  LOGINFO3("%s/%s:%d accepted_socket remote=%s\n",
                           __FILE__, __FUNCTION__, __LINE__, buf);
                }
              }
              LOGINFO3("%s/%s:%d accepted_socket=%d\n",
                       __FILE__, __FUNCTION__, __LINE__, accepted_socket);
              add_client_con(accepted_socket, is_listen, client_cons[i].is_ADS);
              handle_accept_new_client(accepted_socket);
            } else {
              client_cons[i].last_active_sec = tv_now.tv_sec;
              ssize_t read_res;
              size_t len_used = client_cons[i].len_used;
              read_res = recv(fd, (char *)&client_cons[i].buffer[len_used],
                              CLIENT_CONS_BUFLEN - len_used - 1, 0);
              if (read_res <= 0)  {
                if (read_res == 0) {
                  close_and_remove_client_con_i(i);
                  LOGINFO(" EOF i=%d fd=%d\n", i, fd);
                }
              } else {
                if PRINT_STDOUT_BIT8() {
                    sockutilhexdump("IN",
                                    &client_cons[i].buffer[len_used],
                                    read_res);
                }
                len_used = client_cons[i].len_used + read_res;
                client_cons[i].len_used = len_used;
                LOGINFO7("%s/%s:%d buf[0]=0x%x buf[1]=0x%x\n",
                         __FILE__, __FUNCTION__, __LINE__,
                         client_cons[i].buffer[0],
                         client_cons[i].buffer[1]);

                if (client_cons[i].is_ADS) {
                  handle_data_on_ADS_socket(i, fd, read_res, len_used);
                } else {
                  handle_data_on_ASC_socket(i, fd, read_res, len_used);
                }
              }
            }
          }
        }
      }
    } while (!end_recv_loop && !end_select_loop);
  } while (!end_recv_loop);
  LOGINFO("End of loop\n");
}

/*****************************************************************************/
void send_to_socket(int fd, const void *buf, unsigned len)
{
  int res;
  errno = 0;
  res = send(fd, buf, len, 0);
  LOGINFO7("%s/%s:%d fd=%d len=%u res=%d\n",
           __FILE__,__FUNCTION__, __LINE__,
           fd, len, res);
#ifdef ENOTSOCK
  if (res == -1 && errno == ENOTSOCK) {
#else
  if (res == -1) {
#endif
    res = write(fd, buf, len);
    if (res != (int)len)
    {
      LOGERR_ERRNO("write(%u %d) failed, calling close()\n",
                   len, res);
      close_and_remove_client_con_fd(fd);
    }
  }
  else if (res != (int)len)
  {
    LOGERR_ERRNO("send(%u %d) failed, calling close()\n",
                 len, res);
    close_and_remove_client_con_fd(fd);
  }
}


/*****************************************************************************/
void socket_loop(void)
{
  static int is_listen = 1;
  int listen_socket;
  int is_ADS = 0;
  int stop_and_exit = 0;

  init_client_cons();
#if XXX_OLD_CODE
  listen_socket = get_listen_socket("5000");
#else
  {
    unsigned short port = 5000;
    listen_socket = epicsSocketCreateBind(AF_INET6, SOCK_STREAM, IPPROTO_TCP,
                                          port,
                                          EPICSSOCKETENABLEADDRESSREUSE_FLAG);
    if (listen_socket >= 0) {
      LOGINFO("listening2 on port %d\n", port);
    }
    if (listen(listen_socket, 1) < 0) {
      LOGERR_ERRNO ("listen() failed\n");
      exit(3);
    }
  }
#endif
  if (listen_socket < 0)
  {
    LOGERR_ERRNO("no listening socket for ASCII!\n");
    exit(3);
  }
  add_client_con(listen_socket, is_listen, is_ADS);

#if XXXX_OLD_CODE
  listen_socket = get_listen_socket("48898");
#else
  {
    unsigned short port = 48898;
    listen_socket = epicsSocketCreateBind(AF_INET6, SOCK_STREAM, IPPROTO_TCP,
                                          port,
                                          EPICSSOCKETENABLEADDRESSREUSE_FLAG);
    if (listen(listen_socket, 1) < 0) {
      LOGERR_ERRNO ("listen() failed\n");
      exit(3);
    }
    if (listen_socket >= 0) {
      LOGINFO("listening2 on port %d\n", port);
    }
  }
#endif
  if (listen_socket < 0)
  {
    LOGERR_ERRNO("no listening socket for ADS!\n");
    exit(3);
  }
  is_ADS = 1;
  add_client_con(listen_socket, is_listen, is_ADS);

  while (!stop_and_exit)
  {
    socket_loop_with_select();
  }
}


/*****************************************************************************/
extern int socket_set_timeout(int fd, int timeout)
{
  int i = find_client_con(fd);
  if (i >= 0) {
    time_t old = client_cons[i].idleTimeout;
    client_cons[i].idleTimeout = timeout;
    LOGINFO7("%s/%s:%d i=%d fd=%d timeout=%d (old=%lu)\n",
             __FILE__, __FUNCTION__, __LINE__, i, fd, timeout, (unsigned long)old);
    return 0;
  }
  LOGINFO7("%s/%s:%d i=%d fd=%d timeout=%d\n",
           __FILE__, __FUNCTION__, __LINE__, i, fd, timeout);
  return 1;
}

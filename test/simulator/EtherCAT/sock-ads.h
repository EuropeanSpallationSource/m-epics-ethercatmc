#ifndef SOCK_ADS_H
#define SOCK_ADS_H

#include <stddef.h>

size_t handle_ams_request(int fd, char *buf, size_t len, size_t buff_len_max);

#endif

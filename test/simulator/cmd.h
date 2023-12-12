#ifndef CMD_H

extern int handle_input_line(const char *input_line, int had_cr, int had_lf);
extern int handle_input_line_fd(int socket_fd, const char *input_line,
                                int had_cr, int had_lf);
extern void handle_accept_new_client(int fd);
extern void handle_close_and_remove_client_con(void);

#define CMD_H
#endif

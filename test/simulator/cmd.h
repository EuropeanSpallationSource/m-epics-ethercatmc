#ifndef CMD_H

extern int handle_input_line(const char *input_line, int had_cr, int had_lf);
extern int handle_input_line_fd(int socket_fd, const char *input_line, int had_cr, int had_lf);

#define CMD_H
#endif

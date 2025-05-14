#ifndef LOGERR_INFO_H
#define LOGERR_INFO_H
#include <errno.h>
#include <stdio.h>  /* FILE */
#include <stdlib.h> /* exit() */
#include <string.h> /* strerror */
#include <time.h>

#include "cmd_buf.h"

extern unsigned int debug_print_flags;
extern unsigned int die_on_error_flags;

extern FILE *stdlog;
const char *epicsBaseDebugStripPath(const char *file);

#define PRINT_STDOUT_BIT0() (debug_print_flags & 1)
#define PRINT_STDOUT_BIT1() (debug_print_flags & (1 << 1))
#define PRINT_STDOUT_BIT2() (debug_print_flags & (1 << 2))
#define PRINT_STDOUT_BIT3() (debug_print_flags & (1 << 3))
#define PRINT_STDOUT_BIT4() (debug_print_flags & (1 << 4))
#define PRINT_STDOUT_BIT5() (debug_print_flags & (1 << 5))
#define PRINT_STDOUT_BIT6() (debug_print_flags & (1 << 6))
#define PRINT_STDOUT_BIT7() (debug_print_flags & (1 << 7))
#define PRINT_STDOUT_BIT8() (debug_print_flags & (1 << 8))

#define DIE_ON_ERROR_BIT0() (die_on_error_flags & 1)
#define DIE_ON_ERROR_BIT1() (die_on_error_flags & (1 << 1))

#define LOGINFO(fmt, ...) \
  { (void)fprintf(stdlog, fmt, ##__VA_ARGS__); }

#define LOGTIME(fmt, ...)                                                     \
  do {                                                                        \
    struct timespec ts;                                                       \
    char nowSecondsText[25];                                                  \
    char nowNanoSecText[5];                                                   \
    nowSecondsText[0] = 0;                                                    \
    nowNanoSecText[0] = 0;                                                    \
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {                                \
      struct tm now;                                                          \
      if (localtime_r(&ts.tv_sec, &now)) {                                    \
        strftime(nowSecondsText, sizeof(nowSecondsText), "%Y/%m/%d %H:%M:%S", \
                 &now);                                                       \
        snprintf(nowNanoSecText, sizeof(nowNanoSecText), ".03%d",             \
                 (int)ts.tv_nsec / 1000000);                                  \
      }                                                                       \
    }                                                                         \
    fprintf(stdlog, "%s%s %s:%-4d " fmt, nowSecondsText, nowNanoSecText,      \
            epicsBaseDebugStripPath(__FILE__), __LINE__, __VA_ARGS__);        \
  } while (0)

#define LOGINFO3(fmt, ...)                                              \
  do {                                                                  \
    if (PRINT_STDOUT_BIT3()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
  } while (0)

#define LOGTIME3(fmt, ...)                                                 \
  do {                                                                     \
    if (PRINT_STDOUT_BIT3()) {                                             \
      struct timespec ts;                                                  \
      char nowSecondsText[25];                                             \
      char nowNanoSecText[5];                                              \
      nowSecondsText[0] = 0;                                               \
      nowNanoSecText[0] = 0;                                               \
      if (!clock_gettime(CLOCK_REALTIME, &ts)) {                           \
        struct tm now;                                                     \
        if (localtime_r(&ts.tv_sec, &now)) {                               \
          strftime(nowSecondsText, sizeof(nowSecondsText),                 \
                   "%Y/%m/%d %H:%M:%S", &now);                             \
          snprintf(nowNanoSecText, sizeof(nowNanoSecText), ".03%d",        \
                   (int)ts.tv_nsec / 1000000);                             \
        }                                                                  \
      }                                                                    \
      fprintf(stdlog, "%s%s %s:%-4d " fmt, nowSecondsText, nowNanoSecText, \
              epicsBaseDebugStripPath(__FILE__), __LINE__, __VA_ARGS__);   \
    }                                                                      \
  } while (0)

#define LOGINFO4(fmt, ...)                                              \
  do {                                                                  \
    if (PRINT_STDOUT_BIT4()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
  } while (0)

#define LOGINFO5(fmt, ...)                                              \
  do {                                                                  \
    if (PRINT_STDOUT_BIT5()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
  } while (0)

#define LOGINFO6(fmt, ...)                                              \
  do {                                                                  \
    if (PRINT_STDOUT_BIT6()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
  } while (0)

#define LOGTIME6(fmt, ...)                                                 \
  do {                                                                     \
    if (PRINT_STDOUT_BIT6()) {                                             \
      struct timespec ts;                                                  \
      char nowSecondsText[25];                                             \
      char nowNanoSecText[5];                                              \
      nowSecondsText[0] = 0;                                               \
      nowNanoSecText[0] = 0;                                               \
      if (!clock_gettime(CLOCK_REALTIME, &ts)) {                           \
        struct tm now;                                                     \
        if (localtime_r(&ts.tv_sec, &now)) {                               \
          strftime(nowSecondsText, sizeof(nowSecondsText),                 \
                   "%Y/%m/%d %H:%M:%S", &now);                             \
          snprintf(nowNanoSecText, sizeof(nowNanoSecText), ".03%d",        \
                   (int)ts.tv_nsec / 1000000);                             \
        }                                                                  \
      }                                                                    \
      fprintf(stdlog, "%s%s %s:%-4d " fmt, nowSecondsText, nowNanoSecText, \
              epicsBaseDebugStripPath(__FILE__), __LINE__, __VA_ARGS__);   \
    }                                                                      \
  } while (0)

#define LOGINFO7(fmt, ...)                                              \
  do {                                                                  \
    if (PRINT_STDOUT_BIT7()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
  } while (0)

#define LOGERR(fmt, ...) \
  { (void)fprintf(stdlog, fmt, ##__VA_ARGS__); }

#define RETURN_ERROR_OR_DIE(errcode, fmt, ...)                          \
  do {                                                                  \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "Error: ");          \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "%s", "\n");         \
    if (DIE_ON_ERROR_BIT1()) (void)fprintf(stdlog, "%s\n", "exit(2)");  \
    if (DIE_ON_ERROR_BIT1()) exit(2);                                   \
    return errcode;                                                     \
  } while (0)

#define LOGERR_ERRNO(fmt, ...)                                               \
  {                                                                          \
    (void)fprintf(stdlog, "%s/%s:%d errno=%d (%s) ", __FILE__, __FUNCTION__, \
                  __LINE__, errno, strerror(errno));                         \
    (void)fprintf(stdlog, fmt, ##__VA_ARGS__);                               \
  }

#define CMD_BUF_PRINTF_RETURN_OR_DIE(fmt, ...)                          \
  do {                                                                  \
    cmd_buf_printf("Error: ");                                          \
    cmd_buf_printf(fmt, ##__VA_ARGS__);                                 \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "%s", "\n");         \
    if (DIE_ON_ERROR_BIT1()) exit(2);                                   \
    return;                                                             \
  } while (0)

#define CMD_BUF_PRINTF_RETURN_ERROR_OR_DIE(errcode, fmt, ...)           \
  do {                                                                  \
    cmd_buf_printf("Error: ");                                          \
    cmd_buf_printf(fmt, ##__VA_ARGS__);                                 \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "Error: ");          \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__); \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "%s", "\n");         \
    if (DIE_ON_ERROR_BIT1()) exit(2);                                   \
    return errcode;                                                     \
  } while (0)

#endif

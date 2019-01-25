/**
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 */

#include <cutils/sockets.h>
#include <cutils/properties.h>

#define YLOGD_SOCKET_FILE "/dev/socket/ylog/ylogdw"

/**
 * Steps:
 * 1. open file system/core/liblog/logd_write.c
 * 2. append following code
 * 2.a. #include "libylog.c"
 * 2.b. WRITE_TO_YLOGD(); in __write_to_log_daemon
 */
#define WRITE_TO_YLOGD() \
    ylogd_header_t yh; \
    yh.id = log_id; \
    yh.uid = last_uid; \
    yh.pid = last_pid; \
    yh.tid = header.tid; \
    yh.realtime = header.realtime; \
    write_to_ylogd(vec, nr, &yh);


#define  FAIL_RETRY_COUNT 5
#define  LOG_TAG_LIBYLOG  "libylog"

typedef struct  __attribute__((__packed__)){
    uint16_t version;
    typeof_log_id_t id;
    uint16_t uid;
    uint16_t pid;
    uint16_t tid;
    log_time realtime;
    uint16_t msg_offset;
    uint16_t msg_len;
} ylogd_header_t;

static int ylogd_fd = -1;
static int ylogd_enabled = -1;

#define ylogd_debug(msg...) (void*)(0)

static int __write_to_ylog_initialize(void) {
    int i, ret = 0;
    i = TEMP_FAILURE_RETRY(socket(PF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0));
    if (i < 0) {
        ret = -errno;
    } else if (TEMP_FAILURE_RETRY(fcntl(i, F_SETFL, O_NONBLOCK)) < 0) {
        ret = -errno;
        close(i);
    } else {
        struct sockaddr_un un;
        memset(&un, 0, sizeof(struct sockaddr_un));
        un.sun_family = AF_UNIX;
        strcpy(un.sun_path, YLOGD_SOCKET_FILE);
        if (TEMP_FAILURE_RETRY(connect(i, (struct sockaddr *)&un,sizeof(struct sockaddr_un))) < 0){
            ret = -errno;
            close(i);
        } else {
            ylogd_fd = i;
        }
    }
    if (ret < 0){
        ylogd_debug("%s failed %d -> %s\n", __func__, ret, strerror(-ret));
    }else{
        ylogd_debug("%s success\n", __func__);
    }
    return ret;
}


static int record_fail_msg(char *lost_messge,char *tag,log_time realtime){
    struct iovec vec[3];
    ssize_t ret;
    static const unsigned headerLength = 1;
    struct iovec newVec[3 + headerLength];
    android_log_header_t header;
    size_t i, payloadSize;
    int prio = ANDROID_LOG_ERROR;
    vec[0].iov_base = (unsigned char *)&prio;
    vec[0].iov_len  = 1;
    vec[1].iov_base = (void *)tag;
    vec[1].iov_len  = strlen(tag) + 1;
    vec[2].iov_base = (void *)lost_messge;
    vec[2].iov_len  = strlen(lost_messge) + 1;
    header.id = LOG_ID_SYSTEM;
    header.tid = gettid();
    header.realtime.tv_sec = realtime.tv_sec;
    header.realtime.tv_nsec = realtime.tv_nsec;
    newVec[0].iov_base = (unsigned char *)&header;
    newVec[0].iov_len  = sizeof(header);
    for (payloadSize = 0, i = headerLength; i < 3 + headerLength; i++){
        newVec[i].iov_base = vec[i - headerLength].iov_base;
        payloadSize += newVec[i].iov_len = vec[i - headerLength].iov_len;
        if (payloadSize > LOGGER_ENTRY_MAX_PAYLOAD){
            newVec[i].iov_len -= payloadSize - LOGGER_ENTRY_MAX_PAYLOAD;
            if (newVec[i].iov_len){
                ++i;
            }
            break;
        }
    }
    if (logd_fd > 0){
        TEMP_FAILURE_RETRY(writev(logd_fd, newVec, i));
    }
    return 0;
}

static int write_to_ylogd(struct iovec *vec, size_t nr, ylogd_header_t *yh){
    int ret, i, cnt;

    if (ylogd_fd < 0) {
        /**
         * 1. YLOGD_SOCKET_FILE will be created when ylogd started, and deleted when ylogd stopped
         * 2. access /dev/ ramfs should be faster than socket + fcntl + connnect
         *
         * we use access_socket_fcnt_connect_close_test to test this thing
         *
         * 1. in x80, access is always faster than socket+fcntl+connect
         * 2. in arm, access is unstable, but more time is slower than socket+fcntl+connect
         *
         * Conclusion: in arm still keep using orignal method socket+fcntl+connect*/
        if (access(YLOGD_SOCKET_FILE, F_OK)){
            return -EBADF;
        }
        lock();
        if (ylogd_fd < 0){
            ret = __write_to_ylog_initialize();
        }
        unlock();
        if (ylogd_fd < 0) {
            return -EBADF;
        }
    }

    struct iovec newVec[nr + 1];
    newVec[0].iov_base   = (unsigned char *)yh;
    newVec[0].iov_len    = yh->msg_offset = sizeof(*yh);
    cnt = 1;
    for (i = 0; i < (int)nr; i++, cnt++) {
        newVec[i+1] = vec[i];
        yh->msg_offset += vec[i].iov_len;
    }
    yh->msg_offset -= vec[nr-1].iov_len;
    yh->msg_len = vec[nr-1].iov_len;

    if (yh->id == LOG_ID_MAIN ||
        yh->id == LOG_ID_SYSTEM ||
        yh->id == LOG_ID_RADIO) {
        char *msg = vec[2].iov_base;
        ylogd_debug("id=%d %s\n", yh->id, msg);
    }

    /*
    * The write below could be lost, but will never block, because of fcntl(i, F_SETFL, O_NONBLOCK) above
    *
    * ENOTCONN or EPIPE occurs if ylogd dies.
    * EAGAIN occurs if ylogd is overloaded.
    * EBADF occurs if ylogd dies and many threads are calling writev but ylogd_fd is assigned -1 by one thread
    */
    ret = -1;
    for(i = 0; i < FAIL_RETRY_COUNT&&ret<0; i++){
      ret = TEMP_FAILURE_RETRY(writev(ylogd_fd, newVec, cnt));
    }

        /*record the reason of lost log,when ylogd not run ,the socket file need to be deleted*/
    char *error_info = NULL;
    char msg[128];
    memset(msg,0,sizeof(msg));
    if (ret < 0){
      ret = -errno;
      error_info =strerror(errno);
      sprintf(msg, "lost log  reason: %s,",error_info);
      record_fail_msg(msg,LOG_TAG_LIBYLOG,yh->realtime);
      if (1/* ret == -ENOTCONN || ret == -EPIPE */){
        lock();
        if (ylogd_fd >= 0){
            close(ylogd_fd);
            ylogd_fd = -1;
            /* we don't think ylogd can recover so fast, lost this log ruthless */
        }
        unlock();
      }
      ylogd_debug("%s failed %d -> %s\n", __func__, ret, strerror(-ret));
    }
    return ret;
}

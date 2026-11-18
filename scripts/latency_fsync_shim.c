/* D 工单慢磁盘注入 shim（LD_PRELOAD，仅测试进程使用，不属于产品代码）。
 *
 * 拦截 fsync/fdatasync：先按 LATENCY_FSYNC_DELAY_MS 休眠，再调用真实实现。
 * SQLite synchronous=FULL 的每次提交都会触发 fsync，因此账本持久写入被精确
 * 延迟；Bridge 的日志/manifest 走普通 write（无 fsync），不受影响。
 *
 * 可选证据日志：LATENCY_FSYNC_LOG 指向文件时，每次拦截追加一行
 *   {"mono": <CLOCK_MONOTONIC>, "call": "fsync|fdatasync", "fd": n, "delayMs": n}
 *
 * 编译：gcc -shared -fPIC -O2 -o latency_fsync_shim.so latency_fsync_shim.c -ldl
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static long delay_ms(void) {
    const char *s = getenv("LATENCY_FSYNC_DELAY_MS");
    if (!s) return 0;
    long v = strtol(s, NULL, 10);
    return v > 0 ? v : 0;
}

static void inject(const char *call, int fd) {
    long ms = delay_ms();
    struct timespec mono;
    clock_gettime(CLOCK_MONOTONIC, &mono);
    if (ms > 0) {
        struct timespec ts = {ms / 1000, (ms % 1000) * 1000000L};
        nanosleep(&ts, NULL);
    }
    const char *log = getenv("LATENCY_FSYNC_LOG");
    if (log) {
        FILE *f = fopen(log, "a");
        if (f) {
            fprintf(f, "{\"mono\": %.9f, \"call\": \"%s\", \"fd\": %d, \"delayMs\": %ld}\n",
                    (double)mono.tv_sec + (double)mono.tv_nsec / 1e9, call, fd, ms);
            fclose(f);
        }
    }
}

int fsync(int fd) {
    inject("fsync", fd);
    static int (*real_fsync)(int) = NULL;
    if (!real_fsync) real_fsync = dlsym(RTLD_NEXT, "fsync");
    return real_fsync(fd);
}

int fdatasync(int fd) {
    inject("fdatasync", fd);
    static int (*real_fdatasync)(int) = NULL;
    if (!real_fdatasync) real_fdatasync = dlsym(RTLD_NEXT, "fdatasync");
    return real_fdatasync(fd);
}
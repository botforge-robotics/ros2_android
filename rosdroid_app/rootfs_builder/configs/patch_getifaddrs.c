#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <errno.h>
#include <pwd.h>
#include <pthread.h>
#include <limits.h>
#include <fcntl.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>

#define TARGET_HOSTNAME "rosdroid"

// Convert ENOSYS on filesystem access into ENOENT so Python/glibc fallback gracefully in PRoot
int open(const char *pathname, int flags, ...)
{
    mode_t mode = 0;
    if (flags & O_CREAT) {
        va_list args;
        va_start(args, flags);
        mode = va_arg(args, mode_t);
        va_end(args);
    }
    typedef int (*orig_open_fn)(const char *, int, ...);
    static orig_open_fn orig_open = NULL;
    if (!orig_open) orig_open = (orig_open_fn)dlsym(RTLD_NEXT, "open");
    int res = orig_open ? orig_open(pathname, flags, mode) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int open64(const char *pathname, int flags, ...)
{
    mode_t mode = 0;
    if (flags & O_CREAT) {
        va_list args;
        va_start(args, flags);
        mode = va_arg(args, mode_t);
        va_end(args);
    }
    typedef int (*orig_open64_fn)(const char *, int, ...);
    static orig_open64_fn orig_open64 = NULL;
    if (!orig_open64) orig_open64 = (orig_open64_fn)dlsym(RTLD_NEXT, "open64");
    int res = orig_open64 ? orig_open64(pathname, flags, mode) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int openat(int dirfd, const char *pathname, int flags, ...)
{
    mode_t mode = 0;
    if (flags & O_CREAT) {
        va_list args;
        va_start(args, flags);
        mode = va_arg(args, mode_t);
        va_end(args);
    }
    typedef int (*orig_openat_fn)(int, const char *, int, ...);
    static orig_openat_fn orig_openat = NULL;
    if (!orig_openat) orig_openat = (orig_openat_fn)dlsym(RTLD_NEXT, "openat");
    int res = orig_openat ? orig_openat(dirfd, pathname, flags, mode) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int openat64(int dirfd, const char *pathname, int flags, ...)
{
    mode_t mode = 0;
    if (flags & O_CREAT) {
        va_list args;
        va_start(args, flags);
        mode = va_arg(args, mode_t);
        va_end(args);
    }
    typedef int (*orig_openat64_fn)(int, const char *, int, ...);
    static orig_openat64_fn orig_openat64 = NULL;
    if (!orig_openat64) orig_openat64 = (orig_openat64_fn)dlsym(RTLD_NEXT, "openat64");
    int res = orig_openat64 ? orig_openat64(dirfd, pathname, flags, mode) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int stat(const char *pathname, struct stat *statbuf)
{
    typedef int (*orig_stat_fn)(const char *, struct stat *);
    static orig_stat_fn orig_stat = NULL;
    if (!orig_stat) orig_stat = (orig_stat_fn)dlsym(RTLD_NEXT, "stat");
    int res = orig_stat ? orig_stat(pathname, statbuf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int stat64(const char *pathname, struct stat64 *statbuf)
{
    typedef int (*orig_stat64_fn)(const char *, struct stat64 *);
    static orig_stat64_fn orig_stat64 = NULL;
    if (!orig_stat64) orig_stat64 = (orig_stat64_fn)dlsym(RTLD_NEXT, "stat64");
    int res = orig_stat64 ? orig_stat64(pathname, statbuf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int lstat(const char *pathname, struct stat *statbuf)
{
    typedef int (*orig_lstat_fn)(const char *, struct stat *);
    static orig_lstat_fn orig_lstat = NULL;
    if (!orig_lstat) orig_lstat = (orig_lstat_fn)dlsym(RTLD_NEXT, "lstat");
    int res = orig_lstat ? orig_lstat(pathname, statbuf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int lstat64(const char *pathname, struct stat64 *statbuf)
{
    typedef int (*orig_lstat64_fn)(const char *, struct stat64 *);
    static orig_lstat64_fn orig_lstat64 = NULL;
    if (!orig_lstat64) orig_lstat64 = (orig_lstat64_fn)dlsym(RTLD_NEXT, "lstat64");
    int res = orig_lstat64 ? orig_lstat64(pathname, statbuf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int fstat(int fd, struct stat *statbuf)
{
    typedef int (*orig_fstat_fn)(int, struct stat *);
    static orig_fstat_fn orig_fstat = NULL;
    if (!orig_fstat) orig_fstat = (orig_fstat_fn)dlsym(RTLD_NEXT, "fstat");
    int res = orig_fstat ? orig_fstat(fd, statbuf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int fstat64(int fd, struct stat64 *statbuf)
{
    typedef int (*orig_fstat64_fn)(int, struct stat64 *);
    static orig_fstat64_fn orig_fstat64 = NULL;
    if (!orig_fstat64) orig_fstat64 = (orig_fstat64_fn)dlsym(RTLD_NEXT, "fstat64");
    int res = orig_fstat64 ? orig_fstat64(fd, statbuf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int fstatat(int dirfd, const char *pathname, struct stat *statbuf, int flags)
{
    typedef int (*orig_fstatat_fn)(int, const char *, struct stat *, int);
    static orig_fstatat_fn orig_fstatat = NULL;
    if (!orig_fstatat) orig_fstatat = (orig_fstatat_fn)dlsym(RTLD_NEXT, "fstatat");
    int res = orig_fstatat ? orig_fstatat(dirfd, pathname, statbuf, flags) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int fstatat64(int dirfd, const char *pathname, struct stat64 *statbuf, int flags)
{
    typedef int (*orig_fstatat64_fn)(int, const char *, struct stat64 *, int);
    static orig_fstatat64_fn orig_fstatat64 = NULL;
    if (!orig_fstatat64) orig_fstatat64 = (orig_fstatat64_fn)dlsym(RTLD_NEXT, "fstatat64");
    int res = orig_fstatat64 ? orig_fstatat64(dirfd, pathname, statbuf, flags) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int __xstat(int ver, const char *path, struct stat *stat_buf)
{
    typedef int (*orig_xstat_fn)(int, const char *, struct stat *);
    static orig_xstat_fn orig_xstat = NULL;
    if (!orig_xstat) orig_xstat = (orig_xstat_fn)dlsym(RTLD_NEXT, "__xstat");
    int res = orig_xstat ? orig_xstat(ver, path, stat_buf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int __lxstat(int ver, const char *path, struct stat *stat_buf)
{
    typedef int (*orig_lxstat_fn)(int, const char *, struct stat *);
    static orig_lxstat_fn orig_lxstat = NULL;
    if (!orig_lxstat) orig_lxstat = (orig_lxstat_fn)dlsym(RTLD_NEXT, "__lxstat");
    int res = orig_lxstat ? orig_lxstat(ver, path, stat_buf) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int __fxstatat(int ver, int dirfd, const char *path, struct stat *stat_buf, int flag)
{
    typedef int (*orig_fxstatat_fn)(int, int, const char *, struct stat *, int);
    static orig_fxstatat_fn orig_fxstatat = NULL;
    if (!orig_fxstatat) orig_fxstatat = (orig_fxstatat_fn)dlsym(RTLD_NEXT, "__fxstatat");
    int res = orig_fxstatat ? orig_fxstatat(ver, dirfd, path, stat_buf, flag) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int access(const char *pathname, int mode)
{
    typedef int (*orig_access_fn)(const char *, int);
    static orig_access_fn orig_access = NULL;
    if (!orig_access) orig_access = (orig_access_fn)dlsym(RTLD_NEXT, "access");
    int res = orig_access ? orig_access(pathname, mode) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int faccessat(int dirfd, const char *pathname, int mode, int flags)
{
    typedef int (*orig_faccessat_fn)(int, const char *, int, int);
    static orig_faccessat_fn orig_faccessat = NULL;
    if (!orig_faccessat) orig_faccessat = (orig_faccessat_fn)dlsym(RTLD_NEXT, "faccessat");
    int res = orig_faccessat ? orig_faccessat(dirfd, pathname, mode, flags) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int execve(const char *pathname, char *const argv[], char *const envp[])
{
    typedef int (*orig_execve_fn)(const char *, char *const [], char *const []);
    static orig_execve_fn orig_execve = NULL;
    if (!orig_execve) orig_execve = (orig_execve_fn)dlsym(RTLD_NEXT, "execve");
    int res = orig_execve ? orig_execve(pathname, argv, envp) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

int execv(const char *pathname, char *const argv[])
{
    typedef int (*orig_execv_fn)(const char *, char *const []);
    static orig_execv_fn orig_execv = NULL;
    if (!orig_execv) orig_execv = (orig_execv_fn)dlsym(RTLD_NEXT, "execv");
    int res = orig_execv ? orig_execv(pathname, argv) : -1;
    if (res < 0 && errno == ENOSYS) {
        errno = ENOENT;
    }
    return res;
}

// Hook gethostname to return "rosdroid"
int gethostname(char *name, size_t len)
{
    if (!name || len == 0) {
        errno = EINVAL;
        return -1;
    }
    strncpy(name, TARGET_HOSTNAME, len - 1);
    name[len - 1] = '\0';
    return 0;
}

// Hook sethostname (no-op in PRoot container)
int sethostname(const char *name, size_t len)
{
    return 0;
}

// Hook uname to return nodename "rosdroid"
int uname(struct utsname *buf)
{
    if (!buf) {
        errno = EFAULT;
        return -1;
    }
    typedef int (*orig_uname_fn)(struct utsname *);
    static orig_uname_fn orig_uname = NULL;
    if (!orig_uname) {
        orig_uname = (orig_uname_fn)dlsym(RTLD_NEXT, "uname");
    }
    int res = 0;
    if (orig_uname) {
        res = orig_uname(buf);
    } else {
        memset(buf, 0, sizeof(*buf));
        strcpy(buf->sysname, "Linux");
        strcpy(buf->release, "4.14.180");
        strcpy(buf->version, "#1 SMP PREEMPT");
        strcpy(buf->machine, "aarch64");
    }
    strncpy(buf->nodename, TARGET_HOSTNAME, sizeof(buf->nodename) - 1);
    buf->nodename[sizeof(buf->nodename) - 1] = '\0';
    return res;
}

// PWD and cwd tracking for PRoot containers where getcwd kernel syscall / parent traversal fails
static pthread_mutex_t cwd_mutex = PTHREAD_MUTEX_INITIALIZER;
static char current_working_dir[4096] = {0};

static void normalize_path(const char *src, char *dst, size_t maxlen)
{
    if (!src || !dst || maxlen == 0) return;
    char temp[4096];
    strncpy(temp, src, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char *tokens[256];
    int token_count = 0;

    char *saveptr = NULL;
    char *token = strtok_r(temp, "/", &saveptr);
    while (token && token_count < 256) {
        if (strcmp(token, ".") == 0) {
            // ignore current dir
        } else if (strcmp(token, "..") == 0) {
            if (token_count > 0) token_count--;
        } else if (token[0] != '\0') {
            tokens[token_count++] = token;
        }
        token = strtok_r(NULL, "/", &saveptr);
    }

    if (token_count == 0) {
        snprintf(dst, maxlen, "/");
        return;
    }

    dst[0] = '\0';
    size_t cur = 0;
    for (int i = 0; i < token_count; i++) {
        if (cur + 1 >= maxlen) break;
        dst[cur++] = '/';
        dst[cur] = '\0';
        size_t tlen = strlen(tokens[i]);
        if (cur + tlen >= maxlen) break;
        memcpy(dst + cur, tokens[i], tlen);
        cur += tlen;
        dst[cur] = '\0';
    }
}

static void init_cwd_locked(void)
{
    char link_target[PATH_MAX];
    ssize_t len = readlink("/proc/self/cwd", link_target, sizeof(link_target) - 1);
    if (len > 0) {
        link_target[len] = '\0';
        if (link_target[0] == '/') {
            normalize_path(link_target, current_working_dir, sizeof(current_working_dir));
            return;
        }
    }

    if (current_working_dir[0] != '\0') return;

    typedef char *(*orig_getcwd_fn)(char *, size_t);
    static orig_getcwd_fn orig_getcwd = NULL;
    if (!orig_getcwd) orig_getcwd = (orig_getcwd_fn)dlsym(RTLD_NEXT, "getcwd");
    if (orig_getcwd) {
        char buf[PATH_MAX];
        if (orig_getcwd(buf, sizeof(buf)) != NULL && buf[0] == '/') {
            normalize_path(buf, current_working_dir, sizeof(current_working_dir));
            return;
        }
    }

    const char *pwd = getenv("PWD");
    if (pwd && pwd[0] == '/') {
        normalize_path(pwd, current_working_dir, sizeof(current_working_dir));
        return;
    }
    const char *home = getenv("HOME");
    if (home && home[0] == '/') {
        normalize_path(home, current_working_dir, sizeof(current_working_dir));
        return;
    }
    if (geteuid() == 0) {
        snprintf(current_working_dir, sizeof(current_working_dir), "/root");
    } else {
        snprintf(current_working_dir, sizeof(current_working_dir), "/home/robot");
    }
}

// Guarantee chdir succeeds and updates virtual cwd
int chdir(const char *path)
{
    if (!path || path[0] == '\0') {
        path = (geteuid() == 0) ? "/root" : "/home/robot";
    }
    typedef int (*orig_chdir_fn)(const char *);
    static orig_chdir_fn orig_chdir = NULL;
    if (!orig_chdir) orig_chdir = (orig_chdir_fn)dlsym(RTLD_NEXT, "chdir");
    if (orig_chdir) orig_chdir(path);

    pthread_mutex_lock(&cwd_mutex);
    char link_target[PATH_MAX];
    ssize_t len = readlink("/proc/self/cwd", link_target, sizeof(link_target) - 1);
    if (len > 0) {
        link_target[len] = '\0';
        if (link_target[0] == '/') {
            normalize_path(link_target, current_working_dir, sizeof(current_working_dir));
            setenv("PWD", current_working_dir, 1);
            pthread_mutex_unlock(&cwd_mutex);
            return 0;
        }
    }

    init_cwd_locked();
    if (path[0] == '/') {
        normalize_path(path, current_working_dir, sizeof(current_working_dir));
    } else {
        char combined[4096];
        snprintf(combined, sizeof(combined), "%s/%s", current_working_dir, path);
        normalize_path(combined, current_working_dir, sizeof(current_working_dir));
    }
    setenv("PWD", current_working_dir, 1);
    pthread_mutex_unlock(&cwd_mutex);
    return 0;
}

int fchdir(int fd)
{
    typedef int (*orig_fchdir_fn)(int);
    static orig_fchdir_fn orig_fchdir = NULL;
    if (!orig_fchdir) orig_fchdir = (orig_fchdir_fn)dlsym(RTLD_NEXT, "fchdir");
    int res = orig_fchdir ? orig_fchdir(fd) : 0;

    char path[PATH_MAX];
    char fd_path[64];
    snprintf(fd_path, sizeof(fd_path), "/proc/self/fd/%d", fd);
    ssize_t n = readlink(fd_path, path, sizeof(path) - 1);
    if (n > 0) {
        path[n] = '\0';
        pthread_mutex_lock(&cwd_mutex);
        normalize_path(path, current_working_dir, sizeof(current_working_dir));
        setenv("PWD", current_working_dir, 1);
        pthread_mutex_unlock(&cwd_mutex);
    }
    return (res < 0) ? 0 : res;
}

// Provide reliable getcwd for shells and utilities
char *getcwd(char *buf, size_t size)
{
    pthread_mutex_lock(&cwd_mutex);
    char link_target[PATH_MAX];
    ssize_t len = readlink("/proc/self/cwd", link_target, sizeof(link_target) - 1);
    if (len > 0) {
        link_target[len] = '\0';
        if (link_target[0] == '/') {
            normalize_path(link_target, current_working_dir, sizeof(current_working_dir));
        }
    } else {
        init_cwd_locked();
    }
    size_t needed = strlen(current_working_dir) + 1;
    if (!buf) {
        size_t alloc_sz = (size == 0 || size < needed) ? needed : size;
        buf = (char *)malloc(alloc_sz);
        if (!buf) {
            pthread_mutex_unlock(&cwd_mutex);
            errno = ENOMEM;
            return NULL;
        }
        memcpy(buf, current_working_dir, needed);
        pthread_mutex_unlock(&cwd_mutex);
        return buf;
    }
    if (size < needed) {
        pthread_mutex_unlock(&cwd_mutex);
        errno = ERANGE;
        return NULL;
    }
    memcpy(buf, current_working_dir, needed);
    pthread_mutex_unlock(&cwd_mutex);
    return buf;
}

char *get_current_dir_name(void)
{
    pthread_mutex_lock(&cwd_mutex);
    init_cwd_locked();
    char *res = strdup(current_working_dir);
    pthread_mutex_unlock(&cwd_mutex);
    return res;
}

// Guarantee valid pw_dir and pw_shell for getpwnam
struct passwd *getpwnam(const char *name)
{
    typedef struct passwd *(*orig_getpwnam_fn)(const char *);
    static orig_getpwnam_fn orig_getpwnam = NULL;
    if (!orig_getpwnam) orig_getpwnam = (orig_getpwnam_fn)dlsym(RTLD_NEXT, "getpwnam");
    struct passwd *pw = orig_getpwnam ? orig_getpwnam(name) : NULL;
    if (pw) {
        if (!pw->pw_dir || pw->pw_dir[0] == '\0') {
            pw->pw_dir = (pw->pw_name && strcmp(pw->pw_name, "root") == 0) ? "/root" : "/home/robot";
        }
        if (!pw->pw_shell || pw->pw_shell[0] == '\0') {
            pw->pw_shell = "/bin/bash";
        }
    }
    return pw;
}

// Guarantee valid pw_dir and pw_shell for getpwuid
struct passwd *getpwuid(uid_t uid)
{
    typedef struct passwd *(*orig_getpwuid_fn)(uid_t);
    static orig_getpwuid_fn orig_getpwuid = NULL;
    if (!orig_getpwuid) orig_getpwuid = (orig_getpwuid_fn)dlsym(RTLD_NEXT, "getpwuid");
    struct passwd *pw = orig_getpwuid ? orig_getpwuid(uid) : NULL;
    if (pw) {
        if (!pw->pw_dir || pw->pw_dir[0] == '\0') {
            pw->pw_dir = (uid == 0) ? "/root" : "/home/robot";
        }
        if (!pw->pw_shell || pw->pw_shell[0] == '\0') {
            pw->pw_shell = "/bin/bash";
        }
    }
    return pw;
}

// Synchronize real UID/GID with effective UID/GID so uid == euid in PRoot (prevents bash privileged mode)
uid_t getuid(void)
{
    const char *user = getenv("USER");
    if (user && strcmp(user, "robot") == 0) {
        return 1000;
    }
    typedef uid_t (*orig_geteuid_fn)(void);
    static orig_geteuid_fn orig_geteuid = NULL;
    if (!orig_geteuid) orig_geteuid = (orig_geteuid_fn)dlsym(RTLD_NEXT, "geteuid");
    return orig_geteuid ? orig_geteuid() : 0;
}

uid_t geteuid(void)
{
    const char *user = getenv("USER");
    if (user && strcmp(user, "robot") == 0) {
        return 1000;
    }
    typedef uid_t (*orig_geteuid_fn)(void);
    static orig_geteuid_fn orig_geteuid = NULL;
    if (!orig_geteuid) orig_geteuid = (orig_geteuid_fn)dlsym(RTLD_NEXT, "geteuid");
    return orig_geteuid ? orig_geteuid() : 0;
}

gid_t getgid(void)
{
    const char *user = getenv("USER");
    if (user && strcmp(user, "robot") == 0) {
        return 1000;
    }
    typedef gid_t (*orig_getegid_fn)(void);
    static orig_getegid_fn orig_getegid = NULL;
    if (!orig_getegid) orig_getegid = (orig_getegid_fn)dlsym(RTLD_NEXT, "getegid");
    return orig_getegid ? orig_getegid() : 0;
}

gid_t getegid(void)
{
    const char *user = getenv("USER");
    if (user && strcmp(user, "robot") == 0) {
        return 1000;
    }
    typedef gid_t (*orig_getegid_fn)(void);
    static orig_getegid_fn orig_getegid = NULL;
    if (!orig_getegid) orig_getegid = (orig_getegid_fn)dlsym(RTLD_NEXT, "getegid");
    return orig_getegid ? orig_getegid() : 0;
}

int setgroups(size_t size, const gid_t *list)
{
    return 0;
}

int initgroups(const char *user, gid_t group)
{
    return 0;
}

int getgroups(int size, gid_t list[])
{
    if (size == 0) return 1;
    if (list && size >= 1) {
        list[0] = getegid();
        return 1;
    }
    return 0;
}

// Fake success for chown / fchown / lchown to bypass Android pts/storage permission restrictions
int chown(const char *pathname, uid_t owner, gid_t group)
{
    return 0;
}

int fchown(int fd, uid_t owner, gid_t group)
{
    return 0;
}

int lchown(const char *pathname, uid_t owner, gid_t group)
{
    return 0;
}

int fchownat(int dirfd, const char *pathname, uid_t owner, gid_t group, int flags)
{
    return 0;
}

int chmod(const char *pathname, mode_t mode)
{
    typedef int (*orig_chmod_fn)(const char *, mode_t);
    static orig_chmod_fn orig_chmod = NULL;
    if (!orig_chmod) orig_chmod = (orig_chmod_fn)dlsym(RTLD_NEXT, "chmod");
    int res = orig_chmod ? orig_chmod(pathname, mode) : 0;
    if (res < 0 && (errno == EPERM || errno == EACCES)) {
        return 0;
    }
    return res;
}

int fchmod(int fd, mode_t mode)
{
    typedef int (*orig_fchmod_fn)(int, mode_t);
    static orig_fchmod_fn orig_fchmod = NULL;
    if (!orig_fchmod) orig_fchmod = (orig_fchmod_fn)dlsym(RTLD_NEXT, "fchmod");
    int res = orig_fchmod ? orig_fchmod(fd, mode) : 0;
    if (res < 0 && (errno == EPERM || errno == EACCES)) {
        return 0;
    }
    return res;
}

int fchmodat(int dirfd, const char *pathname, mode_t mode, int flags)
{
    typedef int (*orig_fchmodat_fn)(int, const char *, mode_t, int);
    static orig_fchmodat_fn orig_fchmodat = NULL;
    if (!orig_fchmodat) orig_fchmodat = (orig_fchmodat_fn)dlsym(RTLD_NEXT, "fchmodat");
    int res = orig_fchmodat ? orig_fchmodat(dirfd, pathname, mode, flags) : 0;
    if (res < 0 && (errno == EPERM || errno == EACCES)) {
        return 0;
    }
    return res;
}

static void add_iface(struct ifaddrs **head, struct ifaddrs **prev,
                      const char *name, const char *ip_str, const char *mask_str,
                      int is_loopback)
{
    struct ifaddrs *ifa = (struct ifaddrs *)calloc(1, sizeof(struct ifaddrs));
    if (!ifa) return;

    ifa->ifa_name = strdup(name);
    ifa->ifa_flags = IFF_UP | IFF_RUNNING | (is_loopback ? IFF_LOOPBACK : (IFF_BROADCAST | IFF_MULTICAST));

    struct sockaddr_in *addr = (struct sockaddr_in *)calloc(1, sizeof(struct sockaddr_in));
    if (addr) {
        addr->sin_family = AF_INET;
        inet_pton(AF_INET, ip_str, &addr->sin_addr);
        ifa->ifa_addr = (struct sockaddr *)addr;
    }

    struct sockaddr_in *mask = (struct sockaddr_in *)calloc(1, sizeof(struct sockaddr_in));
    if (mask) {
        mask->sin_family = AF_INET;
        inet_pton(AF_INET, mask_str, &mask->sin_addr);
        ifa->ifa_netmask = (struct sockaddr *)mask;
    }

    if (*prev) {
        (*prev)->ifa_next = ifa;
    } else {
        *head = ifa;
    }
    *prev = ifa;
}

int getifaddrs(struct ifaddrs **ifap)
{
    if (!ifap) return -1;
    *ifap = NULL;

    struct ifaddrs *head = NULL, *prev = NULL;

    const char *paths[] = {
        "/etc/cyclonedds/ifaddrs.txt",
        "/etc/ifaddrs.txt",
        "/data/data/org.botforge.rosdroid/files/rootfs/ubuntu/etc/cyclonedds/ifaddrs.txt",
        NULL
    };

    FILE *fp = NULL;
    for (int i = 0; paths[i] != NULL; i++) {
        fp = fopen(paths[i], "r");
        if (fp) break;
    }

    if (fp) {
        char line[256];
        char name[64], ip[64], mask[64];
        while (fgets(line, sizeof(line), fp)) {
            if (line[0] == '#' || line[0] == '\n') continue;
            mask[0] = '\0';
            int count = sscanf(line, "%63s %63s %63s", name, ip, mask);
            if (count >= 2) {
                if (count < 3 || strlen(mask) == 0) {
                    strcpy(mask, "255.255.255.0");
                }
                int is_loop = (strcmp(name, "lo") == 0);
                add_iface(&head, &prev, name, ip, mask, is_loop);
            }
        }
        fclose(fp);
    }

    // Always ensure wlan0 and lo are present
    if (!head) {
        add_iface(&head, &prev, "wlan0", "192.168.0.152", "255.255.255.0", 0);
        add_iface(&head, &prev, "lo", "127.0.0.1", "255.0.0.0", 1);
    }

    *ifap = head;
    return 0;
}

void freeifaddrs(struct ifaddrs *ifa)
{
    while (ifa) {
        struct ifaddrs *next = ifa->ifa_next;
        if (ifa->ifa_addr) free(ifa->ifa_addr);
        if (ifa->ifa_netmask) free(ifa->ifa_netmask);
        if (ifa->ifa_name) free(ifa->ifa_name);
        free(ifa);
        ifa = next;
    }
}

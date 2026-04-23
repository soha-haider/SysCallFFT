/*
 * CT-353 Operating Systems — Linux CCP
 * 15 System Calls + Parallel FFT (N=2048)
 * Compile: gcc linux_ccp.c -o ccp -lpthread -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/wait.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define N 2048

typedef struct { double real, imag; } Complex;
typedef struct { Complex *data; int len, start, end; } ThreadArg;

static int gLog = -1;

/* ── Helpers ─────────────────────────────────────────────── */
static void log_msg(const char *msg) {
    if (gLog < 0) return;
    write(gLog, msg, strlen(msg));
    write(gLog, "\n", 1);
}

static double elapsed(struct timespec s, struct timespec e) {
    return (e.tv_sec - s.tv_sec) + (e.tv_nsec - s.tv_nsec) / 1e9;
}

/* ── FFT ─────────────────────────────────────────────────── */
static void bit_reverse(Complex *x, int n) {
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { Complex t = x[i]; x[i] = x[j]; x[j] = t; }
    }
}

static void *fft_worker(void *arg) {
    ThreadArg *a = (ThreadArg *)arg;
    int half = a->len / 2;
    for (int b = a->start; b < a->end; b++) {
        int base = b * a->len;
        for (int j = 0; j < half; j++) {
            double ang = -2.0 * M_PI * j / a->len;
            double c = cos(ang), s = sin(ang);
            Complex u = a->data[base + j], v = a->data[base + j + half], t;
            t.real = c*v.real - s*v.imag;
            t.imag = s*v.real + c*v.imag;
            a->data[base + j].real        = u.real + t.real;
            a->data[base + j].imag        = u.imag + t.imag;
            a->data[base + j + half].real = u.real - t.real;
            a->data[base + j + half].imag = u.imag - t.imag;
        }
    }
    return NULL;
}

/* ── Main ────────────────────────────────────────────────── */
int main(void) {
    struct timespec t0, t1;
    double time_fileio=0, time_write=0, time_read=0, time_shm=0;
    double time_pipe=0,   time_mmap=0,  time_perm=0, time_proc=0, time_fft=0;
    size_t mmap_size = 0;
    char   pipe_msg[64] = {0};

    /* Log file using open() */
    gLog = open("ccp_log.txt", O_CREAT|O_TRUNC|O_WRONLY, 0644);
    if (gLog < 0) fprintf(stderr, "Warning: cannot open log\n");
    log_msg("=== CCP LOG START ===");

    /* ══ 1. open / write / read / close ══════════════════════
          Windows equivalent: CreateFile/WriteFile/ReadFile/CloseHandle */
    printf("[1/9] File I/O (open/write/read/close)...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    int fd = open("io_test.txt", O_CREAT|O_TRUNC|O_WRONLY, 0644);
    if (fd < 0) { log_msg("[FAIL] open(io_test.txt, O_WRONLY)"); return 1; }
    log_msg("[OK] open(io_test.txt, O_CREAT|O_WRONLY)");

    const char *io_data = "Linux CCP: open/write/read/close demo";
    if (write(fd, io_data, strlen(io_data)) < 0) { log_msg("[FAIL] write(io_test.txt)"); close(fd); return 1; }
    log_msg("[OK] write(io_test.txt)");

    if (close(fd) != 0) { log_msg("[FAIL] close(io_test.txt) after write"); return 1; }
    log_msg("[OK] close(io_test.txt) after write");

    fd = open("io_test.txt", O_RDONLY);
    if (fd < 0) { log_msg("[FAIL] open(io_test.txt, O_RDONLY)"); return 1; }
    log_msg("[OK] open(io_test.txt, O_RDONLY)");

    char io_buf[128] = {0};
    if (read(fd, io_buf, sizeof(io_buf)-1) < 0) { log_msg("[FAIL] read(io_test.txt)"); close(fd); return 1; }
    log_msg("[OK] read(io_test.txt)");

    if (close(fd) != 0) { log_msg("[FAIL] close(io_test.txt) after read"); return 1; }
    log_msg("[OK] close(io_test.txt) after read");

    unlink("io_test.txt");
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_fileio = elapsed(t0, t1);

    /* ══ 2. Write input.txt ══════════════════════════════════ */
    printf("[2/9] Write input.txt...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    fd = open("input.txt", O_CREAT|O_TRUNC|O_WRONLY, 0644);
    if (fd < 0) { log_msg("[FAIL] open(input.txt, O_WRONLY)"); return 1; }
    log_msg("[OK] open(input.txt, O_CREAT|O_WRONLY)");

    char *ibuf = malloc(N * 6 + 4);
    if (!ibuf) { log_msg("[FAIL] malloc ibuf"); return 1; }
    int ilen = 0;
    for (int i = 1; i <= N; i++) ilen += sprintf(ibuf + ilen, "%d ", i);

    if (write(fd, ibuf, ilen) < 0) { log_msg("[FAIL] write(input.txt)"); free(ibuf); close(fd); return 1; }
    log_msg("[OK] write(input.txt)");
    free(ibuf);

    if (close(fd) != 0) { log_msg("[FAIL] close(input.txt)"); return 1; }
    log_msg("[OK] close(input.txt)");
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_write = elapsed(t0, t1);

    /* ══ 3. pipe() IPC ═══════════════════════════════════════
          Windows equivalent: CreatePipe */
    printf("[3/9] Pipe IPC (pipe)...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    int pipefd[2];
    if (pipe(pipefd) != 0) { log_msg("[FAIL] pipe()"); return 1; }
    log_msg("[OK] pipe()");

    if (write(pipefd[1], "Pipe Message", 12) < 0) { log_msg("[FAIL] write(pipe)"); return 1; }
    log_msg("[OK] write(pipe write-end)");

    if (read(pipefd[0], pipe_msg, sizeof(pipe_msg)-1) < 0) { log_msg("[FAIL] read(pipe)"); return 1; }
    log_msg("[OK] read(pipe read-end)");

    close(pipefd[0]); close(pipefd[1]);
    log_msg("[OK] close(pipe fds)");
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_pipe = elapsed(t0, t1);

    /* ══ 4. mmap / munmap ════════════════════════════════════
          Windows equivalent: VirtualAlloc / VirtualFree */
    printf("[4/9] mmap / munmap...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    mmap_size = N * sizeof(int);
    int *mmap_mem = mmap(NULL, mmap_size, PROT_READ|PROT_WRITE,
                         MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
    if (mmap_mem == MAP_FAILED) { log_msg("[FAIL] mmap()"); return 1; }
    log_msg("[OK] mmap(NULL, N*sizeof(int), PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS)");

    for (int i = 0; i < N; i++) mmap_mem[i] = i;

    if (munmap(mmap_mem, mmap_size) != 0) { log_msg("[FAIL] munmap()"); return 1; }
    log_msg("[OK] munmap()");
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_mmap = elapsed(t0, t1);

    /* ══ 5. chmod / chown / umask ════════════════════════════
          Windows equivalent: SetFileSecurity */
    printf("[5/9] chmod / chown / umask...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    fd = open("perm_test.txt", O_CREAT|O_WRONLY|O_TRUNC, 0644);
    if (fd < 0) {
        log_msg("[FAIL] open(perm_test.txt)");
    } else {
        write(fd, "perm demo", 9);
        close(fd);
        log_msg("[OK] open + write(perm_test.txt)");

        if (chmod("perm_test.txt", S_IRUSR|S_IWUSR|S_IRGRP) == 0)
            log_msg("[OK] chmod(perm_test.txt, 0640)");
        else
            log_msg("[FAIL] chmod");

        if (chown("perm_test.txt", getuid(), getgid()) == 0)
            log_msg("[OK] chown(perm_test.txt, uid, gid)");
        else
            log_msg("[FAIL] chown (may need root)");

        mode_t old_mask = umask(0022);
        log_msg("[OK] umask(0022)");
        umask(old_mask);
        log_msg("[OK] umask restored");

        unlink("perm_test.txt");
        log_msg("[OK] unlink(perm_test.txt)");
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_perm = elapsed(t0, t1);

    /* ══ 6. fork / exec / wait ═══════════════════════════════
          Windows equivalent: CreateProcess / WaitForSingleObject */
    printf("[6/9] fork / exec / wait...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    pid_t pid = fork();
    if (pid < 0) {
        log_msg("[FAIL] fork()");
    } else if (pid == 0) {
        /* child: exec ls > process_demo.txt */
        int out = open("process_demo.txt", O_CREAT|O_TRUNC|O_WRONLY, 0644);
        if (out >= 0) { dup2(out, STDOUT_FILENO); close(out); }
        execlp("ls", "ls", "-1", (char *)NULL);
        _exit(1);
    } else {
        log_msg("[OK] fork() — child process created");
        int status;
        if (waitpid(pid, &status, 0) < 0) log_msg("[FAIL] waitpid()");
        else                               log_msg("[OK] waitpid() — child finished");
        unlink("process_demo.txt");
        log_msg("[OK] exec(ls) completed via child");
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_proc = elapsed(t0, t1);

    /* ══ 7. shmget / shmat ═══════════════════════════════════
          Windows equivalent: CreateFileMapping / MapViewOfFile */
    printf("[7/9] shmget / shmat (shared memory)...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    key_t shm_key = ftok("input.txt", 'F');
    int   shm_id  = shmget(shm_key, sizeof(Complex) * N, IPC_CREAT|0666);
    if (shm_id < 0) { log_msg("[FAIL] shmget()"); return 1; }
    log_msg("[OK] shmget(key, N*sizeof(Complex), IPC_CREAT|0666)");

    Complex *fft_data = (Complex *)shmat(shm_id, NULL, 0);
    if (fft_data == (Complex *)-1) {
        log_msg("[FAIL] shmat()");
        shmctl(shm_id, IPC_RMID, NULL);
        return 1;
    }
    log_msg("[OK] shmat(shm_id, NULL, 0)");
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_shm = elapsed(t0, t1);

    /* ══ 8. Read input.txt into shared memory ════════════════ */
    printf("[8/9] Read input.txt -> shared memory...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    fd = open("input.txt", O_RDONLY);
    if (fd < 0) { log_msg("[FAIL] open(input.txt, O_RDONLY)"); return 1; }
    log_msg("[OK] open(input.txt, O_RDONLY)");

    char *rbuf = malloc(N * 6 + 4);
    if (!rbuf) { log_msg("[FAIL] malloc rbuf"); return 1; }
    ssize_t bytes_read = read(fd, rbuf, N * 6);
    if (bytes_read < 0) { log_msg("[FAIL] read(input.txt)"); free(rbuf); close(fd); return 1; }
    rbuf[bytes_read] = '\0';
    log_msg("[OK] read(input.txt)");

    if (close(fd) != 0) { log_msg("[FAIL] close(input.txt)"); free(rbuf); return 1; }
    log_msg("[OK] close(input.txt) after read");

    char *tok = strtok(rbuf, " \t\r\n");
    for (int i = 0; i < N && tok; i++, tok = strtok(NULL, " \t\r\n")) {
        fft_data[i].real = atof(tok); fft_data[i].imag = 0.0;
    }
    free(rbuf);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_read = elapsed(t0, t1);

    /* ══ 9. Parallel FFT: pthread_create / pthread_join ══════
          Windows equivalent: CreateThread / WaitForMultipleObjects */
    printf("[9/9] Parallel FFT (pthreads)...\n");
    clock_gettime(CLOCK_MONOTONIC, &t0);

    bit_reverse(fft_data, N);
    for (int len = 2; len <= N; len <<= 1) {
        int blocks = N / len;
        int mid    = blocks / 2 < 1 ? 1 : blocks / 2;
        ThreadArg a1 = { fft_data, len, 0, mid }, a2 = { fft_data, len, mid, blocks };
        pthread_t th[2];

        if (pthread_create(&th[0], NULL, fft_worker, &a1) != 0) { log_msg("[FAIL] pthread_create worker 1"); return 1; }
        log_msg("[OK] pthread_create worker 1");

        if (pthread_create(&th[1], NULL, fft_worker, &a2) != 0) { log_msg("[FAIL] pthread_create worker 2"); pthread_join(th[0], NULL); return 1; }
        log_msg("[OK] pthread_create worker 2");

        if (pthread_join(th[0], NULL) != 0) log_msg("[FAIL] pthread_join worker 1");
        else                                log_msg("[OK] pthread_join worker 1");
        if (pthread_join(th[1], NULL) != 0) log_msg("[FAIL] pthread_join worker 2");
        else                                log_msg("[OK] pthread_join worker 2");
    }
    for (int i = 0; i < N; i++) { fft_data[i].real /= N; fft_data[i].imag /= N; }
    log_msg("[OK] FFT complete + normalized");
    clock_gettime(CLOCK_MONOTONIC, &t1);
    time_fft = elapsed(t0, t1);

    /* ══ Write output.txt ════════════════════════════════════ */
    fd = open("output.txt", O_CREAT|O_TRUNC|O_WRONLY, 0644);
    if (fd < 0) { log_msg("[FAIL] open(output.txt)"); return 1; }
    log_msg("[OK] open(output.txt)");

    char line[256]; int n;
#define WL(...) do { n = sprintf(line, __VA_ARGS__); write(fd, line, n); } while(0)

    WL("=== Linux CCP — CT-353 Operating Systems ===\n");
    WL("Pipe received : %s\n\n", pipe_msg);
    WL("=== SYSTEM CALLS (15) ===\n");
    WL("  File I/O    : open, read, write, close\n");
    WL("  Process     : fork, exec, wait\n");
    WL("  Memory      : mmap, munmap\n");
    WL("  IPC         : pipe, shmget, shmat\n");
    WL("  Permissions : chmod, chown, umask\n\n");
    WL("=== PERFORMANCE METRICS ===\n");
    WL("  File I/O   (open/write/read/close)     : %.6f sec\n", time_fileio);
    WL("  Input Write (open/write)               : %.6f sec\n", time_write);
    WL("  Input Read  (open/read)                : %.6f sec\n", time_read);
    WL("  Shared Mem  (shmget/shmat)             : %.6f sec\n", time_shm);
    WL("  Pipe IPC    (pipe)                     : %.6f sec\n", time_pipe);
    WL("  mmap/munmap                            : %.6f sec\n", time_mmap);
    WL("  Memory Mapped (mmap)                   : %zu bytes\n", mmap_size);
    WL("  Permissions (chmod/chown/umask)        : %.6f sec\n", time_perm);
    WL("  Process     (fork/exec/wait)           : %.6f sec\n", time_proc);
    WL("  FFT (parallel pthreads/stage)          : %.6f sec\n\n", time_fft);
    WL("=== FFT OUTPUT (N=%d) ===\n", N);

    for (int i = 0; i < N; i++) {
        n = sprintf(line, "X[%4d] = %10.4f %+10.4fi\n", i, fft_data[i].real, fft_data[i].imag);
        write(fd, line, n);
    }
    close(fd);
    log_msg("[OK] output.txt written and closed");

    /* ══ Cleanup shared memory ═══════════════════════════════ */
    if (shmdt(fft_data) != 0) log_msg("[FAIL] shmdt()");
    else                       log_msg("[OK] shmdt()");
    if (shmctl(shm_id, IPC_RMID, NULL) != 0) log_msg("[FAIL] shmctl(IPC_RMID)");
    else                                       log_msg("[OK] shmctl(IPC_RMID) — shm removed");

    /* ══ Console summary ═════════════════════════════════════ */
    printf("\n=== Performance Summary ===\n");
    printf("  Pipe received : %s\n\n",    pipe_msg);
    printf("  File I/O      : %.6f sec\n", time_fileio);
    printf("  Input write   : %.6f sec\n", time_write);
    printf("  Input read    : %.6f sec\n", time_read);
    printf("  Shared mem    : %.6f sec\n", time_shm);
    printf("  Pipe IPC      : %.6f sec\n", time_pipe);
    printf("  mmap/munmap   : %.6f sec  (%zu bytes)\n", time_mmap, mmap_size);
    printf("  Permissions   : %.6f sec\n", time_perm);
    printf("  fork/exec/wait: %.6f sec\n", time_proc);
    printf("  FFT           : %.6f sec\n", time_fft);
    printf("\nFiles ready: input.txt  output.txt  ccp_log.txt\n");

    log_msg("=== CCP LOG END ===");
    close(gLog);
    return 0;
}

/*
 * CT-353 Operating Systems — Windows CCP
 * 15 System Calls + Parallel FFT (N=1024)
 * Compile (MinGW): gcc windows_ccp.c -o ccp.exe -ladvapi32 -lm
 * Compile (MSVC):  cl windows_ccp.c /Fe:ccp.exe /link advapi32.lib
 *
 * ┌─────────────┬──────────────────────────────────────────────┬─────────────────────────┐
 * │  Category   │  Windows (this file)                         │  Linux equivalent       │
 * ├─────────────┼──────────────────────────────────────────────┼─────────────────────────┤
 * │  File I/O   │  CreateFile, ReadFile, WriteFile, CloseHandle│  open, read,            │
 * │             │                                              │  write, close           │
 * │  CRT I/O    │  _open, _write, _read, _close               │  open, write,           │
 * │             │                                              │  read, close            │
 * │  Process    │  CreateProcess, WaitForSingleObject          │  fork, execlp, waitpid  │
 * │  Memory     │  VirtualAlloc, VirtualFree                   │  mmap, munmap           │
 * │  IPC        │  CreatePipe, CreateFileMapping, MapViewOfFile│  pipe, shmget, shmat    │
 * │  Permissions│  SetFileSecurity                             │  chmod, chown, umask    │
 * │  Threading  │  CreateThread, WaitForMultipleObjects        │  pthread_create,        │
 * │             │                                              │  pthread_join           │
 * └─────────────┴──────────────────────────────────────────────┴─────────────────────────┘
 */

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define N 1024

typedef struct { double real, imag; } Complex;
typedef struct { Complex *data; int len, start, end; } ThreadArg;

static HANDLE gLog = INVALID_HANDLE_VALUE;

/* ── Helpers ─────────────────────────────────────────────── */
static void log_msg(const char *msg) {
    if (gLog == INVALID_HANDLE_VALUE) return;
    DWORD w;
    WriteFile(gLog, msg, (DWORD)strlen(msg), &w, NULL);
    WriteFile(gLog, "\n", 1, &w, NULL);
}

static double elapsed(LARGE_INTEGER s, LARGE_INTEGER e, LARGE_INTEGER f) {
    return (double)(e.QuadPart - s.QuadPart) / (double)f.QuadPart;
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

/* Windows thread worker — mirrors static void *fft_worker() on Linux */
DWORD WINAPI fft_worker(LPVOID arg) {
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
    return 0;
}

/* ── Main ────────────────────────────────────────────────── */
int main(void) {
    LARGE_INTEGER freq, t0, t1;
    QueryPerformanceFrequency(&freq);

    double time_crt   = 0, time_write = 0, time_read = 0, time_map  = 0;
    double time_pipe  = 0, time_vm    = 0, time_perm = 0, time_proc = 0;
    double time_fft   = 0;
    SIZE_T vm_bytes   = 0;
    char   pipe_msg[64] = {0};
    DWORD  written      = 0;

    /* Log file */
    gLog = CreateFileA("ccp_log.txt", GENERIC_WRITE, 0, NULL,
                       CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (gLog == INVALID_HANDLE_VALUE) fprintf(stderr, "Warning: log failed\n");
    log_msg("=== CCP LOG START ===");

    /* ══ 1. CRT I/O: _open / _write / _read / _close ════════
          SC #1 _open  SC #2 _write  SC #3 _read  SC #4 _close
          Linux equivalent: open() / write() / read() / close() */
    printf("[1/9] CRT File I/O...\n");
    QueryPerformanceCounter(&t0);

    int fd = _open("crt_test.txt", _O_CREAT|_O_TRUNC|_O_WRONLY|_O_BINARY, _S_IREAD|_S_IWRITE);
    if (fd < 0) { log_msg("[FAIL] _open"); return 1; }
    log_msg("[OK] _open(crt_test.txt, O_CREAT|O_WRONLY)");

    const char *crt_data = "CRT demo: _open/_write/_read/_close";
    if (_write(fd, crt_data, (unsigned)strlen(crt_data)) < 0) { log_msg("[FAIL] _write"); _close(fd); return 1; }
    log_msg("[OK] _write(crt_test.txt)");

    if (_close(fd) != 0) { log_msg("[FAIL] _close after write"); return 1; }
    log_msg("[OK] _close(crt_test.txt) after write");

    fd = _open("crt_test.txt", _O_RDONLY|_O_BINARY);
    if (fd < 0) { log_msg("[FAIL] _open for read"); return 1; }
    log_msg("[OK] _open(crt_test.txt, O_RDONLY)");

    char crt_buf[128] = {0};
    if (_read(fd, crt_buf, sizeof(crt_buf)-1) < 0) { log_msg("[FAIL] _read"); _close(fd); return 1; }
    log_msg("[OK] _read(crt_test.txt)");

    if (_close(fd) != 0) { log_msg("[FAIL] _close after read"); return 1; }
    log_msg("[OK] _close(crt_test.txt) after read");

    DeleteFileA("crt_test.txt");
    QueryPerformanceCounter(&t1);
    time_crt = elapsed(t0, t1, freq);

    /* ══ 2. CreateFile / WriteFile / CloseHandle ═════════════
          SC #5 CreateFile  SC #6 WriteFile  SC #7 CloseHandle
          Linux equivalent: open() / write() / close() */
    printf("[2/9] Write input.txt...\n");
    QueryPerformanceCounter(&t0);

    HANDLE hf = CreateFileA("input.txt", GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hf == INVALID_HANDLE_VALUE) { log_msg("[FAIL] CreateFileA(input.txt write)"); return 1; }
    log_msg("[OK] CreateFileA(input.txt, GENERIC_WRITE)");

    char *ibuf = (char *)malloc(N * 6 + 4);
    if (!ibuf) { log_msg("[FAIL] malloc"); return 1; }
    int ilen = 0;
    for (int i = 1; i <= N; i++) ilen += sprintf(ibuf + ilen, "%d ", i);

    if (!WriteFile(hf, ibuf, (DWORD)ilen, &written, NULL)) { log_msg("[FAIL] WriteFile(input.txt)"); free(ibuf); CloseHandle(hf); return 1; }
    log_msg("[OK] WriteFile(input.txt)");
    free(ibuf);

    if (!CloseHandle(hf)) { log_msg("[FAIL] CloseHandle(input.txt)"); return 1; }
    log_msg("[OK] CloseHandle(input.txt)");

    QueryPerformanceCounter(&t1);
    time_write = elapsed(t0, t1, freq);

    /* ══ 3. CreatePipe / WriteFile / ReadFile ════════════════
          SC #8 CreatePipe
          Linux equivalent: pipe() */
    printf("[3/9] Pipe IPC...\n");
    QueryPerformanceCounter(&t0);

    SECURITY_ATTRIBUTES sa = { sizeof(sa), NULL, TRUE };
    HANDLE hRead, hWrite;
    if (!CreatePipe(&hRead, &hWrite, &sa, 0)) { log_msg("[FAIL] CreatePipe"); return 1; }
    log_msg("[OK] CreatePipe");

    DWORD rb = 0;
    if (!WriteFile(hWrite, "Pipe Message", 12, &written, NULL)) { log_msg("[FAIL] WriteFile(pipe)"); CloseHandle(hRead); CloseHandle(hWrite); return 1; }
    log_msg("[OK] WriteFile(pipe)");

    if (!ReadFile(hRead, pipe_msg, sizeof(pipe_msg)-1, &rb, NULL)) { log_msg("[FAIL] ReadFile(pipe)"); CloseHandle(hRead); CloseHandle(hWrite); return 1; }
    log_msg("[OK] ReadFile(pipe)");

    CloseHandle(hRead); CloseHandle(hWrite);
    log_msg("[OK] CloseHandle(pipe ends)");
    QueryPerformanceCounter(&t1);
    time_pipe = elapsed(t0, t1, freq);

    /* ══ 4. VirtualAlloc / VirtualFree ══════════════════════
          SC #9 VirtualAlloc  SC #10 VirtualFree
          Linux equivalent: mmap() / munmap() */
    printf("[4/9] VirtualAlloc/Free...\n");
    QueryPerformanceCounter(&t0);

    vm_bytes = N * sizeof(int);
    int *vmem = (int *)VirtualAlloc(NULL, vm_bytes, MEM_COMMIT|MEM_RESERVE, PAGE_READWRITE);
    if (!vmem) { log_msg("[FAIL] VirtualAlloc"); return 1; }
    log_msg("[OK] VirtualAlloc");

    for (int i = 0; i < N; i++) vmem[i] = i;

    if (!VirtualFree(vmem, 0, MEM_RELEASE)) { log_msg("[FAIL] VirtualFree"); return 1; }
    log_msg("[OK] VirtualFree");
    QueryPerformanceCounter(&t1);
    time_vm = elapsed(t0, t1, freq);

    /* ══ 5. SetFileSecurity ══════════════════════════════════
          SC #11 SetFileSecurity
          Linux equivalent: chmod() / chown() / umask() */
    printf("[5/9] SetFileSecurity...\n");
    QueryPerformanceCounter(&t0);

    HANDLE hp = CreateFileA("perm_test.txt", GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hp == INVALID_HANDLE_VALUE) {
        log_msg("[FAIL] CreateFileA(perm_test.txt)");
    } else {
        if (!WriteFile(hp, "perm demo", 9, &written, NULL)) log_msg("[FAIL] WriteFile(perm_test.txt)");
        else log_msg("[OK] WriteFile(perm_test.txt)");
        CloseHandle(hp);
        log_msg("[OK] CloseHandle(perm_test.txt)");

        SECURITY_DESCRIPTOR sd;
        if (InitializeSecurityDescriptor(&sd, SECURITY_DESCRIPTOR_REVISION))
            log_msg("[OK] InitializeSecurityDescriptor");
        if (SetSecurityDescriptorDacl(&sd, TRUE, NULL, FALSE))
            log_msg("[OK] SetSecurityDescriptorDacl");

        if (SetFileSecurityA("perm_test.txt", DACL_SECURITY_INFORMATION, &sd))
            log_msg("[OK] SetFileSecurityA(perm_test.txt)");
        else
            log_msg("[FAIL] SetFileSecurityA (may need elevation)");

        DeleteFileA("perm_test.txt");
        log_msg("[OK] DeleteFileA(perm_test.txt)");
    }
    QueryPerformanceCounter(&t1);
    time_perm = elapsed(t0, t1, freq);

    /* ══ 6. CreateProcess / WaitForSingleObject ══════════════
          SC #12 CreateProcess  SC #13 WaitForSingleObject
          Linux equivalent: fork() / execlp() / waitpid() */
    printf("[6/9] CreateProcess...\n");
    QueryPerformanceCounter(&t0);

    STARTUPINFOA si; ZeroMemory(&si, sizeof(si)); si.cb = sizeof(si);
    PROCESS_INFORMATION pi; ZeroMemory(&pi, sizeof(pi));
    char cmd[] = "cmd.exe /C echo process_demo > process_demo.txt";

    if (!CreateProcessA(NULL, cmd, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
        log_msg("[FAIL] CreateProcessA");
    } else {
        log_msg("[OK] CreateProcessA(cmd.exe)");
        DWORD wr = WaitForSingleObject(pi.hProcess, INFINITE);
        if (wr == WAIT_FAILED) log_msg("[FAIL] WaitForSingleObject");
        else                   log_msg("[OK] WaitForSingleObject");
        CloseHandle(pi.hThread); CloseHandle(pi.hProcess);
        log_msg("[OK] CloseHandle(process/thread)");
        DeleteFileA("process_demo.txt");
    }
    QueryPerformanceCounter(&t1);
    time_proc = elapsed(t0, t1, freq);

    /* ══ 7. CreateFileMapping / MapViewOfFile ════════════════
          SC #14 CreateFileMapping  SC #15 MapViewOfFile
          Linux equivalent: shmget() / shmat() */
    printf("[7/9] Shared memory map...\n");
    QueryPerformanceCounter(&t0);

    HANDLE hmap = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,
                                     (DWORD)(sizeof(Complex) * N), "FFT_MAP");
    if (!hmap) { log_msg("[FAIL] CreateFileMappingA"); return 1; }
    log_msg("[OK] CreateFileMappingA(FFT_MAP)");

    Complex *fft_data = (Complex *)MapViewOfFile(hmap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (!fft_data) { log_msg("[FAIL] MapViewOfFile"); CloseHandle(hmap); return 1; }
    log_msg("[OK] MapViewOfFile(FFT_MAP)");
    QueryPerformanceCounter(&t1);
    time_map = elapsed(t0, t1, freq);

    /* ══ 8. CreateFile / ReadFile → shared memory ════════════
          Linux equivalent: open() / read() */
    printf("[8/9] Read input.txt -> shared memory...\n");
    QueryPerformanceCounter(&t0);

    hf = CreateFileA("input.txt", GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hf == INVALID_HANDLE_VALUE) { log_msg("[FAIL] CreateFileA(input.txt read)"); return 1; }
    log_msg("[OK] CreateFileA(input.txt, GENERIC_READ)");

    char *rbuf = (char *)malloc(N * 6 + 4);
    if (!rbuf) { log_msg("[FAIL] malloc rbuf"); return 1; }
    DWORD bytesRead = 0;

    if (!ReadFile(hf, rbuf, N * 6, &bytesRead, NULL)) { log_msg("[FAIL] ReadFile(input.txt)"); free(rbuf); CloseHandle(hf); return 1; }
    rbuf[bytesRead] = '\0';
    log_msg("[OK] ReadFile(input.txt)");

    if (!CloseHandle(hf)) { log_msg("[FAIL] CloseHandle(input.txt read)"); free(rbuf); return 1; }
    log_msg("[OK] CloseHandle(input.txt) after read");

    char *tok = strtok(rbuf, " \t\r\n");
    for (int i = 0; i < N && tok; i++, tok = strtok(NULL, " \t\r\n")) {
        fft_data[i].real = atof(tok); fft_data[i].imag = 0.0;
    }
    free(rbuf);
    QueryPerformanceCounter(&t1);
    time_read = elapsed(t0, t1, freq);

    /* ══ 9. Parallel FFT: CreateThread / WaitForMultipleObjects
          Linux equivalent: pthread_create() / pthread_join() */
    printf("[9/9] Parallel FFT...\n");
    QueryPerformanceCounter(&t0);

    bit_reverse(fft_data, N);
    for (int len = 2; len <= N; len <<= 1) {
        int blocks = N / len;
        int mid    = blocks / 2 < 1 ? 1 : blocks / 2;
        ThreadArg a1 = { fft_data, len, 0, mid }, a2 = { fft_data, len, mid, blocks };
        HANDLE th[2];

        th[0] = CreateThread(NULL, 0, fft_worker, &a1, 0, NULL);
        if (!th[0]) { log_msg("[FAIL] CreateThread worker 1"); return 1; }
        log_msg("[OK] CreateThread worker 1");

        th[1] = CreateThread(NULL, 0, fft_worker, &a2, 0, NULL);
        if (!th[1]) { log_msg("[FAIL] CreateThread worker 2"); CloseHandle(th[0]); return 1; }
        log_msg("[OK] CreateThread worker 2");

        DWORD wres = WaitForMultipleObjects(2, th, TRUE, INFINITE);
        if (wres == WAIT_FAILED) log_msg("[FAIL] WaitForMultipleObjects");
        else                     log_msg("[OK] WaitForMultipleObjects");

        CloseHandle(th[0]); CloseHandle(th[1]);
    }
    for (int i = 0; i < N; i++) { fft_data[i].real /= N; fft_data[i].imag /= N; }
    log_msg("[OK] FFT complete + normalized");
    QueryPerformanceCounter(&t1);
    time_fft = elapsed(t0, t1, freq);

    /* ══ Write output.txt ════════════════════════════════════ */
    hf = CreateFileA("output.txt", GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hf == INVALID_HANDLE_VALUE) { log_msg("[FAIL] CreateFileA(output.txt)"); return 1; }
    log_msg("[OK] CreateFileA(output.txt)");

    char line[256]; int n;

#define WL(...) do { n = sprintf(line, __VA_ARGS__); WriteFile(hf, line, n, &written, NULL); } while(0)
    WL("=========================================");
    WL("\nWindows CCP — CT-353 Operating Systems\r\n");
    WL("=========================================");
    WL("\nInput Signal: Sequential integers from 1 to %d (N=%d)\r\n", N, N);
    WL("Pipe received : %s\r\n\r\n", pipe_msg);
    WL("\n=======================");
    WL("\nSYSTEM CALLS USED (15) \r\n");
    WL("=======================\n");
    WL("  File I/O    : CreateFile, ReadFile, WriteFile, CloseHandle\r\n");
    WL("  CRT I/O     : _open, _read, _write, _close\r\n");
    WL("  Process     : CreateProcess, WaitForSingleObject\r\n");
    WL("  Memory      : VirtualAlloc, VirtualFree\r\n");
    WL("  IPC         : CreatePipe, CreateFileMapping, MapViewOfFile\r\n");
    WL("  Permissions : SetFileSecurity\r\n\r\n");
    WL("\n=====================\n");
    WL("PERFORMANCE METRICS \r\n");
    WL("=====================\n");
    WL("  CRT I/O   (_open/_write/_read/_close) : %.6f seconds\r\n", time_crt);
    WL("  Input Write (CreateFile/WriteFile)     : %.6f seconds\r\n", time_write);
    WL("  Input Read  (CreateFile/ReadFile)      : %.6f seconds\r\n", time_read);
    WL("  Shared Map  (CreateFileMapping/MapView): %.6f seconds\r\n", time_map);
    WL("  Pipe IPC    (CreatePipe)               : %.6f seconds\r\n", time_pipe);
    WL("  VirtualAlloc/Free                      : %.6f seconds\r\n", time_vm);
    WL("  Memory Allocated (VirtualAlloc)        : %zu bytes (%.1f KB)\r\n", vm_bytes, vm_bytes / 1024.0);
    WL("  SetFileSecurity                        : %.6f seconds\r\n", time_perm);
    WL("  CreateProcess/Wait                     : %.6f seconds\r\n", time_proc);
    WL("  FFT (parallel, 2 threads/stage)        : %.6f seconds\r\n\r\n", time_fft);

    WL("\n==================================================\n");
    WL("FFT OUTPUT (N=%d, first 20 and last 20 bins)\r", N);
    WL("==================================================\n");
    WL("  %-8s  %-12s  %-12s  %-12s\r\n", "k", "Real", "Imaginary", "Magnitude");
    WL("  %-8s  %-12s  %-12s  %-12s\r\n", "--------","------------","------------","------------");

    /* First 20 */
    for (int i = 0; i < 20; i++) {
        double mag = sqrt(fft_data[i].real * fft_data[i].real +
                          fft_data[i].imag * fft_data[i].imag);
        n = sprintf(line, "  %-8d  %12.4f  %+12.4fi  %12.4f\r\n",
                    i, fft_data[i].real, fft_data[i].imag, mag);
        WriteFile(hf, line, n, &written, NULL);
    }

    /* Separator */
    WL("  %-8s  %-12s  %-12s  %-12s\r\n", "...", "...", "...", "...");

    /* Last 20 */
    for (int i = N-20; i < N; i++) {
        double mag = sqrt(fft_data[i].real * fft_data[i].real +
                          fft_data[i].imag * fft_data[i].imag);
        n = sprintf(line, "  %-8d  %12.4f  %+12.4fi  %12.4f\r\n",
                    i, fft_data[i].real, fft_data[i].imag, mag);
        WriteFile(hf, line, n, &written, NULL);
    }

    WL("\r\n  Note: Showing first 20 and last 20 of %d total bins.\r\n", N);
    CloseHandle(hf);
    log_msg("[OK] output.txt written");

    /* ══ Cleanup ═════════════════════════════════════════════ */
    if (!UnmapViewOfFile(fft_data)) log_msg("[FAIL] UnmapViewOfFile");
    else                            log_msg("[OK] UnmapViewOfFile");
    if (!CloseHandle(hmap))         log_msg("[FAIL] CloseHandle(FFT_MAP)");
    else                            log_msg("[OK] CloseHandle(FFT_MAP)");

    /* ══ Console summary ═════════════════════════════════════ */
    printf("--------------------------\n");
    printf(" Performance Summary\n");
    printf("--------------------------\n");
    printf("  Pipe received : %s\n\n",    pipe_msg);
    printf("  CRT I/O       : %.6f seconds\n", time_crt);
    printf("  Input write   : %.6f seconds\n", time_write);
    printf("  Input read    : %.6f seconds\n", time_read);
    printf("  Shared map    : %.6f seconds\n", time_map);
    printf("  Pipe IPC      : %.6f seconds\n", time_pipe);
    printf("  VirtualAlloc  : %.6f sec  (%zu bytes / %.1f KB)\n", time_vm, vm_bytes, vm_bytes / 1024.0);
    printf("  Permissions   : %.6f seconds\n", time_perm);
    printf("  CreateProcess : %.6f seconds\n", time_proc);
    printf("  FFT           : %.6f seconds\n", time_fft);
    printf("\nFILES ARE READY: input.txt  output.txt  ccp_log.txt\n");
    printf("-------------------\n");
    log_msg("CCP LOG END ");
    printf("-------------------\n");

    /* Total CPU time — mirrors getrusage() on Linux */
    FILETIME ct, et, kt, ut;
    if (GetProcessTimes(GetCurrentProcess(), &ct, &et, &kt, &ut)) {
        ULARGE_INTEGER k, u;
        k.LowPart = kt.dwLowDateTime; k.HighPart = kt.dwHighDateTime;
        u.LowPart = ut.dwLowDateTime; u.HighPart = ut.dwHighDateTime;
        double cpu_sec = (double)(k.QuadPart + u.QuadPart) / 1.0e7;
        printf("\n Total CPU Time : %.6f seconds\n", cpu_sec);
    }

    CloseHandle(gLog);
    return 0;
}

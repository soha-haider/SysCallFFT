# OS System Calls & Parallel FFT
A Comparative Study of Linux and Windows System Calls via Fourier Transform

A cross-platform systems programming project in **C** that demonstrates 15 OS-level system calls on both **Windows** and **Linux**, integrated with a **parallelized Fast Fourier Transform (FFT)** pipeline using shared memory, threads/processes, and IPC.

---

## Overview

This project implements and benchmarks core operating system primitives across two platforms — Windows (Win32 API) and Linux (POSIX) — and uses those primitives to build a complete FFT computation pipeline:

- Read input signal from file using OS file I/O calls
- Store the FFT working array in shared/mapped memory
- Compute the FFT in parallel using OS threads
- Write the output spectrum to a file
- Measure and report execution time for every system call category

---

## Authors
 
This project was built collaboratively by a group of three as part of a systems programming study on OS internals.
 
1. Soha
2. Ramaize Shahab
3. Muhammad Waleed Kashif
   
---

## System Calls Covered

| Category | Windows | Linux |
|---|---|---|
| File I/O | `CreateFile`, `ReadFile`, `WriteFile`, `CloseHandle` | `open`, `read`, `write`, `close` |
| Process | `CreateProcess`, `WaitForSingleObject` | `fork`, `exec`, `waitpid` |
| Memory | `VirtualAlloc`, `VirtualFree` | `mmap`, `munmap` |
| IPC (pipe) | `CreatePipe` | `pipe` |
| IPC (shared mem) | `CreateFileMapping`, `MapViewOfFile` | `shmget`, `shmat` |
| Permissions | `SetFileSecurity` | `chmod`, `chown`, `umask` |
| Threads | `CreateThread`, `WaitForMultipleObjects` | `pthread_create`, `pthread_join` |

---

## FFT Pipeline

```
input.txt  ──►  ReadFile / read()
                    │
                    ▼
           Shared Memory / mmap
           (CreateFileMapping / shmget)
                    │
                    ▼
         Parallel FFT (N=2048)
         ┌──────────┬──────────┐
         │ Thread 1 │ Thread 2 │   ← per butterfly stage
         └──────────┴──────────┘
                    │
                    ▼
              normalize ÷ N
                    │
                    ▼
           output.txt  +  ccp_log.txt
```

Each FFT stage splits its butterfly blocks across **2 threads** (`CreateThread` / `pthread_create`), then joins them before advancing to the next stage — giving true parallel Cooley-Tukey decomposition.

---

## Build & Run

### Windows

**MinGW / GCC:**
```bash
gcc windows_ccp.c -o ccp.exe -ladvapi32 -lm
./ccp.exe
```

**MSVC:**
```bash
cl windows_ccp.c /Fe:ccp.exe /link advapi32.lib
ccp.exe
```

### Linux

```bash
gcc linux_ccp.c -o ccp -lpthread -lm
./ccp
```

---

## Output Files

| File | Contents |
|---|---|
| `input.txt` | Input signal written by the program (values 1 to N) |
| `output.txt` | System calls list, performance metrics, all N FFT bins |
| `ccp_log.txt` | Per-call `[OK]` / `[FAIL]` log for every system call |

### Sample output.txt

```
=== Windows CCP — CT-353 Operating Systems ===
Pipe received : Pipe Message

=== SYSTEM CALLS (15) ===
  File I/O    : CreateFile, ReadFile, WriteFile, CloseHandle
  CRT I/O     : _open, _read, _write, _close
  Process     : CreateProcess, WaitForSingleObject
  Memory      : VirtualAlloc, VirtualFree
  IPC         : CreatePipe, CreateFileMapping, MapViewOfFile
  Permissions : SetFileSecurity

=== PERFORMANCE METRICS ===
  CRT I/O   (_open/_write/_read/_close) : 0.120109 sec
  Input Write (CreateFile/WriteFile)     : 0.003199 sec
  Input Read  (CreateFile/ReadFile)      : 0.002585 sec
  Shared Map  (CreateFileMapping/MapView): 0.000560 sec
  Pipe IPC    (CreatePipe)               : 0.000761 sec
  VirtualAlloc/Free                      : 0.000244 sec
  Memory Allocated (VirtualAlloc)        : 8192 bytes
  SetFileSecurity                        : 0.001844 sec
  CreateProcess/Wait                     : 0.377326 sec
  FFT (parallel, 2 threads/stage)        : 0.009209 sec

=== FFT OUTPUT (N=2048) ===
X[   0] =  1024.5000    +0.0000i
X[   1] =    -0.5000  +325.9491i
...
```

---

## Windows vs Linux Comparison (draft for now)

| Operation | Windows API | Linux POSIX | Key Difference |
|---|---|---|---|
| File open | `CreateFile` (1 call, many flags) | `open` (simpler flags) | Win32 is more verbose |
| Shared memory | `CreateFileMapping` + `MapViewOfFile` | `shmget` + `shmat` | Linux uses System V IPC keys; Windows uses named objects |
| Process creation | Single `CreateProcess` | `fork` + `exec` (2 steps) | Linux separates spawn from replace |
| Memory mapping | `VirtualAlloc` (page-granular) | `mmap` (flexible, file-backed or anonymous) | `mmap` is more general |
| Permissions | `SetFileSecurity` (ACL-based) | `chmod` / `chown` / `umask` (Unix bits) | Linux model is simpler |
| Threads | `CreateThread` / `WaitForMultipleObjects` | `pthread_create` / `pthread_join` | POSIX threads are portable across Unix systems |
| Timing | `QueryPerformanceCounter` | `clock_gettime(CLOCK_MONOTONIC)` | Both give nanosecond resolution |
| Error reporting | `GetLastError()` | `errno` | Different conventions |

---

## Error Handling

Every system call is wrapped with an explicit return-value check. On failure:
- A `[FAIL]` entry is written to `ccp_log.txt` with context
- The program exits cleanly, freeing any allocated resources before returning

Example log entries:
```
[OK] CreateFileMappingA(FFT_MAP)
[OK] MapViewOfFile(FFT_MAP)
[OK] CreateThread worker 1
[OK] CreateThread worker 2
[OK] WaitForMultipleObjects
[OK] FFT complete + normalized
```

---

## Performance Notes

- **`CreateProcess` / `fork+exec`** are the slowest operations on both platforms (~0.3–0.5 sec) due to process creation overhead
- **Pipe IPC** and **shared memory setup** are both sub-millisecond
- **FFT** on N=2048 completes in under 10 ms with 2-thread parallelism per stage
- **CRT `_open`** on Windows can be slower than Win32 `CreateFile` due to the CRT translation layer

---

## Requirements

| Platform | Requirement |
|---|---|
| Windows | GCC (MinGW-w64) or MSVC, `advapi32.lib` |
| Linux | GCC, `libpthread`, `libm` |
| Both | C99 or later |

---

## License

This project is open-source and available under the MIT License.

---

## Contributing

Contributions are welcome! Feel free to submit issues, feature requests, or pull requests to improve CyberEye.

---

# **Two OS. Fifteen system calls. One parallel FFT.**

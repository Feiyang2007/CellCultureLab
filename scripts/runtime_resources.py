"""Read-only Windows commit/private-memory and optional NVIDIA GPU snapshots."""
import ctypes
from ctypes import wintypes
from datetime import datetime,timezone
import os
import subprocess
import time


def snapshot(label,include_gpu=True):
    result=dict(label=label,pid=os.getpid(),timestamp_utc=datetime.now(timezone.utc).isoformat(),monotonic_seconds=time.perf_counter())
    try:
        class PerformanceInfo(ctypes.Structure):
            _fields_=[("cb",wintypes.DWORD)]+[(name,ctypes.c_size_t) for name in
                ("CommitTotal","CommitLimit","CommitPeak","PhysicalTotal","PhysicalAvailable","SystemCache",
                 "KernelTotal","KernelPaged","KernelNonpaged","PageSize")]+[(name,wintypes.DWORD) for name in
                ("HandleCount","ProcessCount","ThreadCount")]
        class ProcessMemory(ctypes.Structure):
            _fields_=[("cb",wintypes.DWORD),("PageFaultCount",wintypes.DWORD)]+[(name,ctypes.c_size_t) for name in
                ("PeakWorkingSetSize","WorkingSetSize","QuotaPeakPagedPoolUsage","QuotaPagedPoolUsage",
                 "QuotaPeakNonPagedPoolUsage","QuotaNonPagedPoolUsage","PagefileUsage","PeakPagefileUsage","PrivateUsage")]
        psapi=ctypes.WinDLL("psapi",use_last_error=True)
        kernel=ctypes.WinDLL("kernel32",use_last_error=True)
        info=PerformanceInfo()
        info.cb=ctypes.sizeof(info)
        psapi.GetPerformanceInfo.argtypes=[ctypes.POINTER(PerformanceInfo),wintypes.DWORD]
        if not psapi.GetPerformanceInfo(ctypes.byref(info),info.cb):
            raise ctypes.WinError(ctypes.get_last_error())
        memory=ProcessMemory()
        memory.cb=ctypes.sizeof(memory)
        kernel.GetCurrentProcess.restype=wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes=[wintypes.HANDLE,ctypes.POINTER(ProcessMemory),wintypes.DWORD]
        if not psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(),ctypes.byref(memory),memory.cb):
            raise ctypes.WinError(ctypes.get_last_error())
        result.update(system_commit_bytes=info.CommitTotal*info.PageSize,system_commit_limit_bytes=info.CommitLimit*info.PageSize,
                      process_private_bytes=memory.PrivateUsage,process_working_set_bytes=memory.WorkingSetSize)
    except Exception as error:
        result["memory_read_error"]=repr(error)
    if include_gpu:
        try:
            query=subprocess.run(["nvidia-smi","--query-gpu=index,memory.used,memory.total,utilization.gpu",
                "--format=csv,noheader,nounits"],capture_output=True,text=True,timeout=4,check=True,
                creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
            result["gpu"]=[dict(zip(("index","used_mib","total_mib","utilization_percent"),
                [int(x.strip()) for x in line.split(",")])) for line in query.stdout.strip().splitlines()]
        except Exception as error:
            result["gpu_read_error"]=repr(error)
    return result

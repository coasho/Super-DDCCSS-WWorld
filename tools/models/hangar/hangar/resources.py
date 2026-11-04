"""Bound nested native work within a fleet model's CPU reservation."""
import os
from contextlib import contextmanager
from contextvars import ContextVar
import math
from pathlib import Path
import subprocess
import threading
import time

_RESERVATION=ContextVar('hangar_native_thread_reservation',default=None)
_CPU_LEASE=ContextVar('hangar_os_cpu_lease',default=None)
_STARTUP=threading.local()


@contextmanager
def thread_reservation(cap):
    """An explicit phase reservation; nested scopes may only reduce it."""
    if type(cap) is not int or not 1<=cap<=128:raise ValueError('Native thread reservation must be an integer from 1 to 128')
    previous=_RESERVATION.get()
    token=_RESERVATION.set(min(previous,cap) if previous is not None else cap)
    try:yield
    finally:_RESERVATION.reset(token)


def threads(requested=0):
    value=os.environ.get('FSIM_HANGAR_THREADS')
    if value is not None and (not value.isdecimal() or not 1<=int(value)<=128):
        raise ValueError('FSIM_HANGAR_THREADS must be an integer from 1 to 128')
    reservation=_RESERVATION.get()
    if reservation is not None:return min(requested,reservation) if requested>0 else reservation
    if value is None:return requested
    cap=int(value)
    return min(requested,cap) if requested>0 else cap


def _windows():
    import ctypes as c
    k=c.WinDLL('kernel32',use_last_error=True)
    k.GetCurrentProcess.restype=c.c_void_p
    k.GetProcessAffinityMask.argtypes=[c.c_void_p,c.POINTER(c.c_size_t),c.POINTER(c.c_size_t)]
    k.SetProcessAffinityMask.argtypes=[c.c_void_p,c.c_size_t]
    k.CloseHandle.argtypes=[c.c_void_p]
    return c,k


def cpu_affinity():
    """Actual process CPU set, not a native-library thread-count estimate."""
    if os.name=='nt':
        c,k=_windows();process=c.c_size_t();system=c.c_size_t()
        if not k.GetProcessAffinityMask(k.GetCurrentProcess(),c.byref(process),c.byref(system)):
            raise OSError(c.get_last_error(),'Cannot inspect process CPU affinity')
        return tuple(i for i in range(c.sizeof(c.c_size_t)*8) if process.value&(1<<i))
    if hasattr(os,'sched_getaffinity'):return tuple(sorted(os.sched_getaffinity(0)))
    raise RuntimeError('Verified OS CPU affinity is required for geometry execution')


def _processor_core_sets():
    """Verified single-group Windows RelationProcessorCore masks, not CPU-ID guesses."""
    c,k=_windows();from ctypes import wintypes as w
    class Record(c.Structure):
        _fields_=[('mask',c.c_size_t),('relationship',w.DWORD),('reserved',c.c_uint64*2)]
    k.GetActiveProcessorGroupCount.restype=w.WORD
    if k.GetActiveProcessorGroupCount()!=1:
        raise RuntimeError('Automatic physical-core selection requires one verified Windows processor group')
    k.GetLogicalProcessorInformation.argtypes=[c.c_void_p,c.POINTER(w.DWORD)]
    k.GetLogicalProcessorInformation.restype=w.BOOL
    for _ in range(3):
        size=w.DWORD()
        if k.GetLogicalProcessorInformation(None,c.byref(size)) or c.get_last_error()!=122:
            raise OSError(c.get_last_error(),'Cannot size Windows processor topology')
        if not 0<size.value<=1048576:raise RuntimeError('Invalid Windows topology buffer size')
        capacity=size.value;buffer=c.create_string_buffer(capacity)
        if not k.GetLogicalProcessorInformation(buffer,c.byref(size)):
            if c.get_last_error()==122:continue
            raise OSError(c.get_last_error(),'Cannot read Windows processor topology')
        stride=c.sizeof(Record)
        if not 0<size.value<=capacity or size.value%stride:
            raise RuntimeError('Malformed Windows processor topology records')
        cores=[]
        for offset in range(0,size.value,stride):
            row=Record.from_buffer(buffer,offset)
            if row.relationship==0:
                core=tuple(i for i in range(c.sizeof(c.c_size_t)*8) if row.mask&(1<<i))
                if not core:raise RuntimeError('Empty Windows physical-core mask')
                cores.append(core)
        if not cores:raise RuntimeError('Windows reported no physical-core records')
        return tuple(sorted(cores,key=lambda core:min(core)))
    raise RuntimeError('Windows processor topology changed during bounded query')


def _core_first(allowed,cores):
    """One available logical CPU per verified physical core, then SMT siblings."""
    available=set(allowed);seen=set();selected=[]
    if not available or len(available)!=len(allowed):raise RuntimeError('Invalid allowed CPU inventory')
    for core in sorted(cores,key=lambda core:min(core) if core else -1):
        if not core or len(set(core))!=len(core) or seen&set(core):
            raise RuntimeError('Ambiguous physical-core topology')
        seen.update(core)
        subset=tuple(sorted(available&set(core)))
        if subset:selected.append(subset)
    if not available<=seen:raise RuntimeError('Allowed CPUs are absent from verified topology')
    return tuple(core[0] for core in selected)+tuple(sorted(cpu for core in selected for cpu in core[1:]))


def _inventory(*,_physical=True):
    value=os.environ.get('FSIM_HANGAR_CPU_LEASE')
    if value is None:
        allowed=cpu_affinity()
        return _core_first(allowed,_processor_core_sets()) if _physical and os.name=='nt' else allowed
    if not value or any(not v.isdecimal() for v in value.split(',')):
        raise ValueError('Malformed explicit CPU lease')
    cpus=tuple(int(v) for v in value.split(','))
    if len(set(cpus))!=len(cpus) or any(v>=128 for v in cpus):raise ValueError('Malformed explicit CPU lease')
    return cpus


def _set_affinity(cpus):
    cpus=tuple(cpus)
    if not cpus or any(type(i) is not int or i<0 for i in cpus) or len(set(cpus))!=len(cpus):
        raise ValueError('Nonempty distinct CPU identities required')
    if os.name=='nt':
        c,k=_windows()
        if any(i>=c.sizeof(c.c_size_t)*8 for i in cpus):raise ValueError('Unsupported Windows processor group lease')
        if not k.SetProcessAffinityMask(k.GetCurrentProcess(),sum(1<<i for i in cpus)):
            raise OSError(c.get_last_error(),'Cannot apply required process CPU lease')
    elif hasattr(os,'sched_setaffinity'):os.sched_setaffinity(0,cpus)
    else:raise RuntimeError('Verified OS CPU affinity is required')
    if set(cpu_affinity())!=set(cpus):raise RuntimeError('OS refused the exact process CPU lease')


def current_cpu_lease():return _CPU_LEASE.get()


@contextmanager
def cpu_lease(cap=None,*,cpus=None):
    """Main-thread process scope; children join before this scope can restore it."""
    if threading.current_thread() is not threading.main_thread():
        raise RuntimeError('Process affinity scopes must precede concurrent launcher threads')
    previous=_CPU_LEASE.get();inventory=previous or _inventory(_physical=cpus is None)
    cap=threads(2) if cap is None else cap
    if type(cap) is not int or not 1<=cap<=128:raise ValueError('Invalid CPU lease width')
    if previous is not None:cap=min(cap,len(previous))
    original=cpu_affinity()
    if cpus is None:
        candidates=tuple(cpu for cpu in inventory if cpu in set(original))
        if len(candidates)<cap:candidates=inventory
        if len(candidates)<cap:raise RuntimeError('Insufficient CPUs for the explicit geometry lease')
        cpus=tuple(candidates[:cap])
    else:
        cpus=tuple(cpus)
        if len(cpus)!=cap or not set(cpus)<=set(inventory):raise ValueError('CPU scope escapes its explicit lease')
    _set_affinity(cpus);token=_CPU_LEASE.set(tuple(cpus))
    try:yield tuple(cpus)
    finally:
        try:_set_affinity(original)
        finally:_CPU_LEASE.reset(token)


class CpuLeasePool:
    """FIFO leases over fixed disjoint slots; acquiring never mutates affinity."""
    def __init__(self,workers,width,cpus=None):
        if type(workers) is not int or type(width) is not int or min(workers,width)<1 or workers*width>16:
            raise ValueError('Generation workers times CPU width must not exceed16')
        inventory=tuple(_inventory() if cpus is None else cpus)
        if len(set(inventory))!=len(inventory) or len(inventory)<workers*width:
            raise ValueError('Insufficient disjoint CPUs for generation queue')
        self.cpus=inventory[:workers*width]
        self._slots=[self.cpus[i*width:(i+1)*width] for i in range(workers)]
        self._free=list(range(workers));self._condition=threading.Condition();self._next=0;self._serving=0

    @contextmanager
    def lease(self):
        with self._condition:
            ticket=self._next;self._next+=1
            self._condition.wait_for(lambda:ticket==self._serving and bool(self._free))
            slot=self._free.pop(0);self._serving+=1;self._condition.notify_all()
        try:yield self._slots[slot]
        finally:
            with self._condition:
                self._free.append(slot);self._free.sort();self._condition.notify_all()


def _run_windows_leased(command,log,timeout,cpus,cwd,env):
    """Child-only suspended startup/job. Never assign the coordinator to this job."""
    c,k=_windows();from ctypes import wintypes as w
    class Security(c.Structure):_fields_=[('length',w.DWORD),('descriptor',c.c_void_p),('inherit',w.BOOL)]
    class Startup(c.Structure):
        _fields_=[('cb',w.DWORD),('reserved',w.LPWSTR),('desktop',w.LPWSTR),('title',w.LPWSTR),
            ('x',w.DWORD),('y',w.DWORD),('xs',w.DWORD),('ys',w.DWORD),('xc',w.DWORD),('yc',w.DWORD),
            ('fill',w.DWORD),('flags',w.DWORD),('show',w.WORD),('reserved2size',w.WORD),
            ('reserved2',c.c_void_p),('stdin',w.HANDLE),('stdout',w.HANDLE),('stderr',w.HANDLE)]
    class StartupEx(c.Structure):_fields_=[('basic',Startup),('attributes',c.c_void_p)]
    class Process(c.Structure):_fields_=[('process',w.HANDLE),('thread',w.HANDLE),('pid',w.DWORD),('tid',w.DWORD)]
    class BasicLimit(c.Structure):
        _fields_=[('processTime',c.c_int64),('jobTime',c.c_int64),('flags',w.DWORD),
            ('minWorking',c.c_size_t),('maxWorking',c.c_size_t),('active',w.DWORD),
            ('affinity',c.c_size_t),('priority',w.DWORD),('scheduling',w.DWORD)]
    class Io(c.Structure):_fields_=[(name,c.c_uint64) for name in ('readOps','writeOps','otherOps','readBytes','writeBytes','otherBytes')]
    class ExtendedLimit(c.Structure):
        _fields_=[('basic',BasicLimit),('io',Io),('processMemory',c.c_size_t),('jobMemory',c.c_size_t),
            ('peakProcessMemory',c.c_size_t),('peakJobMemory',c.c_size_t)]
    k.CreateFileW.argtypes=[w.LPCWSTR,w.DWORD,w.DWORD,c.POINTER(Security),w.DWORD,w.DWORD,w.HANDLE];k.CreateFileW.restype=w.HANDLE
    k.CreateJobObjectW.argtypes=[c.c_void_p,w.LPCWSTR];k.CreateJobObjectW.restype=w.HANDLE
    k.SetInformationJobObject.argtypes=[w.HANDLE,c.c_int,c.c_void_p,w.DWORD]
    k.AssignProcessToJobObject.argtypes=[w.HANDLE,w.HANDLE]
    k.TerminateJobObject.argtypes=[w.HANDLE,w.UINT]
    k.TerminateProcess.argtypes=[w.HANDLE,w.UINT]
    k.InitializeProcThreadAttributeList.argtypes=[c.c_void_p,w.DWORD,w.DWORD,c.POINTER(c.c_size_t)]
    k.UpdateProcThreadAttribute.argtypes=[c.c_void_p,w.DWORD,c.c_size_t,c.c_void_p,c.c_size_t,c.c_void_p,c.c_void_p]
    k.DeleteProcThreadAttributeList.argtypes=[c.c_void_p]
    k.CreateProcessW.argtypes=[w.LPCWSTR,w.LPWSTR,c.c_void_p,c.c_void_p,w.BOOL,w.DWORD,c.c_void_p,w.LPCWSTR,c.POINTER(StartupEx),c.POINTER(Process)]
    k.ResumeThread.argtypes=[w.HANDLE];k.ResumeThread.restype=w.DWORD
    k.WaitForSingleObject.argtypes=[w.HANDLE,w.DWORD];k.WaitForSingleObject.restype=w.DWORD
    k.GetExitCodeProcess.argtypes=[w.HANDLE,c.POINTER(w.DWORD)]
    handles=[];job=None;process=Process();attributes=None;initialized=False;resumed=False
    def checked(value,message):
        if not value:raise OSError(c.get_last_error(),message)
        return value
    started=time.monotonic()
    try:
        security=Security(c.sizeof(Security),None,True)
        output=k.CreateFileW(str(Path(log).resolve()),0x40000000,0x7,c.byref(security),2,0x80,None)
        if output in (None,c.c_void_p(-1).value):raise OSError(c.get_last_error(),'Cannot create owned process log')
        handles.append(output)
        stdin=k.CreateFileW('NUL',0x80000000,0x7,c.byref(security),3,0x80,None)
        if stdin in (None,c.c_void_p(-1).value):raise OSError(c.get_last_error(),'Cannot open child NUL input')
        handles.append(stdin)
        size=c.c_size_t();k.InitializeProcThreadAttributeList(None,1,0,c.byref(size))
        attributes=c.create_string_buffer(size.value)
        checked(k.InitializeProcThreadAttributeList(attributes,1,0,c.byref(size)),'Cannot initialize child handle whitelist');initialized=True
        inherited=(w.HANDLE*2)(output,stdin)
        checked(k.UpdateProcThreadAttribute(attributes,0,0x20002,inherited,c.sizeof(inherited),None,None),'Cannot restrict child handles')
        startup=StartupEx();startup.basic.cb=c.sizeof(StartupEx);startup.attributes=c.cast(attributes,c.c_void_p)
        startup.basic.flags=0x100;startup.basic.stdin=stdin;startup.basic.stdout=output;startup.basic.stderr=output
        environment=dict(os.environ if env is None else env)
        environment['FSIM_HANGAR_CPU_LEASE']=','.join(map(str,cpus))
        block=c.create_unicode_buffer('\0'.join(k+'='+str(v) for k,v in sorted(environment.items(),key=lambda p:p[0].upper()))+'\0\0')
        commandline=c.create_unicode_buffer(subprocess.list2cmdline(list(map(str,command))))
        checked(k.CreateProcessW(str(command[0]),commandline,None,None,True,0x4|0x8000000|0x400|0x80000,
            block,None if cwd is None else str(cwd),c.byref(startup),c.byref(process)),'Cannot create suspended leased child')
        mask=sum(1<<i for i in cpus)
        checked(k.SetProcessAffinityMask(process.process,mask),'Cannot pin suspended generation child')
        actual=c.c_size_t();system=c.c_size_t()
        checked(k.GetProcessAffinityMask(process.process,c.byref(actual),c.byref(system)),'Cannot verify child CPU lease')
        if actual.value!=mask:raise RuntimeError('Child CPU lease verification failed before resume')
        job=checked(k.CreateJobObjectW(None,None),'Cannot create owned generation job')
        limit=ExtendedLimit();limit.basic.flags=0x2000|0x10|0x4000;limit.basic.affinity=mask
        checked(k.SetInformationJobObject(job,9,c.byref(limit),c.sizeof(limit)),'Cannot set child-only CPU/job lifetime limits')
        checked(k.AssignProcessToJobObject(job,process.process),'Cannot bind suspended child to owned job')
        if k.ResumeThread(process.thread)==0xffffffff:raise OSError(c.get_last_error(),'Cannot resume leased generation child')
        resumed=True;k.CloseHandle(process.thread);process.thread=None
        remaining=timeout-(time.monotonic()-started)
        status=k.WaitForSingleObject(process.process,max(0,min(0xfffffffe,math.ceil(remaining*1000))))
        if status==0x102:
            checked(k.TerminateJobObject(job,124),'Cannot terminate timed-out generation tree')
            if k.WaitForSingleObject(process.process,5000)!=0:raise RuntimeError('Owned generation child did not join after timeout')
            raise subprocess.TimeoutExpired(command,timeout)
        if status!=0:raise OSError(c.get_last_error(),'Cannot wait for leased generation child')
        code=w.DWORD();checked(k.GetExitCodeProcess(process.process,c.byref(code)),'Cannot read leased child exit status')
        return code.value
    finally:
        if process.process:
            if job:k.TerminateJobObject(job,124)
            elif not resumed:k.TerminateProcess(process.process,124)
            k.WaitForSingleObject(process.process,5000)
        if process.thread:k.CloseHandle(process.thread)
        if process.process:k.CloseHandle(process.process)
        try:
            if job:_finish_windows_job(job)
        finally:
            if initialized:k.DeleteProcThreadAttributeList(attributes)
            for handle in handles:k.CloseHandle(handle)


def run_leased(command,log,timeout,cpus,*,cwd=None,env=None):
    """Run an owned child tree inside a previously allocated disjoint CPU set."""
    cpus=tuple(cpus)
    if not cpus or len(set(cpus))!=len(cpus) or not set(cpus)<=set(_inventory(_physical=False)):raise ValueError('Child escapes CPU lease inventory')
    if not math.isfinite(timeout) or timeout<=0:raise subprocess.TimeoutExpired(command,timeout)
    if os.name=='nt':return _run_windows_leased(command,log,timeout,cpus,cwd,env)
    raise RuntimeError('Suspended leased generation startup requires the Windows implementation')


def _attach_windows_job(process,cpus):
    """Pin and own ONLY a suspended child, before its interpreter can execute."""
    c,k=_windows();from ctypes import wintypes as w
    class Basic(c.Structure):
        _fields_=[('pt',c.c_int64),('jt',c.c_int64),('flags',w.DWORD),('min',c.c_size_t),
            ('max',c.c_size_t),('active',w.DWORD),('affinity',c.c_size_t),('priority',w.DWORD),('scheduling',w.DWORD)]
    class Io(c.Structure):_fields_=[('v',c.c_uint64*6)]
    class Limit(c.Structure):_fields_=[('basic',Basic),('io',Io),('memory',c.c_size_t*4)]
    k.CreateJobObjectW.argtypes=[c.c_void_p,w.LPCWSTR];k.CreateJobObjectW.restype=w.HANDLE
    k.SetInformationJobObject.argtypes=[w.HANDLE,c.c_int,c.c_void_p,w.DWORD]
    k.AssignProcessToJobObject.argtypes=[w.HANDLE,w.HANDLE]
    mask=sum(1<<i for i in cpus);actual=c.c_size_t();system=c.c_size_t();job=None
    try:
        if not k.SetProcessAffinityMask(process,mask):raise OSError(c.get_last_error(),'Cannot pin suspended owner worker')
        if not k.GetProcessAffinityMask(process,c.byref(actual),c.byref(system)) or actual.value!=mask:
            raise RuntimeError('Owner worker CPU lease verification failed before resume')
        job=k.CreateJobObjectW(None,None)
        if not job:raise OSError(c.get_last_error(),'Cannot create owner worker lifetime job')
        limit=Limit();limit.basic.flags=0x2000|0x10|0x4000;limit.basic.affinity=mask
        if not k.SetInformationJobObject(job,9,c.byref(limit),c.sizeof(limit)) or not k.AssignProcessToJobObject(job,process):
            raise OSError(c.get_last_error(),'Cannot apply owner worker CPU/job limits')
        if not k.GetProcessAffinityMask(process,c.byref(actual),c.byref(system)) or actual.value!=mask:
            raise RuntimeError('Nested owner job changed its verified single-CPU lease')
        return job
    except BaseException:
        if job:k.CloseHandle(job)
        raise


def _finish_windows_job(job):
    c,k=_windows();from ctypes import wintypes as w
    class Accounting(c.Structure):_fields_=[('times',c.c_int64*4),('faults',w.DWORD),('total',w.DWORD),('active',w.DWORD),('terminated',w.DWORD)]
    k.TerminateJobObject.argtypes=[w.HANDLE,w.UINT]
    k.QueryInformationJobObject.argtypes=[w.HANDLE,c.c_int,c.c_void_p,w.DWORD,c.c_void_p]
    try:
        if not k.TerminateJobObject(job,124):raise OSError(c.get_last_error(),'Cannot terminate owned worker tree')
        deadline=time.monotonic()+5.
        while True:
            info=Accounting()
            if not k.QueryInformationJobObject(job,1,c.byref(info),c.sizeof(info),None):
                raise OSError(c.get_last_error(),'Cannot verify owned job cleanup')
            if info.active==0:return
            if time.monotonic()>=deadline:raise RuntimeError('Owned worker descendants have not joined')
            time.sleep(.01)
    finally:k.CloseHandle(job)


def leased_spawn_popen(process_obj,cpus):
    """Isolated CPython spawn delegate; no global stdlib/affinity monkeypatch."""
    if os.name!='nt':raise RuntimeError('The verified owner spawn adapter requires Windows')
    from multiprocessing import popen_spawn_win32 as base
    from types import FunctionType
    import inspect
    import sys
    if (sys.implementation.name!='cpython' or sys.version_info[:2]!=(3,13) or
        not isinstance(base.Popen.__init__,FunctionType) or
        base.Popen.__init__.__closure__ is not None or base.Popen.__init__.__defaults__ is not None or
        base.Popen.__init__.__kwdefaults__ is not None or
        tuple(inspect.signature(base.Popen.__init__).parameters)!=('self','process_obj') or
        not {'spawn','reduction','_winapi','set_spawning_popen','util'}<=set(base.Popen.__init__.__globals__)):
        raise RuntimeError('Unsupported CPython Windows spawn initializer contract')
    # Pure stdlib source can change independently of python.exe/python313.dll.
    # Its exact bytes also appear in the positive-liquid source receipt.
    source=Path(base.__file__)
    if not source.is_file() or Path(base.Popen.__init__.__code__.co_filename).resolve()!=source.resolve():
        raise RuntimeError('Unbound CPython Windows spawn initializer source')
    from types import CodeType
    compiled=compile(source.read_bytes(),str(source),'exec',optimize=sys.flags.optimize)
    classes=[value for value in compiled.co_consts if isinstance(value,CodeType) and value.co_name=='Popen']
    initializers=[value for cls in classes for value in cls.co_consts if isinstance(value,CodeType) and value.co_name=='__init__']
    active=base.Popen.__init__.__code__
    if len(initializers)!=1 or any(getattr(initializers[0],name)!=getattr(active,name)
            for name in ('co_code','co_consts','co_names','co_varnames','co_argcount','co_posonlyargcount',
                         'co_kwonlyargcount','co_flags','co_exceptiontable','co_freevars','co_cellvars','co_stacksize')):
        raise RuntimeError('Loaded CPython spawn initializer differs from its bound source bytes')
    c,k=_windows();from ctypes import wintypes as w
    k.ResumeThread.argtypes=[w.HANDLE];k.ResumeThread.restype=w.DWORD
    k.TerminateProcess.argtypes=[w.HANDLE,w.UINT]
    original=base.Popen.__init__.__globals__['_winapi'];jobs=[];ticket=getattr(_STARTUP,'ticket',None)
    class Delegate:
        def __getattr__(self,name):return getattr(original,name)
        def CreateProcess(self,*args):
            if ticket is not None:ticket.check()
            modified=list(args);modified[5]|=0x4|0x8000000
            handles=original.CreateProcess(*modified);hp,ht,_,_=handles
            try:
                job=_attach_windows_job(hp,cpus);jobs.append(job)
                if ticket is not None:ticket.register(job);ticket.check()
                if k.ResumeThread(ht)==0xffffffff:raise OSError(c.get_last_error(),'Cannot resume leased owner worker')
                return handles
            except BaseException:
                k.TerminateProcess(hp,124);original.CloseHandle(ht);original.CloseHandle(hp)
                raise
    namespace=dict(base.Popen.__init__.__globals__);namespace['_winapi']=Delegate()
    if ticket is not None:
        original_reduction=namespace['reduction']
        class Reduction:
            def __getattr__(self,name):return getattr(original_reduction,name)
            def dump(self,*args,**kwargs):
                ticket.check();result=original_reduction.dump(*args,**kwargs);ticket.check();return result
        namespace['reduction']=Reduction()
    initializer=FunctionType(base.Popen.__init__.__code__,namespace)
    popen=base.Popen.__new__(base.Popen)
    try:initializer(popen,process_obj)
    except BaseException:
        for job in jobs:_finish_windows_job(job)
        if hasattr(popen,'finalizer'):popen.finalizer()
        raise
    popen._cpu_jobs=jobs
    return popen


def close_spawn_jobs(popen):
    for job in getattr(popen,'_cpu_jobs',()):_finish_windows_job(job)
    popen._cpu_jobs=[]


class _StartupTicket:
    """Supervisor owns duplicated job handles; no initializer handle reuse race."""
    def __init__(self,deadline):
        self.deadline=deadline;self.stop=threading.Event();self.done=threading.Event();self.go=threading.Event()
        self.jobs=[];self.lock=threading.Lock();self.error=None
    def check(self):
        if self.stop.is_set() or time.monotonic()>self.deadline:
            raise TimeoutError('Liquid global startup time deadline exceeded')
    def register(self,job):
        c,k=_windows();from ctypes import wintypes as w
        k.DuplicateHandle.argtypes=[w.HANDLE,w.HANDLE,w.HANDLE,c.POINTER(w.HANDLE),w.DWORD,w.BOOL,w.DWORD]
        duplicate=w.HANDLE();current=k.GetCurrentProcess()
        if not k.DuplicateHandle(current,job,current,c.byref(duplicate),0,False,2):
            raise OSError(c.get_last_error(),'Cannot own startup supervision job handle')
        with self.lock:self.jobs.append(duplicate.value)
    def terminate(self):
        c,k=_windows();from ctypes import wintypes as w
        k.TerminateJobObject.argtypes=[w.HANDLE,w.UINT]
        with self.lock:
            for job in self.jobs:
                if not k.TerminateJobObject(job,124):raise OSError(c.get_last_error(),'Cannot terminate owned startup child job')
    def close(self):
        _,k=_windows()
        with self.lock:
            for job in self.jobs:k.CloseHandle(job)
            self.jobs.clear()


def start_owned_process(process,deadline):
    """Bound Process.start AND its synchronous serialization, not just its target.

    The real dedicated writer thread is never reused. CancelSynchronousIo is
    repeated until it joins: cancellation is asynchronous and may race a write.
    The parent's retained read endpoint is not treated as EOF or closed twice.
    """
    ticket=_StartupTicket(deadline)
    def start():
        _STARTUP.ticket=ticket
        try:
            ticket.go.wait(max(0.,deadline-time.monotonic()));ticket.check();process.start()
        except BaseException as error:ticket.error=error
        finally:
            del _STARTUP.ticket;ticket.done.set()
    thread=threading.Thread(target=start,name='hangar-owner-start',daemon=False)
    thread.start();c,k=_windows();from ctypes import wintypes as w
    k.OpenThread.argtypes=[w.DWORD,w.BOOL,w.DWORD];k.OpenThread.restype=w.HANDLE
    k.CancelSynchronousIo.argtypes=[w.HANDLE]
    handle=None;expired=False
    try:
        if not ticket.done.is_set():
            handle=k.OpenThread(0x1,False,thread.native_id)
            if not handle and not ticket.done.is_set():
                ticket.stop.set();ticket.go.set();thread.join(timeout=5)
                raise OSError(c.get_last_error(),'Cannot own the startup writer thread before process creation')
        ticket.go.set()
        while not ticket.done.wait(max(0.,min(.01,deadline-time.monotonic()))):
            if time.monotonic()>=deadline:expired=True;break
        if time.monotonic()>deadline:expired=True
        if expired:
            ticket.stop.set();cleanup_deadline=time.monotonic()+5.
            while not ticket.done.is_set():
                ticket.terminate()
                if handle and not k.CancelSynchronousIo(handle) and c.get_last_error()!=1168:
                    raise OSError(c.get_last_error(),'Cannot cancel owned startup serialization I/O')
                if time.monotonic()>=cleanup_deadline:
                    raise RuntimeError('Owned startup writer did not join; resource lease cannot be released')
                thread.join(timeout=.01)
        thread.join(timeout=5)
        if thread.is_alive():raise RuntimeError('Owned startup writer thread has not joined')
        if time.monotonic()>deadline:expired=True
        if expired:
            if process._popen is not None:
                if process.is_alive():process.terminate()
                process.join(timeout=5)
                if process.is_alive():raise RuntimeError('Started owner did not join after startup deadline')
                close_spawn_jobs(process._popen);process.close()
            raise TimeoutError('Liquid global startup time deadline exceeded')
        if ticket.error is not None:raise ticket.error
    finally:
        if handle:k.CloseHandle(handle)
        ticket.close()

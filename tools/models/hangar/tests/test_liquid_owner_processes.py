"""Actual spawn/OS/resource controls for ordinary liquid-owner execution.

Synthetic coordinator jobs exercise lifetime and resource boundaries; they do
not stand in for actual aircraft geometry or normal-model acceptance.
"""
import json
import multiprocessing as mp
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from hangar import liquid, resources
BOOTSTRAP_CPUS=resources.cpu_affinity()


def pid_active(pid):
    import ctypes as c
    k=c.WinDLL('kernel32',use_last_error=True)
    k.OpenProcess.argtypes=[c.c_uint32,c.c_int,c.c_uint32];k.OpenProcess.restype=c.c_void_p
    k.GetExitCodeProcess.argtypes=[c.c_void_p,c.POINTER(c.c_uint32)];k.CloseHandle.argtypes=[c.c_void_p]
    handle=k.OpenProcess(0x1000,False,pid)
    if not handle:return False
    try:
        status=c.c_uint32()
        if not k.GetExitCodeProcess(handle,c.byref(status)):raise OSError(c.get_last_error())
        return status.value==259
    finally:k.CloseHandle(handle)


def coordinator_fixture(recipe, execution):
    """Importable spawn target; no receipt-selected module or closure."""
    if recipe.get('crash'): os._exit(23)
    for _ in range(recipe.get('screens',0)): execution.screen()
    execution.reserve_edges(recipe.get('edges',0))
    for _ in range(recipe.get('cells',0)): execution.emit_cell()
    if recipe.get('refuse'): raise liquid.LiquidError('Independent fixture refusal')
    time.sleep(recipe.get('delay',0.))
    execution.check()
    return dict(ordinal=recipe['ordinal'],value=recipe['ordinal']**2,
                cpus=list(resources.cpu_affinity()),bootstrap_cpus=list(BOOTSTRAP_CPUS),cache_limits=execution.cache_limits)


class LiquidOwnerProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir='D:/ENV/tmp' if os.name=='nt' else None)
        self.directory=Path(self.tmp.name)
        self.original_affinity=resources.cpu_affinity();self.original_lease=os.environ.get('FSIM_HANGAR_CPU_LEASE')
        self.reserved=tuple(int(v) for v in self.original_lease.split(',')) if self.original_lease else self.original_affinity[:2]
        if len(self.reserved)<2:self.fail('The explicit process-method gate requires two leased CPUs')
        os.environ['FSIM_HANGAR_CPU_LEASE']=','.join(map(str,self.reserved))
        resources._set_affinity(self.reserved[:1])

    def tearDown(self):
        try:self.tmp.cleanup()
        finally:
            resources._set_affinity(self.original_affinity)
            if self.original_lease is None:os.environ.pop('FSIM_HANGAR_CPU_LEASE',None)
            else:os.environ['FSIM_HANGAR_CPU_LEASE']=self.original_lease

    def budget(self,**overrides):
        limits=liquid._limits(dict(max_screened_cells=40,max_cells=30,max_assignment_edges=50,**overrides))
        return liquid._OwnerBudget(mp.get_context('spawn'),limits,time.monotonic()+20.,2)

    def test_cpu_lease_is_verified_nested_and_restored(self):
        before=resources.cpu_affinity()
        self.assertGreaterEqual(len(self.reserved),2,'The explicit process-method gate needs two reserved CPUs')
        with resources.cpu_lease(2) as outer:
            self.assertEqual(len(outer),2)
            self.assertEqual(tuple(outer),resources.cpu_affinity())
            with resources.cpu_lease(1) as inner:
                self.assertEqual(len(inner),1)
                self.assertTrue(set(inner)<=set(outer))
                self.assertEqual(tuple(inner),resources.cpu_affinity())
            self.assertEqual(resources.cpu_affinity(),tuple(outer))
        self.assertEqual(resources.cpu_affinity(),before)

    def test_explicit_one_cpu_reservation_stays_serial(self):
        with resources.thread_reservation(1), resources.cpu_lease() as cpus:
            self.assertEqual(len(cpus),1)
            self.assertEqual(resources.current_cpu_lease(),tuple(cpus))

    def test_queue_leases_are_disjoint_reused_and_exception_safe(self):
        available=self.reserved
        pool=resources.CpuLeasePool(2,1,available)
        barrier=threading.Barrier(2);held=[]
        def work():
            with pool.lease() as cpus:
                held.append(tuple(cpus));barrier.wait(timeout=5)
        threads=[threading.Thread(target=work) for _ in range(2)]
        for thread in threads:thread.start()
        for thread in threads:thread.join(timeout=6);self.assertFalse(thread.is_alive())
        self.assertEqual(len(held),2);self.assertFalse(set(held[0])&set(held[1]))
        with self.assertRaisesRegex(RuntimeError,'fixture'):
            with pool.lease():raise RuntimeError('fixture')
        with pool.lease() as cpus:self.assertTrue(set(cpus)<=set(available))

    def test_owned_generation_child_starts_inside_lease_and_parent_lives(self):
        before=resources.cpu_affinity();cpus=self.reserved[:2];log=self.directory/'child.log'
        command=[sys.executable,'-B','-c',
            'import json;from hangar.resources import cpu_affinity;print(json.dumps(list(cpu_affinity())))']
        code=resources.run_leased(command,log,15.,cpus,env=os.environ.copy())
        self.assertEqual(code,0)
        self.assertEqual(tuple(json.loads(log.read_text())),cpus)
        self.assertEqual(resources.cpu_affinity(),before)

    def test_owned_generation_timeout_reaps_descendant_tree(self):
        before=resources.cpu_affinity();log=self.directory/'timeout.log';pidfile=self.directory/'descendant.pid'
        command=[sys.executable,'-B','-c',
            'import subprocess,sys,time;from pathlib import Path;p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(20)"]);Path('+repr(str(pidfile))+').write_text(str(p.pid));time.sleep(20)']
        started=time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired):
            resources.run_leased(command,log,.25,self.reserved[:2],env=os.environ.copy())
        self.assertLess(time.monotonic()-started,8.)
        self.assertEqual(resources.cpu_affinity(),before)
        self.assertTrue(pidfile.is_file(),'The fixture must actually launch its descendant before timing out')
        self.assertFalse(pid_active(int(pidfile.read_text())))

    def test_global_budget_checks_actual_screen_edges_and_cells(self):
        budget=self.budget()
        for _ in range(40):budget.screen()
        with self.assertRaisesRegex(liquid.LiquidError,'screen'):budget.screen()
        budget=self.budget();budget.reserve_edges(50)
        with self.assertRaisesRegex(liquid.LiquidError,'edge'):budget.reserve_edges(1)
        budget=self.budget()
        for _ in range(30):budget.emit_cell()
        with self.assertRaisesRegex(liquid.LiquidError,'cell'):budget.emit_cell()

    def test_spawn_completion_order_cannot_reorder_results_or_escape_cpu_cap(self):
        original=set(p.pid for p in mp.active_children());budget=self.budget()
        with resources.cpu_lease(2) as cpus:
            jobs=[dict(ordinal=0,delay=.2,screens=3,edges=5,cells=2),
                  dict(ordinal=1,delay=0.,screens=4,edges=6,cells=3),dict(ordinal=2)]
            rows=liquid._process_owner_jobs(jobs,coordinator_fixture,budget,cpus)
            self.assertEqual([r['ordinal'] for r in rows],[0,1,2])
            self.assertTrue(all(len(r['cpus'])==1 and set(r['cpus'])<=set(cpus) for r in rows))
            self.assertTrue(all(r['bootstrap_cpus']==r['cpus'] for r in rows))
            self.assertEqual(sum(r['cache_limits']['max_triangles'] for r in rows[:2]),budget.limits['max_triangles'])
        self.assertEqual(set(p.pid for p in mp.active_children()),original)
        self.assertEqual(budget.counts(),dict(screened=7,edges=11,cells=5))

    def test_worker_native_refusal_stops_siblings_and_next_call_is_fresh(self):
        original=set(p.pid for p in mp.active_children())
        with resources.cpu_lease(2) as cpus:
            with self.assertRaisesRegex(liquid.LiquidError,'Independent fixture refusal'):
                liquid._process_owner_jobs([dict(ordinal=0,refuse=True),dict(ordinal=1,delay=10.)],
                    coordinator_fixture,self.budget(),cpus)
            rows=liquid._process_owner_jobs([dict(ordinal=0)],coordinator_fixture,self.budget(),cpus)
            self.assertEqual(rows[0]['value'],0)
        self.assertEqual(set(p.pid for p in mp.active_children()),original)

    def test_worker_crash_and_global_deadline_cannot_publish_partial_success(self):
        original=set(p.pid for p in mp.active_children())
        with resources.cpu_lease(2) as cpus:
            with self.assertRaises(liquid.LiquidError):
                liquid._process_owner_jobs([dict(ordinal=0,crash=True),dict(ordinal=1,delay=10.)],
                    coordinator_fixture,self.budget(),cpus)
            budget=self.budget();budget.deadline=time.monotonic()+.15
            started=time.monotonic()
            with self.assertRaisesRegex(liquid.LiquidError,'deadline|time'):
                liquid._process_owner_jobs([dict(ordinal=0,delay=10.),dict(ordinal=1,delay=10.)],
                    coordinator_fixture,budget,cpus)
            self.assertLess(time.monotonic()-started,8.)
        self.assertEqual(set(p.pid for p in mp.active_children()),original)

    def test_plain_job_recipe_refuses_nonfinite_and_live_objects_before_spawn(self):
        with resources.cpu_lease(2) as cpus:
            for value in (float('nan'),object()):
                with self.subTest(value=type(value).__name__), self.assertRaises(liquid.LiquidError):
                    liquid._process_owner_jobs([dict(ordinal=0,value=value)],coordinator_fixture,self.budget(),cpus)

    def test_two_workers_share_actual_aggregate_ceilings(self):
        with resources.cpu_lease(2) as cpus:
            for charges,label in ((dict(screens=30),'screen'),(dict(edges=30),'edge'),(dict(cells=20),'cell')):
                with self.subTest(label=label),self.assertRaisesRegex(liquid.LiquidError,label):
                    liquid._process_owner_jobs([dict(ordinal=i,**charges) for i in range(2)],
                        coordinator_fixture,self.budget(),cpus)
            self.assertEqual(mp.active_children(),[])

    def test_half_cache_keeps_exact_geometry_trace_and_literal_final_audits(self):
        from test_liquid_pack_equivalence import observed,cube,tank
        hull=cube((2.,2.,2.))-cube((.4,.4,2.),(.8,.8,0.))
        limits=liquid._limits({'max_vertices':64,'max_triangles':96})
        budget=liquid._OwnerBudget(mp.get_context('spawn'),limits,time.monotonic()+20.,2)
        execution=liquid._OwnerExecution(budget,0)
        expected=observed(liquid.pack,'half-cache',hull,[tank(0,2.,(.4,.4,.4))],limits=limits)
        actual=observed(liquid.pack,'half-cache',hull,[tank(0,2.,(.4,.4,.4))],limits=limits,_execution=execution)
        self.assertIsNone(expected['refusal']);self.assertIsNone(actual['refusal'])
        self.assertEqual(expected['result'],actual['result']);self.assertEqual(expected['owner_trace'],actual['owner_trace'])
        self.assertEqual({k:v for k,v in expected['counts'].items() if k.startswith('Emitted ')},
                         {k:v for k,v in actual['counts'].items() if k.startswith('Emitted ')})
        self.assertGreater(actual['cache_metrics']['fallback'],0)
        self.assertLessEqual(actual['cache_metrics']['peak_vertices'],32)
        self.assertLessEqual(actual['cache_metrics']['peak_triangles'],48)

    def source_fixture(self):
        import numpy as np
        from hangar.geometry import Aircraft
        from test_liquid import glb,cube,exported_design_vertices
        design=self.directory/'owned.toml'
        design.write_text('''[aircraft]
name="owned"
[mass]
tank=[{capacity=200.0,position=[0.5,0.5,0.5]},{capacity=200.0,position=[1.5,0.5,0.5]}]
[[surface]]
name="wing"
kind="wing"
sections=[{le=[1.6,0.0,10.0],chord=1.0,airfoil="naca0012"},{le=[1.6,5.0,10.0],chord=1.0,airfoil="naca0012"}]
''',encoding='utf-8')
        model=self.directory/'owned.glb'
        glb(model,[cube((1,1,1)),cube((1,1,1))],
            [dict(name='root',children=[1,2]),dict(name='fsim:section:nose',mesh=0),
             dict(name='fsim:section:centre',mesh=1,translation=[0,0,-1])],
            vertex_transform=exported_design_vertices,authoritative=cube((2,1,1)))
        native=dict(aircraft='owned',origin_design_m=[0.,0.,0.],components=[],
            sections=[dict(name=name,tanks=[dict(index=i,capacity_kg=200.)]) for i,name in enumerate(('nose','centre'))])
        model.with_suffix('.damage.json').write_text(json.dumps(native),encoding='utf-8')
        return Aircraft.load(str(design)),model,np.zeros(3)

    def test_actual_source_decoder_serial_parallel_cells_traces_and_audits_match(self):
        aircraft,model,origin=self.source_fixture()
        traces=[];audits={};parallel_rows=[];original_pack=liquid.pack;original_valid=liquid._valid
        def traced_pack(*args,**options):
            trace=[];certificate=options['owner_certificate']
            def owner(lo,hi):
                result=certificate(lo,hi)
                trace.append([[float(x).hex() for x in lo],[float(x).hex() for x in hi],result])
                return result
            options['owner_certificate']=owner;result=original_pack(*args,**options);traces.append(trace);return result
        def valid(solid,context,**options):
            if context.startswith('Emitted '):audits[context]=audits.get(context,0)+1
            return original_valid(solid,context,**options)
        with resources.thread_reservation(1),patch.object(liquid,'pack',side_effect=traced_pack),patch.object(liquid,'_valid',side_effect=valid):
            serial=liquid.produce(aircraft,origin,glb=model)
        original_jobs=liquid._process_owner_jobs
        def traced_jobs(jobs,*args):
            for job in jobs:job['trace']=True
            rows=original_jobs(jobs,*args);parallel_rows.extend(rows);return rows
        with resources.thread_reservation(2),patch.object(liquid,'_process_owner_jobs',side_effect=traced_jobs):
            parallel=liquid.produce(aircraft,origin,glb=model)
        self.assertEqual(json.dumps(serial,sort_keys=True),json.dumps(parallel,sort_keys=True))
        self.assertEqual(traces,[row['owner_trace'] for row in parallel_rows])
        observed={}
        for row in parallel_rows:
            for key,value in row['audit_counts'].items():observed[key]=observed.get(key,0)+value
        self.assertEqual(audits,observed)
        self.assertEqual(audits['Emitted slab/hull difference'],len(parallel['cells']))
        self.assertEqual(len(parallel['sources']),15)
        self.assertEqual({r['role'] for r in parallel['sources']}&{'execution_resources','spawn_runtime'},
                         {'execution_resources','spawn_runtime'})

    def test_source_change_after_recipe_capture_is_refused_before_geometry(self):
        aircraft,model,origin=self.source_fixture();original_jobs=liquid._process_owner_jobs
        def stale_jobs(jobs,*args):
            for job in jobs:job['bindings'][0]['sha256']='0'*64
            return original_jobs(jobs,*args)
        with resources.thread_reservation(2),patch.object(liquid,'_process_owner_jobs',side_effect=stale_jobs):
            with self.assertRaisesRegex(liquid.LiquidError,'Stale owner execution'):
                liquid.produce(aircraft,origin,glb=model)

    def test_unsupported_spawn_initializer_refuses_before_child_creation(self):
        from multiprocessing import popen_spawn_win32
        with resources.cpu_lease(2) as cpus,patch.object(popen_spawn_win32.Popen,'__init__',lambda self,process_obj,extra=None:None):
            before=set(p.pid for p in mp.active_children())
            with self.assertRaisesRegex(liquid.LiquidError,'Unsupported CPython'):
                liquid._process_owner_jobs([dict(ordinal=0)],coordinator_fixture,self.budget(),cpus)
            self.assertEqual(set(p.pid for p in mp.active_children()),before)

    def test_actual_spawn_pipe_failure_and_cpu_setup_failure_leave_no_children(self):
        from multiprocessing import reduction,popen_spawn_win32
        before=set(p.pid for p in mp.active_children());api=popen_spawn_win32.Popen.__init__.__globals__['_winapi']
        with resources.cpu_lease(2) as cpus:
            for target in ('pipe','cpu'):
                with self.subTest(target=target):
                    fault=patch.object(reduction,'dump',side_effect=OSError('injected pipe failure')) if target=='pipe' else patch.object(resources,'_attach_windows_job',side_effect=OSError('injected CPU setup failure'))
                    with fault,self.assertRaises(liquid.LiquidError):
                        liquid._process_owner_jobs([dict(ordinal=0)],coordinator_fixture,self.budget(),cpus)
                    self.assertEqual(set(p.pid for p in mp.active_children()),before)
            rows=liquid._process_owner_jobs([dict(ordinal=0)],coordinator_fixture,self.budget(),cpus)
            self.assertEqual(rows[0]['value'],0)
        self.assertIs(popen_spawn_win32.Popen.__init__.__globals__['_winapi'],api)

    def test_nested_generation_two_cpu_job_allows_verified_one_cpu_owner_workers(self):
        script=self.directory/'nested.py';log=self.directory/'nested.log'
        script.write_text('''import multiprocessing as mp,time,json
from hangar import liquid,resources
def work(recipe,execution):
    return {'cpus':list(resources.cpu_affinity())}
if __name__=='__main__':
    with resources.cpu_lease(2) as cpus:
        budget=liquid._OwnerBudget(mp.get_context('spawn'),liquid._limits(None),time.monotonic()+10.,2)
        rows=liquid._process_owner_jobs([{'ordinal':0},{'ordinal':1}],work,budget,cpus)
        assert all(len(r['cpus'])==1 for r in rows)
        print(json.dumps(rows))
''',encoding='utf-8')
        code=resources.run_leased([sys.executable,'-B',str(script)],log,15.,self.reserved[:2],env=os.environ.copy())
        self.assertEqual(code,0,log.read_text(encoding='utf-8'))
        print(json.dumps(dict(case='nested-generation-owner-affinity',observation=log.read_text(encoding='utf-8').strip())),flush=True)

    def test_slow_main_import_backpressure_obeys_same_startup_deadline(self):
        script=self.directory/'slow_import.py';log=self.directory/'slow_import.log'
        script.write_text('''import multiprocessing as mp,threading,time,json,ctypes as c,gc,os
from hangar import liquid,resources
if __name__=='__mp_main__' and os.environ.get('HANGAR_STARTUP_DELAY')=='1':time.sleep(10.)
def work(recipe,execution):return {'ordinal':recipe['ordinal']}
if __name__=='__main__':
    kernel=c.WinDLL('kernel32',use_last_error=True)
    kernel.GetCurrentProcess.restype=c.c_void_p
    kernel.GetProcessHandleCount.argtypes=[c.c_void_p,c.POINTER(c.c_uint32)]
    def handles():
        value=c.c_uint32()
        assert kernel.GetProcessHandleCount(kernel.GetCurrentProcess(),c.byref(value))
        return value.value
    started=time.monotonic()
    with resources.cpu_lease(2) as cpus:
        warm=liquid._OwnerBudget(mp.get_context('spawn'),liquid._limits(None),time.monotonic()+20.,2)
        budget=liquid._OwnerBudget(mp.get_context('spawn'),liquid._limits(None),time.monotonic()+.25,2)
        cold=handles();liquid._process_owner_jobs([{'ordinal':0}],work,warm,cpus)
        gc.collect();before=handles();observed=[]
        os.environ['HANGAR_STARTUP_DELAY']='1'
        for iteration in range(2):
            budget.cancel.clear();budget.deadline=time.monotonic()+.25
            try:liquid._process_owner_jobs([{'ordinal':0,'padding':'x'*24000}],work,budget,cpus)
            except liquid.LiquidError as error:
                assert 'time' in str(error).lower() or 'deadline' in str(error).lower(),str(error)
            else:raise AssertionError('Slow startup must refuse')
            assert not mp.active_children()
            assert not [t for t in threading.enumerate() if t.name.startswith('hangar-owner-start')]
            gc.collect();after=handles();assert after==before,(before,after)
            observed.append(after)
    assert time.monotonic()-started<2.5
    print(json.dumps({'bounded':True,'elapsed_s':time.monotonic()-started,'cold_handles':cold,'warmed_handles':before,'canceled_handles':observed}))
''',encoding='utf-8')
        with resources.cpu_lease(2),log.open('w',encoding='utf-8') as stream:
            process=subprocess.Popen([sys.executable,'-B',str(script)],stdout=stream,stderr=subprocess.STDOUT,env=os.environ.copy())
            try:code=process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=5)
                process.wait(timeout=5)
                self.fail('Original startup deadline was bypassed by a blocked serialization pipe')
        self.assertEqual(code,0,log.read_text(encoding='utf-8'))
        print(json.dumps(dict(case='slow-import-pipe-backpressure',observation=log.read_text(encoding='utf-8').strip())),flush=True)

    def test_changed_initializer_exception_table_refuses_before_spawn(self):
        from multiprocessing import popen_spawn_win32
        initializer=popen_spawn_win32.Popen.__init__
        original=initializer.__code__
        try:
            initializer.__code__=original.replace(co_exceptiontable=b'')
            with resources.cpu_lease(2) as cpus:
                with self.assertRaisesRegex(liquid.LiquidError,'source|initializer'):
                    liquid._process_owner_jobs([dict(ordinal=0)],coordinator_fixture,self.budget(),cpus)
        finally:initializer.__code__=original

    def test_physical_core_order_is_independent_of_os_record_order(self):
        cores=((4,5),(0,1),(6,7),(2,3))
        self.assertEqual(resources._core_first(tuple(range(8)),cores),(0,2,4,6,1,3,5,7))
        self.assertEqual(resources._core_first(tuple(range(8)),tuple(reversed(cores))),
                         (0,2,4,6,1,3,5,7))

    def test_restricted_topology_never_adds_unavailable_siblings(self):
        self.assertEqual(resources._core_first((1,2,3,5),((0,1),(2,3),(4,5))),(1,2,5,3))
        self.assertEqual(resources._core_first((0,1),((0,1),(2,3))),(0,1))
        with self.assertRaises(RuntimeError):resources._core_first((0,2),((0,1),))
        with self.assertRaises(RuntimeError):resources._core_first((0,1),((0,1),(1,2)))

    def test_automatic_scope_keeps_verified_inventory_order(self):
        with patch.dict(os.environ),patch.object(resources,'cpu_affinity',return_value=(0,1,2,3)),\
             patch.object(resources,'_processor_core_sets',return_value=((0,1),(2,3)),create=True),\
             patch.object(resources,'_set_affinity') as apply:
            os.environ.pop('FSIM_HANGAR_CPU_LEASE',None)
            self.assertEqual(resources._inventory(),(0,2,1,3))
            with resources.cpu_lease(2) as cpus:self.assertEqual(cpus,(0,2))
            self.assertEqual(apply.call_args_list[0].args[0],(0,2))
            self.assertEqual(apply.call_args_list[-1].args[0],(0,1,2,3))

    def test_explicit_cpu_order_bypasses_automatic_topology(self):
        with patch.dict(os.environ,{'FSIM_HANGAR_CPU_LEASE':'3,1'}),\
             patch.object(resources,'cpu_affinity',return_value=(1,3)),\
             patch.object(resources,'_processor_core_sets',side_effect=AssertionError('Explicit lease must not query topology'),create=True),\
             patch.object(resources,'_set_affinity'):
            self.assertEqual(resources._inventory(),(3,1))
            with resources.cpu_lease(2) as cpus:self.assertEqual(cpus,(3,1))
            with resources.cpu_lease(2,cpus=(1,3)) as cpus:self.assertEqual(cpus,(1,3))
        with patch.dict(os.environ),patch.object(resources,'cpu_affinity',return_value=(0,1,2,3)),\
             patch.object(resources,'_processor_core_sets',side_effect=AssertionError('Explicit CPUs do not need topology')),\
             patch.object(resources,'_set_affinity'):
            os.environ.pop('FSIM_HANGAR_CPU_LEASE',None)
            with resources.cpu_lease(2,cpus=(3,1)) as cpus:self.assertEqual(cpus,(3,1))

    def test_default_pool_uses_disjoint_core_first_slots_and_explicit_pool_is_exact(self):
        with patch.dict(os.environ),patch.object(resources,'cpu_affinity',return_value=tuple(range(8))),\
             patch.object(resources,'_processor_core_sets',return_value=((0,1),(2,3),(4,5),(6,7)),create=True):
            os.environ.pop('FSIM_HANGAR_CPU_LEASE',None)
            pool=resources.CpuLeasePool(2,2)
            with pool.lease() as first,pool.lease() as second:
                self.assertEqual(first,(0,2));self.assertEqual(second,(4,6));self.assertFalse(set(first)&set(second))
            explicit=resources.CpuLeasePool(2,2,(7,5,3,1))
            with explicit.lease() as first,explicit.lease() as second:
                self.assertEqual(first,(7,5));self.assertEqual(second,(3,1))

    def test_actual_windows_topology_covers_current_allowed_cpu_set(self):
        cores=resources._processor_core_sets();available=resources.cpu_affinity()
        ordered=resources._core_first(available,cores)
        self.assertEqual(set(ordered),set(available));self.assertEqual(len(ordered),len(available))
        representatives=[min(set(core)&set(self.reserved)) for core in cores if set(core)&set(self.reserved)]
        print(json.dumps(dict(case='actual-physical-core-topology',cores=cores,available=available,
                              ordered=ordered,reserved_representatives=representatives)),flush=True)


if __name__=='__main__':unittest.main()

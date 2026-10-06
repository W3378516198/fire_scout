#!/usr/bin/env python3
"""Instantiate real launch functions with transport stubs; check final parameters."""
import importlib.util
import math
import pathlib
import sys
import tempfile
import types
import yaml
ROOT=pathlib.Path(__file__).resolve().parents[1]
class Action:
    def __init__(self,*args,**kw): self.args=args; self.__dict__.update(kw)
class Declare(Action):
    def __init__(self,name,default_value='',**kw): super().__init__(name=name,default_value=default_value,**kw)
class Configuration:
    def __init__(self,name): self.name=name
    def perform(self,context): return str(context[self.name])
class Description:
    def __init__(self,actions): self.entities=actions
mods={
 'launch':{'LaunchDescription':Description},
 'launch.actions':{'DeclareLaunchArgument':Declare,'OpaqueFunction':Action,'TimerAction':Action,'IncludeLaunchDescription':Action},
 'launch.substitutions':{'LaunchConfiguration':Configuration},
 'launch.launch_description_sources':{'PythonLaunchDescriptionSource':Action},
 'launch_ros.actions':{'Node':Action},
 'ament_index_python.packages':{'get_package_share_directory':lambda name:str(ROOT)},
}
for name,attrs in mods.items():
    m=types.ModuleType(name);m.__dict__.update(attrs);sys.modules[name]=m

def load(filename):
    spec=importlib.util.spec_from_file_location(filename,ROOT/'launch'/filename)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
multi=load('multi_safe_airfar_navigation.launch.py')
wrapper=load('multi_racer_navigation.launch.py')
def defaults(module): return {e.name:e.default_value for e in module.generate_launch_description().entities if isinstance(e,Declare)}
def nodes(actions):
    found=[]
    for a in actions:
        if hasattr(a,'executable'): found.append(a)
        if hasattr(a,'actions'): found.extend(nodes(a.actions))
    return found
for count in [1,2,3,4]:
    context=defaults(multi);context.update(scout_count=str(count),start_fusion='true',start_racer='true',start_mapper='true')
    ns=nodes(multi._make_actions(context))
    planners=[x for x in ns if x.executable=='safe_airfar_like_planner']
    followers=[x for x in ns if x.executable=='safe_airfar_path_follower']
    coord=[x for x in ns if x.executable=='passage_coordinator']
    fusion=[x for x in ns if x.executable=='multi_uav_map_fusion'][0].parameters[0]
    assert fusion['source_snapshot_topics']==[f'/scout{j+1}/map/radar_snapshot' for j in range(count)]
    assert fusion['fused_snapshot_topics']==[f'/scout{j+1}/map/fused_snapshot' for j in range(count)]
    assert len(planners)==len(followers)==count and len(coord)==int(count>1)
    for i,(a,b) in enumerate(zip(planners,followers)):
        p=a.parameters[0];f=b.parameters[0];root=f'/scout{i+1}'
        assert p['path_topic']==f['path_topic']==root+'/planning/global_path'
        assert p['execution_state_topic']==f['execution_state_topic']==root+'/planning/execution_state'
        assert f['passage_scheduler_enabled']==(count>1)
        assert p['preferred_clearance']==.23 and f['max_speed_xy']==1.8
        assert p['candidate_path_count']==3 and p['peer_route_penalty_weight']==0.
        assert p['voxel_size']==f['voxel_size']==.18
        assert p['map_snapshot_topic']==root+'/map/fused_snapshot'
        assert p['local_map_snapshot_topic']==f['map_snapshot_topic']==root+'/map/radar_snapshot'
        assert p['local_map_fallback_enabled'] and p['map_transport']==f['map_transport']=='compact'
        assert p.get('peer_path_topics',[])==[f'/scout{j+1}/planning/global_path' for j in range(i)]
        for v in p.values(): assert v!=() and v!=[],('invalid ROS parameter list',v)
        if count>1:
            assert f['passage_request_topic']==root+'/planning/passage_request'
            assert f['passage_grant_topic']==root+'/planning/passage_grant'
            assert coord[0].parameters[0]['passage_exit_radius']==f['passage_release_distance']
    context['start_passage_coordinator']='false'
    ns=nodes(multi._make_actions(context));assert not any(x.executable=='passage_coordinator' for x in ns)
    assert all(not x.parameters[0]['passage_scheduler_enabled'] for x in ns if x.executable=='safe_airfar_path_follower')
    context['fusion_voxel_size']='0.1'
    assert all(x.parameters[0]['voxel_size']==.1 for x in nodes(multi._make_actions(context)) if x.executable in ['safe_airfar_like_planner','safe_airfar_path_follower','radar_free_space_mapper','multi_uav_map_fusion'])
# Wrapper forwards every essential argument, including compatibility mode and
# the empty resolution override (so edits to YAML are no longer overwritten).
d=defaults(wrapper)
assert d['fusion_voxel_size']=='' and d['racer_task_mode']=='goal_guided'
includes=[e for e in wrapper.generate_launch_description().entities if hasattr(e,'launch_arguments')]
forward=dict(includes[0].launch_arguments)
assert forward['racer_task_mode'].name=='racer_task_mode'
assert forward['start_passage_coordinator'].name=='start_passage_coordinator'
# A custom YAML resolution follows through the default launch, preserving the
# user's edit while using one consistent resolution in every consumer.
with tempfile.TemporaryDirectory() as temp:
    data=yaml.safe_load((ROOT/'config/safe_airfar.yaml').read_text())
    data['/**']['ros__parameters']['voxel_size']=.12
    config=pathlib.Path(temp)/'custom.yaml';config.write_text(yaml.safe_dump(data))
    context=defaults(multi);context.update(params_file=str(config),start_mapper='true',start_fusion='true')
    assert all(x.parameters[0]['voxel_size']==.12 for x in nodes(multi._make_actions(context)) if x.executable in ['safe_airfar_like_planner','safe_airfar_path_follower','radar_free_space_mapper','multi_uav_map_fusion'])
print('v210_launch_test: PASS real 1..4 scout wiring, DAG priority, radius agreement, YAML/CLI resolution, coordinator switch')

#!/usr/bin/env python3
"""Camera-informed task planning. LLM output never reaches a trajectory controller."""
import json
import os
import signal
import threading
import time
import requests
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.exceptions import ROSInterruptException
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from ur3_llm_control_week3.srv import ExecuteSkill, GetSceneState

OBJECTS = {'red_cube', 'yellow_cube', 'blue_cube', 'green_cube', 'purple_cube'}
TARGETS = {'zone_a', 'zone_b', 'zone_c', 'temporary_1'}
FIELDS = {'home': {'skill'}, 'pick': {'skill', 'object'}, 'place': {'skill', 'object', 'target'}}


def parse_plan(text):
    def unique_pairs(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError('Duplicate JSON key: ' + key)
            value[key] = item
        return value
    def invalid_constant(value):
        raise ValueError('Invalid JSON constant: ' + value)
    if not isinstance(text, str):
        raise ValueError('LLM response must be text')
    return json.loads(text, object_pairs_hook=unique_pairs, parse_constant=invalid_constant)


def validate_plan(data, scene):
    if not isinstance(data, dict) or set(data) != {'plan'}:
        return False, "Top-level must contain only 'plan'"
    plan = data['plan']
    if not isinstance(plan, list) or not 1 <= len(plan) <= 30:
        return False, 'Plan must contain 1 to 30 steps'
    try:
        occupancy = {**scene['zones'], **scene['temporary']}
        if set(occupancy) != TARGETS:
            return False, 'Incomplete camera scene'
        objects = scene['objects']
        if set(objects) != OBJECTS:
            return False, 'Incomplete camera objects'
        held = [name for name, obj in objects.items() if obj.get('location') == 'held']
        if len(held) > 1 or any(name in occupancy.values() or objects[name].get('detected') is not False for name in held):
            return False, 'Inconsistent held object in camera scene'
        holding = held[0] if held else None
        for index, step in enumerate(plan):
            if not isinstance(step, dict):
                return False, f'Step {index+1}: expected an object'
            skill = step.get('skill')
            if not isinstance(skill, str) or skill not in FIELDS:
                return False, f'Step {index+1}: invalid skill'
            if set(step) != FIELDS[skill]:
                return False, f'Step {index+1}: invalid fields'
            if skill == 'home':
                if holding is not None or index != len(plan)-1:
                    return False, 'Home must be last, with nothing held'
                continue
            obj = step['object']
            if not isinstance(obj, str) or obj not in OBJECTS:
                return False, f'Step {index+1}: invalid object'
            if skill == 'pick':
                if holding is not None:
                    return False, 'Already holding an object'
                if objects[obj].get('detected') is not True:
                    return False, 'OBJECT_NOT_FOUND: ' + obj
                holding = obj
                for target, occupant in occupancy.items():
                    if occupant == obj:
                        occupancy[target] = 'empty'
            else:
                target = step['target']
                if not isinstance(target, str) or target not in TARGETS:
                    return False, f'Step {index+1}: invalid target'
                if holding != obj:
                    return False, 'Place does not match the held object'
                if occupancy[target] != 'empty':
                    return False, 'Target occupied or unknown: ' + target
                occupancy[target] = obj
                holding = None
        if plan[-1].get('skill') != 'home':
            return False, 'Plan must end with home and nothing held'
    except (KeyError, TypeError, AttributeError):
        return False, 'Invalid camera scene'
    return True, 'VALID'


def step_text(step):
    skill = step['skill']
    if skill == 'home':
        return 'home()'
    if skill == 'pick':
        return f"pick({step['object']})"
    return f"place({step['object']}, {step['target']})"


class Planner(Node):
    def __init__(self):
        super().__init__('llm_task_planner')
        share = get_package_share_directory('ur3_llm_control_week3')
        with open(os.path.join(share, 'config', 'prompt.txt')) as stream:
            self.prompt = stream.read()
        with open(os.path.join(share, 'config', 'student_config.yaml')) as stream:
            student = yaml.safe_load(stream)
        self.api_url = os.environ.get('NINEROUTER_URL', 'http://localhost:20128/v1').rstrip('/')
        self.api_key = os.environ.get('NINEROUTER_KEY', '')
        self.model = os.environ.get('NINEROUTER_MODEL', 'cx/gpt-5.6-luna')
        group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.context.on_shutdown(self.stopping.set)
        self.scene_client = self.create_client(GetSceneState, '/get_scene_state', callback_group=group)
        self.skill_client = self.create_client(ExecuteSkill, '/execute_skill', callback_group=group)
        self.create_subscription(String, '/user_command', self.command, 10, callback_group=group)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.get_logger().info(f"Planner ready for {student['student_name']} ({student['student_id']})")

    def running(self):
        return not self.stopping.is_set() and self.context.ok()

    def ensure_running(self):
        if not self.running():
            raise ROSInterruptException()

    def call(self, client, request, timeout):
        self.ensure_running()
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Service unavailable: ' + client.srv_name)
        self.ensure_running()
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while self.running() and not future.done() and time.monotonic() < deadline:
            self.stopping.wait(.02)
        if not self.running():
            client.remove_pending_request(future)
            raise ROSInterruptException()
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError('Service timeout; stop this task: ' + client.srv_name)
        result = future.result()
        if not result.success:
            raise RuntimeError(result.status)
        return result

    def scene(self):
        data = json.loads(self.call(self.scene_client, GetSceneState.Request(), 8).scene_json)
        if data.get('frame_id') != 'base_link':
            raise RuntimeError('Scene frame must be base_link')
        return data

    def refresh(self, previous_stamp):
        deadline = time.monotonic()+6
        while time.monotonic() < deadline:
            scene = self.scene()
            if scene['stamp'] > previous_stamp:
                return scene
            time.sleep(.1)
        raise RuntimeError('Camera did not produce a fresh frame')

    def call_llm(self, command, scene):
        if not self.api_key:
            raise RuntimeError('NINEROUTER_KEY is not set')
        self.ensure_running()
        done, result = threading.Event(), []
        def request():
            try:
                result.append(requests.post(self.api_url+'/chat/completions',
                    headers={'Content-Type': 'application/json', 'Authorization': 'Bearer '+self.api_key},
                    json={'model': self.model, 'temperature': 0, 'messages': [
                        {'role': 'system', 'content': self.prompt},
                        {'role': 'user', 'content': 'USER COMMAND:\n'+command+'\nCAMERA SCENE:\n'+json.dumps(scene)}]}, timeout=60))
            except Exception as exc:
                result.append(exc)
            finally:
                done.set()
        # Let shutdown cancel a pending HTTP request without delaying the executor.
        threading.Thread(target=request, daemon=True).start()
        while not done.wait(.05):
            self.ensure_running()
        self.ensure_running()
        if isinstance(result[0], Exception):
            raise result[0]
        response = result[0]
        response.raise_for_status()
        return parse_plan(response.json()['choices'][0]['message']['content'])

    def command(self, msg):
        command = msg.data.strip()
        if not command or not self.running():
            return
        if not self.lock.acquire(blocking=False):
            self.get_logger().warning('Robot is busy')
            return
        status = 'TASK FAILED'
        try:
            print('\n'+'='*60+'\nUSER COMMAND:\n'+command, flush=True)
            scene = self.scene()
            print('\nCAMERA SCENE:', flush=True)
            for name, obj in scene['objects'].items():
                print(f"{name:<14} -> {obj['location'] if obj['detected'] else 'not detected'}", flush=True)
            for name, value in {**scene['zones'], **scene['temporary']}.items():
                print(f'{name:<14} -> {value}', flush=True)
            try:
                data = self.call_llm(command, scene)
            except (ValueError, KeyError, TypeError) as exc:
                print('PLAN REJECTED: '+str(exc), flush=True)
                return
            valid, reason = validate_plan(data, scene)
            if not valid:
                print('PLAN REJECTED: '+reason, flush=True)
                return
            print('\nLLM PLAN:', flush=True)
            for index, step in enumerate(data['plan'], 1):
                print(f'{index}. {step_text(step)}', flush=True)
            # Revalidate against a fresh scene immediately before the first action.
            scene = self.refresh(scene['stamp'])
            valid, reason = validate_plan(data, scene)
            if not valid:
                print('PLAN REJECTED: '+reason, flush=True)
                return
            print('\nPLAN VALIDATED\n\nEXECUTION:', flush=True)
            for step in data['plan']:
                text = step_text(step)
                request = ExecuteSkill.Request(skill=step['skill'], object=step.get('object', ''), target=step.get('target', ''))
                try:
                    self.call(self.skill_client, request, 180)
                    if step['skill'] in ('pick', 'place'):
                        scene = self.refresh(scene['stamp'])
                    print(f'{text:.<42} SUCCESS', flush=True)
                except Exception:
                    if self.running():
                        print(f'{text:.<42} FAILED', flush=True)
                    raise
            status = 'TASK SUCCESS'
        except Exception as exc:
            if self.running():
                print('REASON: '+str(exc), flush=True)
        finally:
            try:
                if self.running():
                    print('\n'+status+'\n'+'='*60, flush=True)
                    self.status_pub.publish(String(data=status))
            except Exception:
                if self.running():
                    raise
            finally:
                self.lock.release()


def main():
    rclpy.init()
    node = Planner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    signal.signal(signal.SIGHUP, lambda *_: rclpy.try_shutdown())
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if node.context.ok():
            raise
    finally:
        node.stopping.set()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

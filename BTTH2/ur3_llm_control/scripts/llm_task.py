#!/usr/bin/env python3

import json
import os
import threading
import time

import requests
import yaml
import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from ur3_llm_control.srv import ExecuteSkill


# ============================================================
# WHITELIST
# ============================================================

ALLOWED_SKILLS = {
    "pick",
    "place",
    "home",
}

ALLOWED_OBJECTS = {
    "red_cube",
    "yellow_cube",
    "blue_cube",
}

ALLOWED_ZONES = {
    "zone_a",
    "zone_b",
    "zone_c",
}


# ============================================================
# STUDENT-ID MAPPING
# ============================================================

PERMUTATIONS = {
    0: {
        "zone_a": "red_cube",
        "zone_b": "yellow_cube",
        "zone_c": "blue_cube",
    },
    1: {
        "zone_a": "red_cube",
        "zone_b": "blue_cube",
        "zone_c": "yellow_cube",
    },
    2: {
        "zone_a": "yellow_cube",
        "zone_b": "red_cube",
        "zone_c": "blue_cube",
    },
    3: {
        "zone_a": "yellow_cube",
        "zone_b": "blue_cube",
        "zone_c": "red_cube",
    },
    4: {
        "zone_a": "blue_cube",
        "zone_b": "red_cube",
        "zone_c": "yellow_cube",
    },
    5: {
        "zone_a": "blue_cube",
        "zone_b": "yellow_cube",
        "zone_c": "red_cube",
    },
}


class LLMTaskPlanner(Node):

    def __init__(self):
        super().__init__("llm_task_planner")

        self.callback_group = ReentrantCallbackGroup()
        self.task_lock = threading.Lock()

        # ====================================================
        # PACKAGE PATH
        # ====================================================

        share_dir = get_package_share_directory(
            "ur3_llm_control"
        )

        prompt_path = os.path.join(
            share_dir,
            "config",
            "prompt.txt"
        )

        student_path = os.path.join(
            share_dir,
            "config",
            "student_config.yaml"
        )

        # ====================================================
        # LOAD PROMPT
        # ====================================================

        with open(
            prompt_path,
            "r",
            encoding="utf-8"
        ) as f:
            self.system_prompt = f.read()

        # ====================================================
        # LOAD STUDENT CONFIG
        # ====================================================

        with open(
            student_path,
            "r",
            encoding="utf-8"
        ) as f:
            config = yaml.safe_load(f)

        self.student_name = str(
            config["student_name"]
        )

        self.student_id = str(
            config["student_id"]
        )

        # Hai chữ số cuối
        self.xx = int(
            self.student_id[-2:]
        )

        # P = XX mod 6
        self.p = self.xx % 6

        self.student_mapping = (
            PERMUTATIONS[self.p]
        )

        # ====================================================
        # ADD STUDENT CONTEXT TO LLM PROMPT
        # ====================================================

        student_context = f"""

============================================================
STUDENT INFORMATION
============================================================

Student name:
{self.student_name}

Student ID:
{self.student_id}

The last two digits are:
XX = {self.xx}

P = XX mod 6 = {self.p}

For THIS student, the required arrangement is:

Zone A -> {self.student_mapping["zone_a"]}
Zone B -> {self.student_mapping["zone_b"]}
Zone C -> {self.student_mapping["zone_c"]}

If the user asks:

Arrange all objects according to my student ID.

or expresses the same meaning in Vietnamese or English,
generate a complete plan which:

1. Moves the object assigned to Zone A into zone_a.
2. Moves the object assigned to Zone B into zone_b.
3. Moves the object assigned to Zone C into zone_c.
4. Calls home exactly once at the end.

Use ONLY pick, place and home.

Do not output calculations or explanations.
Return only the JSON plan.
"""

        self.system_prompt += student_context

        # ====================================================
        # 9ROUTER
        # ====================================================

        self.api_url = os.environ.get(
            "NINEROUTER_URL",
            "http://localhost:20128/v1"
        ).rstrip("/")

        self.api_key = os.environ.get(
            "NINEROUTER_KEY",
            ""
        )

        self.model = os.environ.get(
            "NINEROUTER_MODEL",
            "cx/gpt-5.6-luna"
        )

        # ====================================================
        # SKILL CLIENT
        # ====================================================

        self.skill_client = self.create_client(
            ExecuteSkill,
            "/execute_skill",
            callback_group=self.callback_group
        )

        # ====================================================
        # COMMAND SUBSCRIBER
        # ====================================================

        self.command_sub = self.create_subscription(
            String,
            "/user_command",
            self.command_callback,
            10,
            callback_group=self.callback_group
        )

        # ====================================================
        # STARTUP INFORMATION
        # ====================================================

        self.get_logger().info(
            "LLM Task Planner ready."
        )

        self.get_logger().info(
            f"Model: {self.model}"
        )

        self.get_logger().info(
            f"Student: {self.student_name}"
        )

        self.get_logger().info(
            f"Student ID: {self.student_id}"
        )

        self.get_logger().info(
            f"XX = {self.xx}"
        )

        self.get_logger().info(
            f"P = {self.xx} mod 6 = {self.p}"
        )

        self.get_logger().info(
            "Student mapping:"
        )

        self.get_logger().info(
            f"  Zone A -> "
            f"{self.student_mapping['zone_a']}"
        )

        self.get_logger().info(
            f"  Zone B -> "
            f"{self.student_mapping['zone_b']}"
        )

        self.get_logger().info(
            f"  Zone C -> "
            f"{self.student_mapping['zone_c']}"
        )

        self.get_logger().info(
            "Waiting for commands on /user_command"
        )

    # ========================================================
    # COMMAND CALLBACK
    # ========================================================

    def command_callback(self, msg):

        command = msg.data.strip()

        if not command:
            self.get_logger().error(
                "Empty command."
            )
            return

        if not self.task_lock.acquire(
            blocking=False
        ):
            self.get_logger().warning(
                "Robot is busy."
            )
            return

        try:
            self.process_command(command)

        finally:
            self.task_lock.release()

    # ========================================================
    # PROCESS COMMAND
    # ========================================================

    def process_command(self, command):

        print()
        print("=" * 60)

        print("USER COMMAND:")
        print(command)

        # ====================================================
        # CALL LLM
        # ====================================================

        try:
            plan_data = self.call_llm(
                command
            )

        except Exception as exc:

            print()
            print("LLM ERROR:")
            print(exc)

            print()
            print("TASK FAILED")
            print("=" * 60)

            return

        # ====================================================
        # VALIDATOR
        # ====================================================

        valid, reason = self.validate_plan(
            plan_data
        )

        # ====================================================
        # PRINT PLAN
        # ====================================================

        print()
        print("LLM PLAN:")

        if (
            isinstance(plan_data, dict)
            and isinstance(
                plan_data.get("plan"),
                list
            )
        ):

            for i, step in enumerate(
                plan_data["plan"],
                start=1
            ):
                print(
                    f"{i}. "
                    f"{self.step_to_text(step)}"
                )

        else:
            print(plan_data)

        # ====================================================
        # REJECT INVALID PLAN
        # ====================================================

        if not valid:

            print()
            print("PLAN REJECTED")
            print(
                f"REASON: {reason}"
            )

            print()
            print("TASK FAILED")

            print("=" * 60)

            return

        print()
        print("PLAN VALIDATED")

        # ====================================================
        # EXECUTE
        # ====================================================

        success = self.execute_plan(
            plan_data["plan"]
        )

        print()

        if success:
            print("TASK SUCCESS")
        else:
            print("TASK FAILED")

        print("=" * 60)

    # ========================================================
    # HUMAN READABLE STEP
    # ========================================================

    def step_to_text(self, step):

        skill = step.get(
            "skill",
            ""
        )

        if skill == "pick":

            return (
                f"pick("
                f"{step.get('object', '')}"
                f")"
            )

        if skill == "place":

            return (
                f"place("
                f"{step.get('object', '')}, "
                f"{step.get('zone', '')}"
                f")"
            )

        if skill == "home":

            return "home()"

        return str(step)

    # ========================================================
    # CALL LLM
    # ========================================================

    def call_llm(self, command):

        if not self.api_key:
            raise RuntimeError(
                "NINEROUTER_KEY is not set."
            )

        endpoint = (
            self.api_url
            + "/chat/completions"
        )

        headers = {
            "Content-Type":
                "application/json",

            "Authorization":
                f"Bearer {self.api_key}",
        }

        payload = {
            "model": self.model,

            "temperature": 0,

            "messages": [
                {
                    "role": "system",
                    "content":
                        self.system_prompt
                },
                {
                    "role": "user",
                    "content":
                        command
                }
            ]
        }

        response = requests.post(
            endpoint,
            headers=headers,
            json=payload,
            timeout=60
        )

        response.raise_for_status()

        result = response.json()

        content = (
            result["choices"][0]
            ["message"]["content"]
        )

        return self.parse_json(
            content
        )

    # ========================================================
    # JSON PARSER
    # ========================================================

    def parse_json(self, content):

        if not isinstance(
            content,
            str
        ):
            raise ValueError(
                "LLM response is not text."
            )

        text = content.strip()

        # Remove Markdown fences if model adds them
        if text.startswith("```"):

            lines = text.splitlines()

            lines = lines[1:]

            if (
                lines
                and
                lines[-1].strip()
                == "```"
            ):
                lines = lines[:-1]

            text = "\n".join(
                lines
            ).strip()

        start = text.find("{")
        end = text.rfind("}")

        if (
            start == -1
            or end == -1
        ):
            raise ValueError(
                "No JSON object found."
            )

        return json.loads(
            text[start:end + 1]
        )

    # ========================================================
    # PLAN VALIDATOR
    # ========================================================

    def validate_plan(
        self,
        data
    ):

        # ====================================================
        # TOP LEVEL
        # ====================================================

        if not isinstance(
            data,
            dict
        ):
            return (
                False,
                "Response is not a JSON object."
            )

        if set(
            data.keys()
        ) != {"plan"}:

            return (
                False,
                "Top-level JSON must contain only 'plan'."
            )

        plan = data["plan"]

        if not isinstance(
            plan,
            list
        ):
            return (
                False,
                "'plan' is not a list."
            )

        if len(plan) == 0:
            return (
                False,
                "Plan is empty."
            )

        if len(plan) > 20:
            return (
                False,
                "Plan is too long."
            )

        # Logical robot state
        holding = None

        # ====================================================
        # VALIDATE EACH STEP
        # ====================================================

        for index, step in enumerate(
            plan
        ):

            number = index + 1

            if not isinstance(
                step,
                dict
            ):
                return (
                    False,
                    f"Step {number} is invalid."
                )

            skill = step.get(
                "skill"
            )

            # =================================================
            # SKILL
            # =================================================

            if skill not in ALLOWED_SKILLS:

                return (
                    False,
                    f"Step {number}: "
                    f"INVALID_SKILL '{skill}'"
                )

            # =================================================
            # PICK
            # =================================================

            if skill == "pick":

                if set(
                    step.keys()
                ) != {
                    "skill",
                    "object"
                }:

                    return (
                        False,
                        f"Step {number}: "
                        "invalid pick format."
                    )

                obj = step.get(
                    "object"
                )

                if obj not in ALLOWED_OBJECTS:

                    return (
                        False,
                        f"Step {number}: "
                        f"INVALID_OBJECT '{obj}'"
                    )

                if holding is not None:

                    return (
                        False,
                        f"Step {number}: "
                        "robot is already holding an object."
                    )

                holding = obj

            # =================================================
            # PLACE
            # =================================================

            elif skill == "place":

                if set(
                    step.keys()
                ) != {
                    "skill",
                    "object",
                    "zone"
                }:

                    return (
                        False,
                        f"Step {number}: "
                        "invalid place format."
                    )

                obj = step.get(
                    "object"
                )

                zone = step.get(
                    "zone"
                )

                if obj not in ALLOWED_OBJECTS:

                    return (
                        False,
                        f"Step {number}: "
                        f"INVALID_OBJECT '{obj}'"
                    )

                if zone not in ALLOWED_ZONES:

                    return (
                        False,
                        f"Step {number}: "
                        f"INVALID_ZONE '{zone}'"
                    )

                if holding != obj:

                    return (
                        False,
                        f"Step {number}: "
                        f"robot is not holding "
                        f"'{obj}'"
                    )

                holding = None

            # =================================================
            # HOME
            # =================================================

            elif skill == "home":

                if set(
                    step.keys()
                ) != {"skill"}:

                    return (
                        False,
                        f"Step {number}: "
                        "home must contain only skill."
                    )

                if holding is not None:

                    return (
                        False,
                        f"Step {number}: "
                        "cannot home while holding object."
                    )

                if index != len(plan) - 1:

                    return (
                        False,
                        f"Step {number}: "
                        "home must be the final skill."
                    )

        # ====================================================
        # FINAL CONDITIONS
        # ====================================================

        if holding is not None:

            return (
                False,
                "Plan ends while holding an object."
            )

        if plan[-1].get(
            "skill"
        ) != "home":

            return (
                False,
                "Plan must end with home."
            )

        return (
            True,
            "VALID"
        )

    # ========================================================
    # EXECUTE PLAN
    # ========================================================

    def execute_plan(
        self,
        plan
    ):

        if not self.skill_client.wait_for_service(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                "/execute_skill unavailable."
            )

            return False

        print()
        print("EXECUTION:")

        for step in plan:

            description = (
                self.step_to_text(step)
            )

            request = (
                ExecuteSkill.Request()
            )

            request.skill = (
                step["skill"]
            )

            request.object = (
                step.get(
                    "object",
                    ""
                )
            )

            request.zone = (
                step.get(
                    "zone",
                    ""
                )
            )

            future = (
                self.skill_client
                .call_async(request)
            )

            while (
                rclpy.ok()
                and
                not future.done()
            ):
                time.sleep(0.05)

            if not future.done():

                print(
                    f"{description} "
                    f"........ FAILED"
                )

                return False

            try:

                response = (
                    future.result()
                )

            except Exception as exc:

                print(
                    f"{description} "
                    f"........ FAILED"
                )

                self.get_logger().error(
                    str(exc)
                )

                return False

            if response.success:

                print(
                    f"{description:<32}"
                    f"SUCCESS"
                )

            else:

                print(
                    f"{description:<32}"
                    f"{response.status}"
                )

                return False

        return True


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = LLMTaskPlanner()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()

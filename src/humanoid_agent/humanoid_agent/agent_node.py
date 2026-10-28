"""agent_node: ROS2 obal okolo agent_core.

API:   ros2 action send_goal /agent/task humanoid_interfaces/action/Task "{task: 'chod k modremu boxu'}" --feedback
       (skratka v terminali:  chappie_task "chod k modremu boxu")

Robot sa hybe VYLUCNE cez C++ skills (/skills/execute); tento uzol len posiela snimky AI a vykonava jej plan.
Kluce: export GEMINI_API_KEY=...  alebo  export ANTHROPIC_API_KEY=...
"""
import io
import json
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from humanoid_interfaces.action import Skill, Task

from humanoid_agent.agent_core import Agent, draw_ruler
from humanoid_agent.llm_backends import make_llm


class Cancelled(Exception):
    pass


class RosRobot:
    """vykonanie skillu cez action server /skills/execute (C++)"""

    def __init__(self, node, client, cancel_flag):
        self.node, self.client, self.cancel_flag = node, client, cancel_flag

    def _wait(self, fut, timeout):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise TimeoutError("cakanie na skill vyprsalo")
            time.sleep(0.02)

    def execute(self, skill, value):
        if not self.client.wait_for_server(timeout_sec=5.0):
            return {"success": False, "status": "no_server", "message": "Nebezi /skills/execute (motion_skills).",
                    "value_done": 0.0}
        goal = Skill.Goal()
        goal.skill = skill
        goal.value = float(value)
        send = self.client.send_goal_async(goal)
        self._wait(send, 10.0)
        gh = send.result()
        if not gh.accepted:
            return {"success": False, "status": "rejected", "message": "Cielovy skill nebol prijaty.", "value_done": 0.0}
        res = gh.get_result_async()
        cancelled = False
        t0 = time.time()
        while not res.done():
            if self.cancel_flag() and not cancelled:
                gh.cancel_goal_async()
                cancelled = True
            if time.time() - t0 > 180.0:
                gh.cancel_goal_async()
                return {"success": False, "status": "timeout", "message": "Skill trva prilis dlho.", "value_done": 0.0}
            time.sleep(0.02)
        r = res.result().result
        return {"success": bool(r.success), "status": r.status, "message": r.message, "value_done": float(r.value_done)}


class RosCamera:
    """snimka cez sluzbu /camera/snapshot (C++), nacita sa ulozeny subor"""

    def __init__(self, node, client):
        self.node, self.client = node, client
        self.mime = "image/png"

    def snapshot(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Nebezi sluzba /camera/snapshot (camera_snapshot).")
        fut = self.client.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > 10.0:
                raise RuntimeError("snimka vyprsala")
            time.sleep(0.02)
        resp = fut.result()
        if not resp.success:
            raise RuntimeError(resp.message)
        data = open(resp.message, "rb").read()
        try:                                  # mensi subor pre AI, ak je k dispozicii PIL
            from PIL import Image
            im = Image.open(io.BytesIO(data)).convert("RGB")
            im.thumbnail((768, 768))          # mensia snimka = rychlejsia odpoved AI
            if os.environ.get("CHAPPIE_RULER", "1") != "0":
                im = draw_ruler(im)           # pravitko 0-100 dole (TT_PATCH)
            out = io.BytesIO()
            im.save(out, format="JPEG", quality=85)
            self.mime = "image/jpeg"
            return out.getvalue()
        except Exception:
            self.mime = "image/png"
            return data


class RosLidar:
    """posledne vzdialenosti z lidaru (topic /lidar/distances, JSON od motion_skills)"""

    def __init__(self):
        self.data, self.t = None, 0.0

    def update(self, msg):
        try:
            self.data, self.t = json.loads(msg.data), time.time()
        except ValueError:
            pass

    def read(self, max_age=1.5):
        if self.data is None or time.time() - self.t > max_age:
            return None
        return self.data


class AgentNode(Node):
    def __init__(self):
        super().__init__("agent_node")
        self.provider = self.declare_parameter("provider", "gemini").value
        self.model = self.declare_parameter("model", "").value
        self.max_iterations = int(self.declare_parameter("max_iterations", 10).value)
        self.max_actions = int(self.declare_parameter("max_actions", 20).value)
        self.max_distance = float(self.declare_parameter("max_total_distance", 40.0).value)
        self.runs_dir = self.declare_parameter("runs_dir", os.path.expanduser("~/chappie_runs")).value
        cb = ReentrantCallbackGroup()
        self.skill_client = ActionClient(self, Skill, "/skills/execute", callback_group=cb)
        self.snap_client = self.create_client(Trigger, "/camera/snapshot", callback_group=cb)
        self.catalog = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/skills/catalog", self._on_catalog, qos, callback_group=cb)
        self.lidar = RosLidar()
        self.create_subscription(String, "/lidar/distances", self.lidar.update, 10, callback_group=cb)
        self.server = ActionServer(
            self, Task, "/agent/task", execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.busy = False
        self.get_logger().info("Agent pripraveny: /agent/task (AI: %s %s)" % (self.provider, self.model or "(predvoleny model)"))

    def _on_catalog(self, msg):
        try:
            self.catalog = json.loads(msg.data)
        except ValueError:
            self.get_logger().error("Neplatny /skills/catalog")

    def _execute(self, goal_handle):
        result = Task.Result()
        if self.busy:
            goal_handle.abort()
            result.success, result.status, result.summary = False, "busy", "Agent uz rieši inú úlohu."
            return result
        self.busy = True
        try:
            t0 = time.time()
            while self.catalog is None and time.time() - t0 < 10.0:
                time.sleep(0.1)
            if self.catalog is None:
                raise RuntimeError("Nemam /skills/catalog (bezi motion_skills?)")
            llm = make_llm(self.provider, self.model)
            cancel = lambda: goal_handle.is_cancel_requested
            run_dir = os.path.join(self.runs_dir, time.strftime("%Y%m%d_%H%M%S"))
            agent = Agent(RosRobot(self, self.skill_client, cancel), RosCamera(self, self.snap_client), llm,
                          self.catalog, max_iterations=self.max_iterations, max_actions=self.max_actions,
                          max_total_distance=self.max_distance, run_dir=run_dir, lidar=self.lidar,
                          log=lambda s: self.get_logger().info(s), should_cancel=cancel)

            def feedback(state, it):
                fb = Task.Feedback()
                fb.state, fb.iteration = state, int(it)
                goal_handle.publish_feedback(fb)

            r = agent.run(goal_handle.request.task, feedback)
        except Exception as e:      # chyba kluca, siete, ...
            self.get_logger().error("Agent zlyhal: %s" % e)
            r = {"success": False, "status": "error", "summary": str(e), "steps": 0, "iterations": 0}
        finally:
            self.busy = False
        result.success, result.status = bool(r["success"]), r["status"]
        result.summary, result.steps, result.iterations = r["summary"], int(r["steps"]), int(r["iterations"])
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = AgentNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# LIDAR_PATCH

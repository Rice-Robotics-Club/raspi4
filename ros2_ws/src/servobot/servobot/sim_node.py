from omni.isaac.lab.app import AppLauncher

app_launcher = AppLauncher()
simulation_app = app_launcher.app

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import typing
import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
import copy
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

from math import pi
import typing


SERVOBOT_CFG = ArticulationCfg(
    prim_path="/Robot",
    spawn=sim_utils.UrdfFileCfg(
        asset_path="/home/lopatoj/raspi4/ros2_ws/src/servobot/description/catbot.xacro",
        fix_base=False,
        make_instanceable=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={
            "FL_Hip": 0.0,
            "FL_TopLeg": 0.0,
            "FL_BotLeg": 0.0,
            "FR_Hip": 0.0,
            "FR_TopLeg": 0.0,
            "FR_BotLeg": 0.0,
            "BL_Hip": 0.0,
            "BL_TopLeg": 0.0,
            "BL_BotLeg": 0.0,
            "BR_Hip": 0.0,
            "BR_TopLeg": 0.0,
            "BR_BotLeg": 0.0,
        },
    ),
    actuators={
        "FL_Hip": ImplicitActuatorCfg(
            joint_names_expr=["FL_Hip"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "FL_TopLeg": ImplicitActuatorCfg(
            joint_names_expr=["FL_TopLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "FL_BotLeg": ImplicitActuatorCfg(
            joint_names_expr=["FL_BotLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "FR_Hip": ImplicitActuatorCfg(
            joint_names_expr=["FR_Hip"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "FR_TopLeg": ImplicitActuatorCfg(
            joint_names_expr=["FR_TopLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "FR_BotLeg": ImplicitActuatorCfg(
            joint_names_expr=["FR_BotLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "BL_Hip": ImplicitActuatorCfg(
            joint_names_expr=["BL_Hip"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "BL_TopLeg": ImplicitActuatorCfg(
            joint_names_expr=["BL_TopLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "BL_BotLeg": ImplicitActuatorCfg(
            joint_names_expr=["BL_BotLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "BR_Hip": ImplicitActuatorCfg(
            joint_names_expr=["BR_Hip"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "BR_TopLeg": ImplicitActuatorCfg(
            joint_names_expr=["BR_TopLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "BR_BotLeg": ImplicitActuatorCfg(
            joint_names_expr=["BR_BotLeg"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
    },
)

class SimNode(Node):
    def __init__(self):
        super().__init__("multi_servo_node")
        
        self.servo_angles = self.create_subscription(
            Float64MultiArray, "servo_angles", self.servo_angles_callback, 10
        )
        
        sim_cfg = sim_utils.SimulationCfg()
        self.sim = SimulationContext(sim_cfg)
        self.sim.set_camera_view((2.5, 0.0, 4.0), (0.0, 0.0, 0.0), "/OmniverseKit_Persp")
        
        servobot_cfg = SERVOBOT_CFG
        servobot_cfg.prim_path = "/World/Robot"
        self.robot = Articulation(servobot_cfg)
        
        cfg = sim_utils.GroundPlaneCfg()
        cfg.func("/World/defaultGroundPlane", cfg)
        
        cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
        cfg.func("/World/Light", cfg)

        self.angles = [0.0] * 12
        self.sim_dt = self.sim.get_physics_dt()
        
        self.create_timer(self.sim_dt, self.timer_callback)
        
    def timer_callback(self):
        angles2 = []
        
        for i in range(3):
            angles2.append(self.angles[i])
            angles2.append(self.angles[i + 3])
            angles2.append(self.angles[i + 6])
            angles2.append(self.angles[i + 9])
            
        angles2[8] += pi/2 + .2
        angles2[9] -= pi/2.5 + .2
        angles2[10] += pi/2.5 + .2
        angles2[11] -= pi/2 + .2
        
        self.robot.set_joint_position_target(torch.tensor([angles2], device=self.sim.device))
        # # -- write data to sim
        self.robot.write_data_to_sim()
        # Perform step
        self.sim.step()
        self.robot.update(self.sim_dt)
        
    def servo_angles_callback(self, msg: Float64MultiArray):
        self.angles = msg.data
        

def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

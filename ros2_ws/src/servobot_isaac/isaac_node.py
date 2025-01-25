from omni.isaac.lab.app import AppLauncher

app_launcher = AppLauncher()
simulation_app = app_launcher.app

import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import omni

import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

SERVOBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="meshes/catbot.xacro",
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

class IsaacNode(Node):
    def __init__(self):
        super().__init__("isaac_node")
        
        self.servo_angles = self.create_subscription(
            Float64MultiArray, "servo_angles", self.servo_angles_callback, 10
        )
        
        self.angles = [0.0] * 12
        
        sim_cfg = sim_utils.SimulationCfg()
        sim = SimulationContext(sim_cfg)
        sim.set_camera_view((2.5, 0.0, 4.0), (0.0, 0.0, 0.0), "/OmniverseKit_Persp")
        
        cfg = sim_utils.GroundPlaneCfg()
        cfg.func("/World/defaultGroundPlane", cfg)
        cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
        cfg.func("/World/Light", cfg)

        servobot_cfg = SERVOBOT_CFG
        servobot_cfg.prim_path = "/World/Robot"
        robot = Articulation(cfg=servobot_cfg)
        
        sim.reset()
        
        sim_dt = sim.get_physics_dt()
        t = 0.0
        
        while simulation_app.is_running():
            robot.set_joint_position_target(torch.tensor([self.angles], device=sim.device))
            robot.write_data_to_sim()
            sim.step()
            t += sim_dt
            robot.update(sim_dt)
            
    def servo_angles_callback(self, msg: Float64MultiArray):
        """sets servo angles from float array stored in message

        Args:
            msg (Float64MultiArray): float array interface for /servo_angles topic
        """
        
        for i in range(3):
            self.angles += [msg.data[i + 3*j] for j in range(4)]

def main():
    """Main function."""
    rclpy.init()
    isaac_node = IsaacNode()
    rclpy.spin(isaac_node)
    isaac_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
